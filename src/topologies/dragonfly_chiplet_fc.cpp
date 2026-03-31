#include "dragonfly_chiplet_fc.h"

// ============================================================
// NodeInCGFC
// ============================================================

NodeInCGFC::NodeInCGFC(int k_chiplet, int vc_num, int buffer_size, Channel internal_channel,
                       Channel external_channel)
    // radix = (k*k - 1) internal ports + 1 external port
    : Node(k_chiplet * k_chiplet, vc_num, buffer_size),
      ext_in_buffer_(in_buffers_[k_chiplet * k_chiplet - 1]),
      ext_link_node_(link_nodes_[k_chiplet * k_chiplet - 1]),
      ext_link_buffer_(link_buffers_[k_chiplet * k_chiplet - 1]) {
  cgroup_ = nullptr;
  k_chiplet_ = k_chiplet;
  internal_port_count_ = k_chiplet * k_chiplet - 1;
  node_id_in_cg_ = -1;  // set later in set_node

  // Internal ports: low-latency on-wafer channels
  for (int i = 0; i < internal_port_count_; i++) {
    in_buffers_[i]->channel_ = internal_channel;
  }
  // External port: higher-latency off-wafer channel
  ext_in_buffer_->channel_ = external_channel;
}

void NodeInCGFC::set_node(Chip* cgroup, NodeID id) {
  assert(cgroup != nullptr);
  Node::set_node(cgroup, id);
  cgroup_ = dynamic_cast<CGroupFC*>(chip_);
  node_id_in_cg_ = id.node_id;
}

int NodeInCGFC::internal_port_to(int peer_node_id) const {
  // Ports 0..(k*k-2) reach peers in order, skipping this node's own id.
  // peer_node_id < node_id_in_cg_ -> port index = peer_node_id
  // peer_node_id > node_id_in_cg_ -> port index = peer_node_id - 1
  assert(peer_node_id != node_id_in_cg_);
  return (peer_node_id < node_id_in_cg_) ? peer_node_id : peer_node_id - 1;
}

// ============================================================
// CGroupFC
// ============================================================

CGroupFC::CGroupFC(int k_chiplet, int cgroup_radix, int vc_num, int buffer_size,
                   Channel internal_channel, Channel external_channel)
    : num_chiplets_(number_nodes_), cgroup_id_(chip_id_) {
  k_node_ = k_chiplet;
  cgroup_radix_ = cgroup_radix;
  num_chiplets_ = k_chiplet * k_chiplet;
  number_cores_ = num_chiplets_;
  wgroup_id_ = 0;
  dragonfly_ = nullptr;
  nodes_.reserve(num_chiplets_);
  for (int i = 0; i < num_chiplets_; i++) {
    nodes_.push_back(new NodeInCGFC(k_chiplet, vc_num, buffer_size,
                                    internal_channel, external_channel));
  }
}

CGroupFC::~CGroupFC() {
  for (auto n : nodes_) delete n;
  nodes_.clear();
}

void CGroupFC::set_chip(System* dragonfly, int cgroup_id) {
  Chip::set_chip(dragonfly, cgroup_id);
  dragonfly_ = dynamic_cast<DragonflyChipletFC*>(system_);
  wgroup_id_ = cgroup_id / dragonfly_->cgroup_per_wgroup_;

  // Wire up the fully-connected internal network.
  // Each pair (i, j) gets one bidirectional link.
  // Node i uses port internal_port_to(j) to reach node j, and vice-versa.
  //
  // Trade-off note: this is O(N^2) links where N = k*k.
  // For k=2 (4 chiplets): 6 links.  For k=4 (16 chiplets): 120 links.
  // Compare with 2D-mesh: 2*k*(k-1) = 24 links for k=4.
  for (int i = 0; i < num_chiplets_; i++) {
    NodeInCGFC* node_i = get_node(i);
    for (int j = i + 1; j < num_chiplets_; j++) {
      NodeInCGFC* node_j = get_node(j);
      int port_i = node_i->internal_port_to(j);
      int port_j = node_j->internal_port_to(i);
      // node_i port_i <-> node_j port_j
      node_i->link_nodes_[port_i] = NodeID(j, cgroup_id);
      node_i->link_buffers_[port_i] = node_j->in_buffers_[port_j];
      node_j->link_nodes_[port_j] = NodeID(i, cgroup_id);
      node_j->link_buffers_[port_j] = node_i->in_buffers_[port_i];
    }
  }
}

// ============================================================
// DragonflyChipletFC
// ============================================================

DragonflyChipletFC::DragonflyChipletFC() : num_cgroup_(num_chips_), cgroups_(chips_) {
  read_config();

  num_chiplets_per_cg_ = k_node_in_CG_ * k_node_in_CG_;
  num_nodes_per_cg_ = num_chiplets_per_cg_;

  // ---- Port budget analysis ----
  // Each chiplet has (k*k) total ports:
  //   (k*k - 1) internal FC ports
  //   1 external port
  // So the C-group has exactly k*k external ports total (one per chiplet).
  // This is the key trade-off: the FC variant has FEWER external ports than
  // the 2D-mesh variant (which had 4*k - 4 edge ports for a k*k mesh).
  //
  // Example: k=4
  //   2D-mesh: cgroup_radix = 4*4 - 4 = 12 external ports
  //   FC:      cgroup_radix = 4*4     = 16 external ports
  //
  // Interestingly for k=4 the FC variant has MORE external ports,
  // but each chiplet pays 15 internal ports vs 4 in the mesh case.
  // For large k, external port count grows as k^2 (FC) vs 4k (mesh),
  // which is better, but internal wiring cost also grows as k^2.
  cgroup_radix_ = num_chiplets_per_cg_;  // one external port per chiplet

  // Replicate the Dragonfly port split from DragonflyChiplet:
  //   l_ports : g_ports = 2:1 approximately, matching global-local balance.
  l_ports_per_cg_ = cgroup_radix_ / 3 * 2 - 1;
  g_ports_per_cg_ = cgroup_radix_ - l_ports_per_cg_;
  cgroup_per_wgroup_ = l_ports_per_cg_ + 1;
  g_ports_per_wg_ = g_ports_per_cg_ * cgroup_per_wgroup_;
  num_wgroup_ = g_ports_per_wg_ + 1;
  num_cgroup_ = num_wgroup_ * cgroup_per_wgroup_;
  num_cores_ = num_cgroup_ * num_nodes_per_cg_;
  num_nodes_ = num_cores_;

  std::cout << "[DragonflyChipletFC]"
            << "  k=" << k_node_in_CG_
            << "  chiplets_per_cg=" << num_chiplets_per_cg_
            << "  internal_ports_per_chiplet=" << (num_chiplets_per_cg_ - 1)
            << "  external_ports_per_cg=" << cgroup_radix_
            << "  l=" << l_ports_per_cg_
            << "  g=" << g_ports_per_cg_
            << "  cg_per_wg=" << cgroup_per_wgroup_
            << "  num_wg=" << num_wgroup_
            << "  num_cg=" << num_cgroup_
            << "  num_cores=" << num_cores_ << std::endl;

  // Build port_node_map: port_id -> node_id
  // In the FC variant each chiplet owns exactly one external port,
  // so the mapping is simply port_id == node_id.
  for (int i = 0; i < cgroup_radix_; i++) {
    port_node_map_[i] = i;
  }

  // Instantiate C-groups
  cgroups_.reserve(num_cgroup_);
  for (int cg_id = 0; cg_id < num_cgroup_; cg_id++) {
    cgroups_.push_back(new CGroupFC(k_node_in_CG_, cgroup_radix_, param->vc_number,
                                    param->buffer_size, internal_channel_, external_channel_));
    cgroups_[cg_id]->set_chip(this, cg_id);
  }

  connect_local();
  connect_global();
}

DragonflyChipletFC::~DragonflyChipletFC() {
  for (auto cg : cgroups_) delete cg;
  cgroups_.clear();
}

void DragonflyChipletFC::read_config() {
  k_node_in_CG_ = param->params_ptree.get<int>("Network.k_node", 4);
  algorithm_ = param->params_ptree.get<std::string>("Network.routing_algorithm", "MIN");
  int internal_bandwidth = param->params_ptree.get<int>("Network.internal_bandwidth", 1);
  int external_latency = param->params_ptree.get<int>("Network.external_latency", 4);
  internal_channel_ = Channel(internal_bandwidth, 1);
  external_channel_ = Channel(1, external_latency);
  mis_routing_ = param->params_ptree.get<bool>("Network.mis_routing", false);
}

// ---- Local connectivity ----
// Same labeling scheme as DragonflyChiplet: within a W-group, C-groups are
// all-to-all connected using their external ports.
// Port assignment follows the same pattern (highest ports connect to lower
// numbered C-groups, lowest ports connect to higher numbered C-groups).
void DragonflyChipletFC::connect_local() {
  for (int wg_id = 0; wg_id < num_wgroup_; wg_id++) {
    // Step 1: connect adjacent C-groups (i, i+1)
    for (int i = 0; i < cgroup_per_wgroup_ - 1; i++) {
      int node_id_1 = port_node_map_.at(cgroup_radix_ - 1);
      int node_id_2 = port_node_map_.at(0);
      Port port1 = get_port(wg_id * cgroup_per_wgroup_ + i,     node_id_1);
      Port port2 = get_port(wg_id * cgroup_per_wgroup_ + i + 1, node_id_2);
      Port::connect(port1, port2);
      if (wg_id == 0) {
        local_link_map_[{i, i + 1}] = node_id_1;
        local_link_map_[{i + 1, i}] = node_id_2;
      }
    }
    // Step 2: connect non-adjacent C-group pairs (i, j) for j >= i+2
    for (int i = 0; i < cgroup_per_wgroup_ - 2; i++) {
      for (int j = i + 2; j < cgroup_per_wgroup_; j++) {
        int node_id_1 = port_node_map_.at(cgroup_radix_ - (j - i));
        int node_id_2 = port_node_map_.at(i + 1);
        Port port1 = get_port(wg_id * cgroup_per_wgroup_ + i, node_id_1);
        Port port2 = get_port(wg_id * cgroup_per_wgroup_ + j, node_id_2);
        Port::connect(port1, port2);
        if (wg_id == 0) {
          local_link_map_[{i, j}] = node_id_1;
          local_link_map_[{j, i}] = node_id_2;
        }
      }
    }
  }
}

// ---- Global connectivity ----
// Identical scheme to DragonflyChiplet: all W-groups are all-to-all connected.
void DragonflyChipletFC::connect_global() {
  // Step 1: connect adjacent W-groups (i, i+1)
  for (int i = 0; i < num_wgroup_ - 1; i++) {
    int cg_id_in_wg_1, node_id_1, cg_id_in_wg_2, node_id_2;
    std::tie(cg_id_in_wg_1, node_id_1) = global_port_id_to_port_id(g_ports_per_wg_ - 1);
    std::tie(cg_id_in_wg_2, node_id_2) = global_port_id_to_port_id(0);
    Port port1 = get_port(i * cgroup_per_wgroup_ + cg_id_in_wg_1, node_id_1);
    Port port2 = get_port((i + 1) * cgroup_per_wgroup_ + cg_id_in_wg_2, node_id_2);
    Port::connect(port1, port2);
    global_link_map_.insert({std::make_pair(i, i + 1), port1});
    global_link_map_.insert({std::make_pair(i + 1, i), port2});
  }
  // Step 2: connect non-adjacent W-group pairs (i, j)
  for (int i = 0; i < num_wgroup_ - 2; i++) {
    for (int j = i + 2; j < num_wgroup_; j++) {
      int cg_id_in_wg_1, node_id_1, cg_id_in_wg_2, node_id_2;
      std::tie(cg_id_in_wg_1, node_id_1) = global_port_id_to_port_id(g_ports_per_wg_ - (j - i));
      std::tie(cg_id_in_wg_2, node_id_2) = global_port_id_to_port_id(i + 1);
      Port port1 = get_port(i * cgroup_per_wgroup_ + cg_id_in_wg_1, node_id_1);
      Port port2 = get_port(j * cgroup_per_wgroup_ + cg_id_in_wg_2, node_id_2);
      Port::connect(port1, port2);
      global_link_map_.insert({std::make_pair(i, j), port1});
      global_link_map_.insert({std::make_pair(j, i), port2});
    }
  }
}

// ---- Routing ----

void DragonflyChipletFC::routing_algorithm(Packet& s) const {
  if (algorithm_ == "MIN")
    MIN_routing(s);
  else
    std::cerr << "Unknown routing algorithm: " << algorithm_ << std::endl;
}

// Minimal routing for the FC variant.
//
// The key simplification over DragonflyChiplet::MIN_routing is that
// within a C-group, any chiplet can reach any other in ONE hop directly,
// so there is no need for XY_routing to traverse intermediate nodes.
//
// VC assignment follows the same up*/down* scheme as the paper:
//   VC 0 : misrouting (non-minimal global hop from source W-group)
//   VC 1 : minimal global hop
//   VC 2 : destination W-group (local + intra-CG)
//
// Because intra-CG paths are single-hop, deadlock within the C-group
// is impossible by construction — no cyclic dependency can form on a
// direct link. This means the VC requirements may actually be reducible
// compared with the mesh-based variant; VC 2 is kept for consistency.
void DragonflyChipletFC::MIN_routing(Packet& s) const {
  NodeInCGFC* current   = get_node(s.head_trace().id);
  NodeInCGFC* destination = get_node(s.destination_);

  CGroupFC* current_cg  = current->cgroup_;
  CGroupFC* dest_cg     = destination->cgroup_;

  int current_cg_id_in_wg = current_cg->cgroup_id_ % cgroup_per_wgroup_;
  int dest_cg_id_in_wg    = dest_cg->cgroup_id_ % cgroup_per_wgroup_;

  // ---- Case 1: same C-group ----
  // Single internal hop directly to destination chiplet.
  if (current_cg->cgroup_id_ == dest_cg->cgroup_id_) {
    if (current->node_id_in_cg_ == destination->node_id_in_cg_) {
      // Already there — should not occur in normal operation
      return;
    }
    int port = current->internal_port_to(destination->node_id_in_cg_);
    s.candidate_channels_.push_back(VCInfo(current->link_buffers_[port], 2));
    return;
  }

  // ---- Case 2: same W-group, different C-group ----
  // One internal hop to the chiplet that holds the local link,
  // then one external hop to the destination C-group.
  if (current_cg->wgroup_id_ == dest_cg->wgroup_id_) {
    int target_node_id = local_link_map_.at({current_cg_id_in_wg, dest_cg_id_in_wg});
    Port local_port = get_port(current_cg->cgroup_id_, target_node_id);

    if (target_node_id == current->node_id_in_cg_) {
      // Already at the chiplet with the local link — use it directly
      s.candidate_channels_.push_back(VCInfo(local_port.link_buffer, 2));
    } else {
      // One internal FC hop to the chiplet that owns the local port
      int port = current->internal_port_to(target_node_id);
      s.candidate_channels_.push_back(VCInfo(current->link_buffers_[port], 2));
    }
    return;
  }

  // ---- Case 3: different W-group (global hop needed) ----
  int current_wg_id = current_cg->wgroup_id_;
  int dest_wg_id    = dest_cg->wgroup_id_;

  // Optional misrouting (non-minimal): spread load across global links
  if (mis_routing_) {
    CGroupFC* source_cg = get_node(s.source_)->cgroup_;
    int source_wg_id = source_cg->wgroup_id_;
    int src_cg_id_in_wg = source_cg->cgroup_id_ % cgroup_per_wgroup_;
    if (current_wg_id == source_wg_id) {
      // Pick a global port based on source node to spread traffic
      int misroute_node_id =
          port_node_map_.at(src_cg_id_in_wg + s.source_.node_id % g_ports_per_cg_);
      Port mis_port = get_port(current_cg->cgroup_id_, misroute_node_id);
      if (current->node_id_in_cg_ == misroute_node_id) {
        s.candidate_channels_.push_back(VCInfo(mis_port.link_buffer, 0));
      } else {
        int port = current->internal_port_to(misroute_node_id);
        s.candidate_channels_.push_back(VCInfo(current->link_buffers_[port], 0));
      }
      return;
    }
  }

  // Find the global port connecting current_wg to dest_wg
  Port global_port = global_link_map_.at({current_wg_id, dest_wg_id});
  int global_node_id = global_port.node_id.node_id;
  CGroupFC* global_cg = get_cgroup(global_port.node_id);

  if (current_cg->cgroup_id_ == global_cg->cgroup_id_) {
    // The global port is in the current C-group
    if (global_node_id == current->node_id_in_cg_) {
      // Already at the right chiplet — send via global link
      s.candidate_channels_.push_back(VCInfo(global_port.link_buffer, 1));
    } else {
      // One internal hop to the chiplet that owns the global port
      int port = current->internal_port_to(global_node_id);
      s.candidate_channels_.push_back(VCInfo(current->link_buffers_[port], 1));
    }
  } else {
    // Global port is in a different C-group within the same W-group.
    // First take a local link to that C-group, then the global link.
    int global_cg_id_in_wg = global_cg->cgroup_id_ % cgroup_per_wgroup_;
    int local_node_id = local_link_map_.at({current_cg_id_in_wg, global_cg_id_in_wg});
    Port local_port = get_port(current_cg->cgroup_id_, local_node_id);

    if (local_node_id == current->node_id_in_cg_) {
      s.candidate_channels_.push_back(VCInfo(local_port.link_buffer, 1));
    } else {
      int port = current->internal_port_to(local_node_id);
      s.candidate_channels_.push_back(VCInfo(current->link_buffers_[port], 1));
    }
  }
}

std::pair<int, int> DragonflyChipletFC::global_port_id_to_port_id(int global_port_id) {
  int cg_id_in_wg = global_port_id / g_ports_per_cg_;
  int node_id = port_node_map_.at(cg_id_in_wg + global_port_id % g_ports_per_cg_);
  return {cg_id_in_wg, node_id};
}