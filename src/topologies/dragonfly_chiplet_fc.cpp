#include "dragonfly_chiplet_fc.h"

// ============================================================
// NodeInCGFC
// ============================================================

NodeInCGFC::NodeInCGFC(int k_chiplet, int vc_num, int buffer_size, Channel internal_channel,
                       Channel external_channel)
    : Node(k_chiplet * k_chiplet, vc_num, buffer_size),
      ext_in_buffer_(in_buffers_[k_chiplet * k_chiplet - 1]),
      ext_link_node_(link_nodes_[k_chiplet * k_chiplet - 1]),
      ext_link_buffer_(link_buffers_[k_chiplet * k_chiplet - 1]) {
  cgroup_ = nullptr;
  k_chiplet_ = k_chiplet;
  internal_port_count_ = k_chiplet * k_chiplet - 1;
  node_id_in_cg_ = -1;

  for (int i = 0; i < internal_port_count_; i++) {
    in_buffers_[i]->channel_ = internal_channel;
  }
  ext_in_buffer_->channel_ = external_channel;
}

void NodeInCGFC::set_node(Chip* cgroup, NodeID id) {
  assert(cgroup != nullptr);
  Node::set_node(cgroup, id);
  cgroup_ = dynamic_cast<CGroupFC*>(chip_);
  node_id_in_cg_ = id.node_id;
}

int NodeInCGFC::internal_port_to(int peer_node_id) const {
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
    nodes_.push_back(
        new NodeInCGFC(k_chiplet, vc_num, buffer_size, internal_channel, external_channel));
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
  for (int i = 0; i < num_chiplets_; i++) {
    NodeInCGFC* node_i = get_node(i);
    for (int j = i + 1; j < num_chiplets_; j++) {
      NodeInCGFC* node_j = get_node(j);
      int port_i = node_i->internal_port_to(j);
      int port_j = node_j->internal_port_to(i);
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
  cgroup_radix_ = num_chiplets_per_cg_;  // one external port per chiplet

  // ---- Single C-group mode ----
  // Simulates only the internal FC network of a single C-group.
  // Analogous to using MultiChipMesh for 2D-mesh intra-C-group benchmarks.
  // All inter-C-group and inter-W-group connections are omitted.
  // traffic_scale in the ini file should equal num_chiplets_per_cg_.
  if (single_cgroup_) {
    l_ports_per_cg_ = 0;
    g_ports_per_cg_ = 0;
    cgroup_per_wgroup_ = 1;
    g_ports_per_wg_ = 0;
    num_wgroup_ = 1;
    num_cgroup_ = 1;
    num_cores_ = num_chiplets_per_cg_;
    num_nodes_ = num_cores_;
    std::cout << "[DragonflyChipletFC] Single C-group mode"
              << "  k=" << k_node_in_CG_ << "  chiplets=" << num_chiplets_per_cg_
              << "  internal_ports_per_chiplet=" << (num_chiplets_per_cg_ - 1)
              << "  num_cores=" << num_cores_ << std::endl;
    // port_node_map_ is populated for API consistency even though external
    // ports are unused in single_cgroup mode.
    for (int i = 0; i < cgroup_radix_; i++) port_node_map_[i] = i;
    cgroups_.push_back(new CGroupFC(k_node_in_CG_, cgroup_radix_, param->vc_number,
                                    param->buffer_size, internal_channel_, external_channel_));
    cgroups_[0]->set_chip(this, 0);
    return;
  }

  // ---- Full Dragonfly hierarchy ----
  l_ports_per_cg_ = cgroup_radix_ / 3 * 2 - 1;
  g_ports_per_cg_ = cgroup_radix_ - l_ports_per_cg_;
  cgroup_per_wgroup_ = l_ports_per_cg_ + 1;
  g_ports_per_wg_ = g_ports_per_cg_ * cgroup_per_wgroup_;
  num_wgroup_ = g_ports_per_wg_ + 1;
  num_cgroup_ = num_wgroup_ * cgroup_per_wgroup_;
  num_cores_ = num_cgroup_ * num_nodes_per_cg_;
  num_nodes_ = num_cores_;

  std::cout << "[DragonflyChipletFC]"
            << "  k=" << k_node_in_CG_ << "  chiplets_per_cg=" << num_chiplets_per_cg_
            << "  internal_ports_per_chiplet=" << (num_chiplets_per_cg_ - 1)
            << "  external_ports_per_cg=" << cgroup_radix_ << "  l=" << l_ports_per_cg_
            << "  g=" << g_ports_per_cg_ << "  cg_per_wg=" << cgroup_per_wgroup_
            << "  num_wg=" << num_wgroup_ << "  num_cg=" << num_cgroup_
            << "  num_cores=" << num_cores_ << std::endl;

  for (int i = 0; i < cgroup_radix_; i++) {
    port_node_map_[i] = i;
  }

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
  int internal_latency = param->params_ptree.get<int>("Network.internal_latency", 1);
  int external_latency = param->params_ptree.get<int>("Network.external_latency", 4);
  internal_channel_ = Channel(internal_bandwidth, internal_latency);
  external_channel_ = Channel(1, external_latency);
  mis_routing_ = param->params_ptree.get<bool>("Network.mis_routing", false);
  single_cgroup_ = param->params_ptree.get<bool>("Network.single_cgroup", false);
}

// ---- Local connectivity ----
void DragonflyChipletFC::connect_local() {
  for (int wg_id = 0; wg_id < num_wgroup_; wg_id++) {
    for (int i = 0; i < cgroup_per_wgroup_ - 1; i++) {
      int node_id_1 = port_node_map_.at(cgroup_radix_ - 1);
      int node_id_2 = port_node_map_.at(0);
      Port port1 = get_port(wg_id * cgroup_per_wgroup_ + i, node_id_1);
      Port port2 = get_port(wg_id * cgroup_per_wgroup_ + i + 1, node_id_2);
      Port::connect(port1, port2);
      if (wg_id == 0) {
        local_link_map_[{i, i + 1}] = node_id_1;
        local_link_map_[{i + 1, i}] = node_id_2;
      }
    }
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
void DragonflyChipletFC::connect_global() {
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

void DragonflyChipletFC::MIN_routing(Packet& s) const {
  NodeInCGFC* current = get_node(s.head_trace().id);
  NodeInCGFC* destination = get_node(s.destination_);

  CGroupFC* current_cg = current->cgroup_;
  CGroupFC* dest_cg = destination->cgroup_;

  int current_cg_id_in_wg = current_cg->cgroup_id_ % cgroup_per_wgroup_;
  int dest_cg_id_in_wg = dest_cg->cgroup_id_ % cgroup_per_wgroup_;

  // ---- Case 1: same C-group ----
  // This is the ONLY case triggered in single_cgroup mode.
  // VC 2 is used; ini files must specify vc_number >= 3.
  if (current_cg->cgroup_id_ == dest_cg->cgroup_id_) {
    if (current->node_id_in_cg_ == destination->node_id_in_cg_) return;
    int port = current->internal_port_to(destination->node_id_in_cg_);
    s.candidate_channels_.push_back(VCInfo(current->link_buffers_[port], 2));
    return;
  }

  // ---- Case 2: same W-group, different C-group ----
  if (current_cg->wgroup_id_ == dest_cg->wgroup_id_) {
    int target_node_id = local_link_map_.at({current_cg_id_in_wg, dest_cg_id_in_wg});
    Port local_port = get_port(current_cg->cgroup_id_, target_node_id);
    if (target_node_id == current->node_id_in_cg_) {
      s.candidate_channels_.push_back(VCInfo(local_port.link_buffer, 2));
    } else {
      int port = current->internal_port_to(target_node_id);
      s.candidate_channels_.push_back(VCInfo(current->link_buffers_[port], 2));
    }
    return;
  }

  // ---- Case 3: different W-group (global hop needed) ----
  int current_wg_id = current_cg->wgroup_id_;
  int dest_wg_id = dest_cg->wgroup_id_;

  if (mis_routing_) {
    CGroupFC* source_cg = get_node(s.source_)->cgroup_;
    int source_wg_id = source_cg->wgroup_id_;
    int src_cg_id_in_wg = source_cg->cgroup_id_ % cgroup_per_wgroup_;
    if (current_wg_id == source_wg_id) {
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

  Port global_port = global_link_map_.at({current_wg_id, dest_wg_id});
  int global_node_id = global_port.node_id.node_id;
  CGroupFC* global_cg = get_cgroup(global_port.node_id);

  if (current_cg->cgroup_id_ == global_cg->cgroup_id_) {
    if (global_node_id == current->node_id_in_cg_) {
      s.candidate_channels_.push_back(VCInfo(global_port.link_buffer, 1));
    } else {
      int port = current->internal_port_to(global_node_id);
      s.candidate_channels_.push_back(VCInfo(current->link_buffers_[port], 1));
    }
  } else {
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
