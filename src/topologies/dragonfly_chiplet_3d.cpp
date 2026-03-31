#include "dragonfly_chiplet_3d.h"

// ============================================================
// NodeInCG3D
// ============================================================

NodeInCG3D::NodeInCG3D(int k_chiplet, int vc_num, int buffer_size, Channel internal_channel,
                       Channel external_channel)
    // radix = 6 internal (3D mesh directions) + 1 external
    : Node(7, vc_num, buffer_size),
      xneg_in_buffer_(in_buffers_[0]),
      xpos_in_buffer_(in_buffers_[1]),
      yneg_in_buffer_(in_buffers_[2]),
      ypos_in_buffer_(in_buffers_[3]),
      zneg_in_buffer_(in_buffers_[4]),
      zpos_in_buffer_(in_buffers_[5]),
      ext_in_buffer_(in_buffers_[6]),
      xneg_link_node_(link_nodes_[0]),
      xpos_link_node_(link_nodes_[1]),
      yneg_link_node_(link_nodes_[2]),
      ypos_link_node_(link_nodes_[3]),
      zneg_link_node_(link_nodes_[4]),
      zpos_link_node_(link_nodes_[5]),
      ext_link_node_(link_nodes_[6]),
      xneg_link_buffer_(link_buffers_[0]),
      xpos_link_buffer_(link_buffers_[1]),
      yneg_link_buffer_(link_buffers_[2]),
      ypos_link_buffer_(link_buffers_[3]),
      zneg_link_buffer_(link_buffers_[4]),
      zpos_link_buffer_(link_buffers_[5]),
      ext_link_buffer_(link_buffers_[6]) {
  cgroup_ = nullptr;
  k_chiplet_ = k_chiplet;
  x_ = 0;
  y_ = 0;
  z_ = 0;
  node_id_in_cg_ = -1;

  // Internal ports: low-latency on-wafer channels
  for (int i = 0; i < 6; i++) {
    in_buffers_[i]->channel_ = internal_channel;
  }
  // External port: higher-latency off-wafer channel
  ext_in_buffer_->channel_ = external_channel;
}

void NodeInCG3D::set_node(Chip* cgroup, NodeID id) {
  assert(cgroup != nullptr);
  Node::set_node(cgroup, id);
  cgroup_ = dynamic_cast<CGroup3D*>(chip_);
  node_id_in_cg_ = id.node_id;
  // Compute 3D coordinates from flat node_id
  // Layout: node_id = x + y*k + z*k*k
  x_ = id.node_id % k_chiplet_;
  y_ = (id.node_id / k_chiplet_) % k_chiplet_;
  z_ = id.node_id / (k_chiplet_ * k_chiplet_);
}

// ============================================================
// CGroup3D
// ============================================================

CGroup3D::CGroup3D(int k_chiplet, int cgroup_radix, int vc_num, int buffer_size,
                   Channel internal_channel, Channel external_channel)
    : num_chiplets_(number_nodes_), cgroup_id_(chip_id_) {
  k_node_ = k_chiplet;
  cgroup_radix_ = cgroup_radix;
  num_chiplets_ = k_chiplet * k_chiplet * k_chiplet;
  number_cores_ = num_chiplets_;
  wgroup_id_ = 0;
  dragonfly_ = nullptr;
  nodes_.reserve(num_chiplets_);
  for (int i = 0; i < num_chiplets_; i++) {
    nodes_.push_back(
        new NodeInCG3D(k_chiplet, vc_num, buffer_size, internal_channel, external_channel));
  }
}

CGroup3D::~CGroup3D() {
  for (auto n : nodes_) delete n;
  nodes_.clear();
}

void CGroup3D::set_chip(System* dragonfly, int cgroup_id) {
  Chip::set_chip(dragonfly, cgroup_id);
  dragonfly_ = dynamic_cast<DragonflyChiplet3D*>(system_);
  wgroup_id_ = cgroup_id / dragonfly_->cgroup_per_wgroup_;

  // Wire up the 3D mesh.
  // Each node connects to up to 6 neighbors (xneg, xpos, yneg, ypos, zneg, zpos).
  // Boundary nodes simply have no link in the missing direction.
  //
  // Connection count: 3 * k^2 * (k-1) total links for k x k x k mesh.
  // For k=4: 3 * 16 * 3 = 144 links vs 24 for 2D mesh or 120 for FC 4x4.
  for (int node_id = 0; node_id < num_chiplets_; node_id++) {
    NodeInCG3D* node = get_node(node_id);
    int x = node->x_;
    int y = node->y_;
    int z = node->z_;
    int k = k_node_;

    // x-direction
    if (x > 0) {
      int nb_id = node->xyz_to_id(x - 1, y, z);
      node->xneg_link_node_ = NodeID(nb_id, cgroup_id);
      node->xneg_link_buffer_ = get_node(nb_id)->xpos_in_buffer_;
    }
    if (x < k - 1) {
      int nb_id = node->xyz_to_id(x + 1, y, z);
      node->xpos_link_node_ = NodeID(nb_id, cgroup_id);
      node->xpos_link_buffer_ = get_node(nb_id)->xneg_in_buffer_;
    }
    // y-direction
    if (y > 0) {
      int nb_id = node->xyz_to_id(x, y - 1, z);
      node->yneg_link_node_ = NodeID(nb_id, cgroup_id);
      node->yneg_link_buffer_ = get_node(nb_id)->ypos_in_buffer_;
    }
    if (y < k - 1) {
      int nb_id = node->xyz_to_id(x, y + 1, z);
      node->ypos_link_node_ = NodeID(nb_id, cgroup_id);
      node->ypos_link_buffer_ = get_node(nb_id)->yneg_in_buffer_;
    }
    // z-direction
    if (z > 0) {
      int nb_id = node->xyz_to_id(x, y, z - 1);
      node->zneg_link_node_ = NodeID(nb_id, cgroup_id);
      node->zneg_link_buffer_ = get_node(nb_id)->zpos_in_buffer_;
    }
    if (z < k - 1) {
      int nb_id = node->xyz_to_id(x, y, z + 1);
      node->zpos_link_node_ = NodeID(nb_id, cgroup_id);
      node->zpos_link_buffer_ = get_node(nb_id)->zneg_in_buffer_;
    }
  }
}

// ============================================================
// DragonflyChiplet3D
// ============================================================

DragonflyChiplet3D::DragonflyChiplet3D() : num_cgroup_(num_chips_), cgroups_(chips_) {
  read_config();

  num_chiplets_per_cg_ = k_node_in_CG_ * k_node_in_CG_ * k_node_in_CG_;
  num_nodes_per_cg_ = num_chiplets_per_cg_;

  // One external port per chiplet (same principle as FC variant)
  // but now k^3 chiplets per C-group instead of k^2.
  // External port count = k^3, which is much larger than 2D mesh (4k-4)
  // giving a bigger and better-connected Dragonfly network.
  cgroup_radix_ = num_chiplets_per_cg_;

  if (single_cgroup_) {
    // Single C-group mode for studying intra-C-group performance
    l_ports_per_cg_ = 0;
    g_ports_per_cg_ = 0;
    cgroup_per_wgroup_ = 1;
    g_ports_per_wg_ = 0;
    num_wgroup_ = 1;
    num_cgroup_ = 1;
    num_cores_ = num_chiplets_per_cg_;
    num_nodes_ = num_cores_;
    std::cout << "[DragonflyChiplet3D] Single C-group mode"
              << "  k=" << k_node_in_CG_ << "  chiplets=" << num_chiplets_per_cg_
              << "  internal_ports_per_chiplet=6"
              << "  total_links=" << (3 * k_node_in_CG_ * k_node_in_CG_ * (k_node_in_CG_ - 1))
              << "  num_cores=" << num_cores_ << std::endl;
    cgroups_.push_back(new CGroup3D(k_node_in_CG_, cgroup_radix_, param->vc_number,
                                    param->buffer_size, internal_channel_, external_channel_));
    cgroups_[0]->set_chip(this, 0);
    return;
  }

  l_ports_per_cg_ = cgroup_radix_ / 3 * 2 - 1;
  g_ports_per_cg_ = cgroup_radix_ - l_ports_per_cg_;
  cgroup_per_wgroup_ = l_ports_per_cg_ + 1;
  g_ports_per_wg_ = g_ports_per_cg_ * cgroup_per_wgroup_;
  num_wgroup_ = g_ports_per_wg_ + 1;

  if (max_wgroups_ > 0 && num_wgroup_ > max_wgroups_) {
    std::cout << "[DragonflyChiplet3D] Capping num_wgroup from " << num_wgroup_ << " to "
              << max_wgroups_ << std::endl;
    num_wgroup_ = max_wgroups_;
  }

  num_cgroup_ = num_wgroup_ * cgroup_per_wgroup_;
  num_cores_ = num_cgroup_ * num_nodes_per_cg_;
  num_nodes_ = num_cores_;

  std::cout << "[DragonflyChiplet3D]"
            << "  k=" << k_node_in_CG_ << "  chiplets_per_cg=" << num_chiplets_per_cg_
            << "  internal_ports_per_chiplet=6"
            << "  total_internal_links="
            << (3 * k_node_in_CG_ * k_node_in_CG_ * (k_node_in_CG_ - 1))
            << "  external_ports_per_cg=" << cgroup_radix_ << "  l=" << l_ports_per_cg_
            << "  g=" << g_ports_per_cg_ << "  cg_per_wg=" << cgroup_per_wgroup_
            << "  num_wg=" << num_wgroup_ << "  num_cg=" << num_cgroup_
            << "  num_cores=" << num_cores_ << std::endl;

  // port_id -> node_id: same as FC, each chiplet owns one external port
  for (int i = 0; i < cgroup_radix_; i++) {
    port_node_map_[i] = i;
  }

  cgroups_.reserve(num_cgroup_);
  for (int cg_id = 0; cg_id < num_cgroup_; cg_id++) {
    cgroups_.push_back(new CGroup3D(k_node_in_CG_, cgroup_radix_, param->vc_number,
                                    param->buffer_size, internal_channel_, external_channel_));
    cgroups_[cg_id]->set_chip(this, cg_id);
  }

  connect_local();
  connect_global();
}

DragonflyChiplet3D::~DragonflyChiplet3D() {
  for (auto cg : cgroups_) delete cg;
  cgroups_.clear();
}

void DragonflyChiplet3D::read_config() {
  k_node_in_CG_ = param->params_ptree.get<int>("Network.k_node", 3);
  algorithm_ = param->params_ptree.get<std::string>("Network.routing_algorithm", "MIN");
  int internal_bandwidth = param->params_ptree.get<int>("Network.internal_bandwidth", 1);
  int external_latency = param->params_ptree.get<int>("Network.external_latency", 4);
  internal_channel_ = Channel(internal_bandwidth, 1);
  external_channel_ = Channel(1, external_latency);
  mis_routing_ = param->params_ptree.get<bool>("Network.mis_routing", false);
  max_wgroups_ = param->params_ptree.get<int>("Network.max_wgroups", 0);
  single_cgroup_ = param->params_ptree.get<bool>("Network.single_cgroup", false);
}

// ---- Local connectivity ----
// Identical port assignment scheme to DragonflyChiplet and FC variant.
void DragonflyChiplet3D::connect_local() {
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
void DragonflyChiplet3D::connect_global() {
  for (int i = 0; i < num_wgroup_ - 1; i++) {
    int cg1, nd1, cg2, nd2;
    std::tie(cg1, nd1) = global_port_id_to_port_id(g_ports_per_wg_ - 1);
    std::tie(cg2, nd2) = global_port_id_to_port_id(0);
    Port port1 = get_port(i * cgroup_per_wgroup_ + cg1, nd1);
    Port port2 = get_port((i + 1) * cgroup_per_wgroup_ + cg2, nd2);
    Port::connect(port1, port2);
    global_link_map_.insert({std::make_pair(i, i + 1), port1});
    global_link_map_.insert({std::make_pair(i + 1, i), port2});
  }
  for (int i = 0; i < num_wgroup_ - 2; i++) {
    for (int j = i + 2; j < num_wgroup_; j++) {
      int cg1, nd1, cg2, nd2;
      std::tie(cg1, nd1) = global_port_id_to_port_id(g_ports_per_wg_ - (j - i));
      std::tie(cg2, nd2) = global_port_id_to_port_id(i + 1);
      Port port1 = get_port(i * cgroup_per_wgroup_ + cg1, nd1);
      Port port2 = get_port(j * cgroup_per_wgroup_ + cg2, nd2);
      Port::connect(port1, port2);
      global_link_map_.insert({std::make_pair(i, j), port1});
      global_link_map_.insert({std::make_pair(j, i), port2});
    }
  }
}

// ---- Routing ----

void DragonflyChiplet3D::routing_algorithm(Packet& s) const {
  if (algorithm_ == "MIN")
    MIN_routing(s);
  else
    std::cerr << "Unknown routing algorithm: " << algorithm_ << std::endl;
}

// XYZ dimension-order routing within a C-group.
// Routes X first, then Y, then Z -- deadlock free by construction
// on a mesh (no wrap-around). VC assignment passed in from caller.
void DragonflyChiplet3D::XYZ_routing(Packet& s, NodeID dest, int vcb) const {
  NodeInCG3D* cur = get_node(s.head_trace().id);
  NodeInCG3D* dst = get_node(dest);

  int dx = dst->x_ - cur->x_;
  int dy = dst->y_ - cur->y_;
  int dz = dst->z_ - cur->z_;

  // Dimension order: X first, then Y, then Z
  if (dx < 0)
    s.candidate_channels_.push_back(VCInfo(cur->xneg_link_buffer_, vcb));
  else if (dx > 0)
    s.candidate_channels_.push_back(VCInfo(cur->xpos_link_buffer_, vcb));
  else if (dy < 0)
    s.candidate_channels_.push_back(VCInfo(cur->yneg_link_buffer_, vcb));
  else if (dy > 0)
    s.candidate_channels_.push_back(VCInfo(cur->ypos_link_buffer_, vcb));
  else if (dz < 0)
    s.candidate_channels_.push_back(VCInfo(cur->zneg_link_buffer_, vcb));
  else if (dz > 0)
    s.candidate_channels_.push_back(VCInfo(cur->zpos_link_buffer_, vcb));
}

// Minimal routing for the 3D mesh C-group Dragonfly.
// Structure mirrors DragonflyChiplet::MIN_routing but uses
// XYZ_routing instead of XY_routing for intra-C-group traversal.
void DragonflyChiplet3D::MIN_routing(Packet& s) const {
  NodeInCG3D* current = get_node(s.head_trace().id);
  NodeInCG3D* destination = get_node(s.destination_);

  CGroup3D* current_cg = current->cgroup_;
  CGroup3D* dest_cg = destination->cgroup_;

  int current_cg_id_in_wg = current_cg->cgroup_id_ % cgroup_per_wgroup_;
  int dest_cg_id_in_wg = dest_cg->cgroup_id_ % cgroup_per_wgroup_;

  // ---- Case 1: same C-group ----
  if (current_cg->cgroup_id_ == dest_cg->cgroup_id_) {
    XYZ_routing(s, destination->id_, 2);
    return;
  }

  // ---- Case 2: same W-group, different C-group ----
  if (current_cg->wgroup_id_ == dest_cg->wgroup_id_) {
    int target_node_id = local_link_map_.at({current_cg_id_in_wg, dest_cg_id_in_wg});
    Port local_port = get_port(current_cg->cgroup_id_, target_node_id);
    if (target_node_id == current->node_id_in_cg_) {
      s.candidate_channels_.push_back(VCInfo(local_port.link_buffer, 2));
    } else {
      XYZ_routing(s, local_port.node_id, 2);
    }
    return;
  }

  // ---- Case 3: different W-group ----
  int current_wg_id = current_cg->wgroup_id_;
  int dest_wg_id = dest_cg->wgroup_id_;

  // Optional misrouting
  if (mis_routing_) {
    CGroup3D* source_cg = get_node(s.source_)->cgroup_;
    int source_wg_id = source_cg->wgroup_id_;
    int src_cg_id_in_wg = source_cg->cgroup_id_ % cgroup_per_wgroup_;
    if (current_wg_id == source_wg_id) {
      int misroute_node_id =
          port_node_map_.at(src_cg_id_in_wg + s.source_.node_id % g_ports_per_cg_);
      Port mis_port = get_port(current_cg->cgroup_id_, misroute_node_id);
      if (current->node_id_in_cg_ == misroute_node_id) {
        s.candidate_channels_.push_back(VCInfo(mis_port.link_buffer, 0));
      } else {
        XYZ_routing(s, NodeID(misroute_node_id, current_cg->cgroup_id_), 0);
      }
      return;
    }
  }

  // Find global port connecting current_wg to dest_wg
  Port global_port = global_link_map_.at({current_wg_id, dest_wg_id});
  int global_node_id = global_port.node_id.node_id;
  CGroup3D* global_cg = get_cgroup(global_port.node_id);

  if (current_cg->cgroup_id_ == global_cg->cgroup_id_) {
    if (global_node_id == current->node_id_in_cg_) {
      s.candidate_channels_.push_back(VCInfo(global_port.link_buffer, 1));
    } else {
      XYZ_routing(s, global_port.node_id, 1);
    }
  } else {
    int global_cg_id_in_wg = global_cg->cgroup_id_ % cgroup_per_wgroup_;
    int local_node_id = local_link_map_.at({current_cg_id_in_wg, global_cg_id_in_wg});
    Port local_port = get_port(current_cg->cgroup_id_, local_node_id);
    if (local_node_id == current->node_id_in_cg_) {
      s.candidate_channels_.push_back(VCInfo(local_port.link_buffer, 1));
    } else {
      XYZ_routing(s, local_port.node_id, 1);
    }
  }
}

std::pair<int, int> DragonflyChiplet3D::global_port_id_to_port_id(int global_port_id) {
  int cg_id_in_wg = global_port_id / g_ports_per_cg_;
  int node_id = port_node_map_.at(cg_id_in_wg + global_port_id % g_ports_per_cg_);
  return std::make_pair(cg_id_in_wg, node_id);
}
