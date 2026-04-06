#include "dragonfly_chiplet_kn.h"
#include <cmath>
#include <numeric>

// ============================================================
// NodeInCGKN
// ============================================================

NodeInCGKN::NodeInCGKN(int n_dims, int vc_num, int buffer_size,
                       Channel internal_channel, Channel external_channel)
    : Node(2 * n_dims + 1, vc_num, buffer_size),
      ext_in_buffer_(in_buffers_[2 * n_dims]),
      ext_link_node_(link_nodes_[2 * n_dims]),
      ext_link_buffer_(link_buffers_[2 * n_dims]) {
  cgroup_ = nullptr;
  n_dims_ = n_dims;
  k_ = -1;
  is_external_ = false;
  node_id_in_cg_ = -1;
  coords_.resize(n_dims, 0);
  for (int i = 0; i < 2 * n_dims; i++)
    in_buffers_[i]->channel_ = internal_channel;
  ext_in_buffer_->channel_ = external_channel;
}

void NodeInCGKN::set_node(Chip* cgroup, NodeID id) {
  assert(cgroup != nullptr);
  Node::set_node(cgroup, id);
  cgroup_ = dynamic_cast<CGroupKN*>(chip_);
  node_id_in_cg_ = id.node_id;
  k_ = cgroup_->k_;
  // Decode flat node_id = x0 + x1*k + x2*k^2 + ...
  int tmp = id.node_id;
  for (int d = 0; d < n_dims_; d++) {
    coords_[d] = tmp % k_;
    tmp /= k_;
  }
}

bool NodeInCGKN::is_boundary() const {
  for (int d = 0; d < n_dims_; d++)
    if (coords_[d] == 0 || coords_[d] == k_ - 1) return true;
  return false;
}

// ============================================================
// CGroupKN
// ============================================================

CGroupKN::CGroupKN(int k, int n_dims, int cgroup_radix, int vc_num, int buffer_size,
                   Channel internal_channel, Channel external_channel)
    : num_chiplets_(number_nodes_), cgroup_id_(chip_id_) {
  k_ = k;
  n_dims_ = n_dims;
  cgroup_radix_ = cgroup_radix;
  int total = 1;
  for (int d = 0; d < n_dims; d++) total *= k;
  num_chiplets_ = total;
  number_cores_ = num_chiplets_;
  wgroup_id_ = 0;
  dragonfly_ = nullptr;
  nodes_.reserve(num_chiplets_);
  for (int i = 0; i < num_chiplets_; i++)
    nodes_.push_back(new NodeInCGKN(n_dims, vc_num, buffer_size,
                                    internal_channel, external_channel));
}

CGroupKN::~CGroupKN() {
  for (auto n : nodes_) delete n;
  nodes_.clear();
}

int CGroupKN::coords_to_id(const std::vector<int>& coords) const {
  int id = 0, stride = 1;
  for (int d = 0; d < n_dims_; d++) {
    id += coords[d] * stride;
    stride *= k_;
  }
  return id;
}

// Select cgroup_radix_ external nodes evenly spaced across ALL boundary nodes.
//
// Algorithm:
//   1. Collect every boundary node in canonical (node_id) order.
//   2. Stride through them uniformly to pick cgroup_radix_ nodes.
//
// This avoids the face-by-face approach which breaks for n=2 because
// corner nodes are shared between faces, making it impossible to fill
// 4k slots from a perimeter that only has 4k-4 unique nodes.
//
// For n=2: cgroup_radix = 4k-4 (all perimeter nodes selected).
// For n>=3: cgroup_radix = 4k  (subset of boundary nodes selected).
//
// Result is guaranteed: no duplicates, all nodes are boundary nodes,
// spacing is as uniform as possible.
std::vector<int> CGroupKN::select_external_nodes() const {
  // Collect all boundary node ids in ascending order
  std::vector<int> boundary;
  boundary.reserve(num_chiplets_);
  for (int nid = 0; nid < num_chiplets_; nid++) {
    NodeInCGKN* node = get_node(nid);
    if (node->is_boundary()) boundary.push_back(nid);
  }

  int needed = cgroup_radix_;
  int total  = (int)boundary.size();

  if (needed > total) {
    std::cerr << "[CGroupKN] ERROR: need " << needed
              << " external nodes but only " << total
              << " boundary nodes exist." << std::endl;
    exit(1);
  }

  // Uniform stride selection with center offset for even spacing
  std::vector<int> selected;
  selected.reserve(needed);
  double stride = (double)total / needed;
  for (int i = 0; i < needed; i++) {
    int idx = (int)(i * stride + stride / 2.0);
    idx = std::min(idx, total - 1);
    selected.push_back(boundary[idx]);
  }
  return selected;
}

void CGroupKN::set_chip(System* dragonfly, int cgroup_id) {
  Chip::set_chip(dragonfly, cgroup_id);
  DragonflyChipletKN* df = dynamic_cast<DragonflyChipletKN*>(system_);
  wgroup_id_ = cgroup_id / df->cgroup_per_wgroup_;

  // ---- Wire internal k-ary n-cube mesh ----
  // Connect each node to its positive neighbor in each dimension.
  // The negative link is wired from the neighbor's side simultaneously.
  for (int node_id = 0; node_id < num_chiplets_; node_id++) {
    NodeInCGKN* node = get_node(node_id);
    for (int d = 0; d < n_dims_; d++) {
      if (node->coords_[d] < k_ - 1) {
        std::vector<int> nb_coords = node->coords_;
        nb_coords[d]++;
        int nb_id = coords_to_id(nb_coords);
        NodeInCGKN* nb = get_node(nb_id);
        node->link_nodes_[node->port_pos(d)]   = NodeID(nb_id, cgroup_id);
        node->link_buffers_[node->port_pos(d)] = nb->in_buffers_[node->port_neg(d)];
        nb->link_nodes_[nb->port_neg(d)]       = NodeID(node_id, cgroup_id);
        nb->link_buffers_[nb->port_neg(d)]     = node->in_buffers_[node->port_pos(d)];
      }
    }
  }

  // ---- Select and mark external nodes ----
  external_nodes_ = select_external_nodes();
  for (int idx = 0; idx < (int)external_nodes_.size(); idx++) {
    int nid = external_nodes_[idx];
    node_to_ext_port_[nid] = idx;
    ext_port_to_node_[idx] = nid;
    get_node(nid)->is_external_ = true;
  }

  // Only print for first C-group to avoid flooding output
  if (cgroup_id == 0) {
    std::cout << "[CGroupKN] k=" << k_ << " n=" << n_dims_
              << " chiplets=" << num_chiplets_
              << " boundary=" << ([&](){
                   int b=0;
                   for(int i=0;i<num_chiplets_;i++) if(get_node(i)->is_boundary()) b++;
                   return b; })()
              << " ext_nodes=" << external_nodes_.size()
              << std::endl;
  }
}

// ============================================================
// DragonflyChipletKN
// ============================================================

DragonflyChipletKN::DragonflyChipletKN() : num_cgroup_(num_chips_), cgroups_(chips_) {
  read_config();

  num_chiplets_per_cg_ = 1;
  for (int d = 0; d < n_dims_; d++) num_chiplets_per_cg_ *= k_node_in_CG_;
  num_nodes_per_cg_ = num_chiplets_per_cg_;

  // cgroup_radix = 4k-4 for n=2 (perimeter of a square = all boundary nodes)
  //              = 4k   for n>=3 (subset of boundary nodes, boundary >> 4k)
  // This keeps the Dragonfly hierarchy scale identical to the original
  // 2D mesh DragonflyChiplet for n=2, and constant as n increases for n>=3.
  if (n_dims_ == 2) {
    cgroup_radix_ = 4 * k_node_in_CG_ - 4;
    std::cout << "[DragonflyChipletKN] n=2: using cgroup_radix=4k-4="
              << cgroup_radix_ << " (all perimeter nodes)" << std::endl;
  } else {
    cgroup_radix_ = 4 * k_node_in_CG_;
    // Warn if not evenly divisible across faces
    if ((4 * k_node_in_CG_) % (2 * n_dims_) != 0) {
      std::cout << "[DragonflyChipletKN] WARNING: 4k=" << cgroup_radix_
                << " not divisible by 2n=" << (2 * n_dims_)
                << ". External node spacing will be slightly uneven." << std::endl;
    }
  }

  // Validate enough boundary nodes exist
  int interior = 1;
  for (int d = 0; d < n_dims_; d++) interior *= std::max(0, k_node_in_CG_ - 2);
  int boundary = num_chiplets_per_cg_ - interior;
  if (boundary < cgroup_radix_) {
    std::cerr << "[DragonflyChipletKN] ERROR: Only " << boundary
              << " boundary nodes but " << cgroup_radix_
              << " external ports needed. Increase k or decrease n." << std::endl;
    exit(1);
  }

  if (single_cgroup_) {
    l_ports_per_cg_    = 0;
    g_ports_per_cg_    = 0;
    cgroup_per_wgroup_ = 1;
    g_ports_per_wg_    = 0;
    num_wgroup_        = 1;
    num_cgroup_        = 1;
    num_cores_         = num_chiplets_per_cg_;
    num_nodes_         = num_cores_;
    std::cout << "[DragonflyChipletKN] Single C-group mode"
              << "  k=" << k_node_in_CG_
              << "  n=" << n_dims_
              << "  chiplets=" << num_chiplets_per_cg_
              << "  ports_per_node=" << (2 * n_dims_) << "+1"
              << "  ext_ports=" << cgroup_radix_
              << "  num_cores=" << num_cores_ << std::endl;
    cgroups_.push_back(new CGroupKN(k_node_in_CG_, n_dims_, cgroup_radix_,
                                    param->vc_number, param->buffer_size,
                                    internal_channel_, external_channel_));
    cgroups_[0]->set_chip(this, 0);
    CGroupKN* cg = get_cgroup(0);
    for (auto& kv : cg->ext_port_to_node_) port_node_map_[kv.first] = kv.second;
    return;
  }

  // ---- Full Dragonfly hierarchy ----
  l_ports_per_cg_ = cgroup_radix_ / 3 * 2 - 1;
  g_ports_per_cg_ = cgroup_radix_ - l_ports_per_cg_;
  cgroup_per_wgroup_ = l_ports_per_cg_ + 1;
  g_ports_per_wg_ = g_ports_per_cg_ * cgroup_per_wgroup_;
  num_wgroup_ = g_ports_per_wg_ + 1;

  if (max_wgroups_ > 0 && num_wgroup_ > max_wgroups_) {
    std::cout << "[DragonflyChipletKN] Capping num_wgroup from " << num_wgroup_
              << " to " << max_wgroups_ << std::endl;
    num_wgroup_ = max_wgroups_;
  }

  num_cgroup_ = num_wgroup_ * cgroup_per_wgroup_;
  num_cores_  = num_cgroup_ * num_nodes_per_cg_;
  num_nodes_  = num_cores_;

  std::cout << "[DragonflyChipletKN]"
            << "  k=" << k_node_in_CG_
            << "  n=" << n_dims_
            << "  chiplets_per_cg=" << num_chiplets_per_cg_
            << "  ports_per_node=" << (2*n_dims_) << "+1"
            << "  ext_ports_per_cg=" << cgroup_radix_
            << "  l=" << l_ports_per_cg_
            << "  g=" << g_ports_per_cg_
            << "  cg_per_wg=" << cgroup_per_wgroup_
            << "  num_wg=" << num_wgroup_
            << "  num_cg=" << num_cgroup_
            << "  num_cores=" << num_cores_ << std::endl;

  cgroups_.reserve(num_cgroup_);
  for (int cg_id = 0; cg_id < num_cgroup_; cg_id++) {
    cgroups_.push_back(new CGroupKN(k_node_in_CG_, n_dims_, cgroup_radix_,
                                    param->vc_number, param->buffer_size,
                                    internal_channel_, external_channel_));
    cgroups_[cg_id]->set_chip(this, cg_id);
  }

  CGroupKN* cg0 = get_cgroup(0);
  for (auto& kv : cg0->ext_port_to_node_) port_node_map_[kv.first] = kv.second;

  connect_local();
  connect_global();
}

DragonflyChipletKN::~DragonflyChipletKN() {
  for (auto cg : cgroups_) delete cg;
  cgroups_.clear();
}

void DragonflyChipletKN::read_config() {
  k_node_in_CG_  = param->params_ptree.get<int>("Network.k_node", 4);
  n_dims_        = param->params_ptree.get<int>("Network.n_dims", 2);
  algorithm_     = param->params_ptree.get<std::string>("Network.routing_algorithm", "MIN");
  int ib = param->params_ptree.get<int>("Network.internal_bandwidth", 1);
  int el = param->params_ptree.get<int>("Network.external_latency", 4);
  internal_channel_ = Channel(ib, 1);
  external_channel_ = Channel(1, el);
  mis_routing_   = param->params_ptree.get<bool>("Network.mis_routing", false);
  max_wgroups_   = param->params_ptree.get<int>("Network.max_wgroups", 0);
  single_cgroup_ = param->params_ptree.get<bool>("Network.single_cgroup", false);
  if (n_dims_ < 2) { std::cerr << "ERROR: n_dims >= 2 required\n"; exit(1); }
  if (k_node_in_CG_ < 2) { std::cerr << "ERROR: k_node >= 2 required\n"; exit(1); }
}

void DragonflyChipletKN::connect_local() {
  for (int wg_id = 0; wg_id < num_wgroup_; wg_id++) {
    for (int i = 0; i < cgroup_per_wgroup_ - 1; i++) {
      int n1 = port_node_map_.at(cgroup_radix_ - 1);
      int n2 = port_node_map_.at(0);
      Port p1 = get_port(wg_id * cgroup_per_wgroup_ + i,     n1);
      Port p2 = get_port(wg_id * cgroup_per_wgroup_ + i + 1, n2);
      Port::connect(p1, p2);
      if (wg_id == 0) {
        local_link_map_[{i, i+1}] = n1;
        local_link_map_[{i+1, i}] = n2;
      }
    }
    for (int i = 0; i < cgroup_per_wgroup_ - 2; i++) {
      for (int j = i + 2; j < cgroup_per_wgroup_; j++) {
        int n1 = port_node_map_.at(cgroup_radix_ - (j - i));
        int n2 = port_node_map_.at(i + 1);
        Port p1 = get_port(wg_id * cgroup_per_wgroup_ + i, n1);
        Port p2 = get_port(wg_id * cgroup_per_wgroup_ + j, n2);
        Port::connect(p1, p2);
        if (wg_id == 0) {
          local_link_map_[{i, j}] = n1;
          local_link_map_[{j, i}] = n2;
        }
      }
    }
  }
}

void DragonflyChipletKN::connect_global() {
  for (int i = 0; i < num_wgroup_ - 1; i++) {
    int cg1, nd1, cg2, nd2;
    std::tie(cg1, nd1) = global_port_id_to_port_id(g_ports_per_wg_ - 1);
    std::tie(cg2, nd2) = global_port_id_to_port_id(0);
    Port p1 = get_port(i * cgroup_per_wgroup_ + cg1, nd1);
    Port p2 = get_port((i+1) * cgroup_per_wgroup_ + cg2, nd2);
    Port::connect(p1, p2);
    global_link_map_.insert({std::make_pair(i, i+1), p1});
    global_link_map_.insert({std::make_pair(i+1, i), p2});
  }
  for (int i = 0; i < num_wgroup_ - 2; i++) {
    for (int j = i + 2; j < num_wgroup_; j++) {
      int cg1, nd1, cg2, nd2;
      std::tie(cg1, nd1) = global_port_id_to_port_id(g_ports_per_wg_ - (j-i));
      std::tie(cg2, nd2) = global_port_id_to_port_id(i + 1);
      Port p1 = get_port(i * cgroup_per_wgroup_ + cg1, nd1);
      Port p2 = get_port(j * cgroup_per_wgroup_ + cg2, nd2);
      Port::connect(p1, p2);
      global_link_map_.insert({std::make_pair(i, j), p1});
      global_link_map_.insert({std::make_pair(j, i), p2});
    }
  }
}

void DragonflyChipletKN::routing_algorithm(Packet& s) const {
  if (algorithm_ == "MIN") MIN_routing(s);
  else std::cerr << "Unknown routing algorithm: " << algorithm_ << std::endl;
}

// XD: dimension-order routing within a C-group.
// Routes dim 0 first, then 1, ..., n-1. Deadlock free on any mesh.
void DragonflyChipletKN::XD_routing(Packet& s, NodeID dest, int vcb) const {
  NodeInCGKN* cur = get_node(s.head_trace().id);
  NodeInCGKN* dst = get_node(dest);
  for (int d = 0; d < n_dims_; d++) {
    int delta = dst->coords_[d] - cur->coords_[d];
    if (delta < 0) {
      s.candidate_channels_.push_back(VCInfo(cur->link_buffers_[cur->port_neg(d)], vcb));
      return;
    } else if (delta > 0) {
      s.candidate_channels_.push_back(VCInfo(cur->link_buffers_[cur->port_pos(d)], vcb));
      return;
    }
  }
}

void DragonflyChipletKN::MIN_routing(Packet& s) const {
  NodeInCGKN* current     = get_node(s.head_trace().id);
  NodeInCGKN* destination = get_node(s.destination_);
  CGroupKN* current_cg = current->cgroup_;
  CGroupKN* dest_cg    = destination->cgroup_;
  int cur_cg_in_wg  = current_cg->cgroup_id_ % cgroup_per_wgroup_;
  int dest_cg_in_wg = dest_cg->cgroup_id_    % cgroup_per_wgroup_;

  // Case 1: same C-group
  if (current_cg->cgroup_id_ == dest_cg->cgroup_id_) {
    XD_routing(s, destination->id_, 2);
    return;
  }

  // Case 2: same W-group
  if (current_cg->wgroup_id_ == dest_cg->wgroup_id_) {
    int tgt = local_link_map_.at({cur_cg_in_wg, dest_cg_in_wg});
    Port lp = get_port(current_cg->cgroup_id_, tgt);
    if (tgt == current->node_id_in_cg_)
      s.candidate_channels_.push_back(VCInfo(lp.link_buffer, 2));
    else
      XD_routing(s, lp.node_id, 2);
    return;
  }

  // Case 3: different W-group
  int cur_wg  = current_cg->wgroup_id_;
  int dest_wg = dest_cg->wgroup_id_;

  if (mis_routing_) {
    CGroupKN* src_cg = get_node(s.source_)->cgroup_;
    if (cur_wg == src_cg->wgroup_id_) {
      int src_cg_in_wg = src_cg->cgroup_id_ % cgroup_per_wgroup_;
      int mis_nid = port_node_map_.at(src_cg_in_wg + s.source_.node_id % g_ports_per_cg_);
      Port mp = get_port(current_cg->cgroup_id_, mis_nid);
      if (current->node_id_in_cg_ == mis_nid)
        s.candidate_channels_.push_back(VCInfo(mp.link_buffer, 0));
      else
        XD_routing(s, NodeID(mis_nid, current_cg->cgroup_id_), 0);
      return;
    }
  }

  Port gp = global_link_map_.at({cur_wg, dest_wg});
  int  gn = gp.node_id.node_id;
  CGroupKN* gcg = get_cgroup(gp.node_id);

  if (current_cg->cgroup_id_ == gcg->cgroup_id_) {
    if (gn == current->node_id_in_cg_)
      s.candidate_channels_.push_back(VCInfo(gp.link_buffer, 1));
    else
      XD_routing(s, gp.node_id, 1);
  } else {
    int gcg_in_wg = gcg->cgroup_id_ % cgroup_per_wgroup_;
    int ln = local_link_map_.at({cur_cg_in_wg, gcg_in_wg});
    Port lp = get_port(current_cg->cgroup_id_, ln);
    if (ln == current->node_id_in_cg_)
      s.candidate_channels_.push_back(VCInfo(lp.link_buffer, 1));
    else
      XD_routing(s, lp.node_id, 1);
  }
}

std::pair<int, int> DragonflyChipletKN::global_port_id_to_port_id(int global_port_id) {
  int cg_id_in_wg = global_port_id / g_ports_per_cg_;
  int node_id = port_node_map_.at(cg_id_in_wg + global_port_id % g_ports_per_cg_);
  return std::make_pair(cg_id_in_wg, node_id);
}
