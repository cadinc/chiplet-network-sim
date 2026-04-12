#pragma once
#include "system.h"

// Forward declarations
class CGroupFC;
class DragonflyChipletFC;

// ============================================================
// NodeInCGFC
// ============================================================
// A chiplet node inside a fully-connected C-group.
//
// Port layout per node (total radix = (k*k - 1) + 1):
//   Ports [0 .. k*k-2]  : internal FC links to every other chiplet in C-group
//   Port  [k*k-1]       : single external link (local or global)
//
// Compared with NodeInCG (which had 4 mesh ports + 1 external),
// here internal ports scale as O(N) in the C-group size, which is
// the key trade-off explored by this variant.
// ============================================================
class NodeInCGFC : public Node {
 public:
  // internal_port_count = k_chiplet * k_chiplet - 1  (one slot per peer)
  NodeInCGFC(int k_chiplet, int vc_num, int buffer_size, Channel internal_channel,
             Channel external_channel);

  void set_node(Chip* cgroup, NodeID id) override;

  CGroupFC* cgroup_;
  int node_id_in_cg_;        // convenience alias for id_.node_id
  int k_chiplet_;            // side length of the k x k chiplet grid
  int internal_port_count_;  // k*k - 1

  // External port is always the last buffer / link slot
  Buffer*& ext_in_buffer_;
  NodeID& ext_link_node_;
  Buffer*& ext_link_buffer_;

  // Returns the internal port index that this node uses to reach peer_node_id.
  int internal_port_to(int peer_node_id) const;
};

// ============================================================
// CGroupFC
// ============================================================
class CGroupFC : public Chip {
 public:
  CGroupFC(int k_chiplet, int cgroup_radix, int vc_num, int buffer_size, Channel internal_channel,
           Channel external_channel);
  ~CGroupFC();

  void set_chip(System* dragonfly, int cgroup_id) override;

  inline NodeInCGFC* get_node(int chiplet_id) const {
    return static_cast<NodeInCGFC*>(Chip::get_node(NodeID(chiplet_id)));
  }
  inline NodeInCGFC* get_node(NodeID id) const {
    return static_cast<NodeInCGFC*>(Chip::get_node(id));
  }

  DragonflyChipletFC* dragonfly_;
  int& num_chiplets_;  // alias for number_nodes_
  int k_node_;         // side length of chiplet grid
  int cgroup_radix_;   // number of external ports
  int& cgroup_id_;     // alias for chip_id_
  int wgroup_id_;
};

// ============================================================
// DragonflyChipletFC
// ============================================================
// Switch-less Dragonfly where each C-group is fully-connected
// internally instead of using a 2D-mesh.
//
// single_cgroup mode: simulate only one C-group with no external
// connections. Used to benchmark intra-C-group FC performance,
// analogous to using MultiChipMesh for 2D-mesh C-group tests.
// ============================================================
class DragonflyChipletFC : public System {
 public:
  DragonflyChipletFC();
  ~DragonflyChipletFC();

  void read_config() override;

  void connect_local();
  void connect_global();

  void routing_algorithm(Packet& s) const override;
  void MIN_routing(Packet& s) const;

  inline NodeInCGFC* get_node(NodeID id) const {
    return static_cast<NodeInCGFC*>(System::get_node(id));
  }
  inline CGroupFC* get_cgroup(int cgroup_id) const {
    return static_cast<CGroupFC*>(chips_[cgroup_id]);
  }
  inline CGroupFC* get_cgroup(NodeID id) const {
    return static_cast<CGroupFC*>(get_chip(id.chip_id));
  }

  // Returns the Port (external port) of the given chiplet node in the given C-group.
  inline Port get_port(int cgroup_id, int node_id) const {
    NodeInCGFC* chiplet = get_node(NodeID(node_id, cgroup_id));
    return chiplet->ports_[chiplet->internal_port_count_];
  }

  std::pair<int, int> global_port_id_to_port_id(int global_port_id);

  // ---- topology parameters ----
  std::string algorithm_;
  int k_node_in_CG_;
  bool single_cgroup_;  // NEW: if true, build only one C-group for intra-CG benchmarking

  Channel internal_channel_;
  Channel external_channel_;

  // Derived counts
  int cgroup_radix_;
  int num_nodes_per_cg_;
  int num_chiplets_per_cg_;

  // Dragonfly parameters
  int l_ports_per_cg_;
  int g_ports_per_cg_;
  int g_ports_per_wg_;
  int cgroup_per_wgroup_;
  int num_wgroup_;
  int& num_cgroup_;

  bool mis_routing_;

  // ---- routing lookup tables ----
  std::map<int, int> port_node_map_;
  std::map<std::pair<int, int>, int> local_link_map_;
  std::map<std::pair<int, int>, Port> global_link_map_;

  std::vector<Chip*>& cgroups_;
};
