#pragma once
#include "system.h"

// Forward declarations
class CGroup3D;
class DragonflyChiplet3D;

// ============================================================
// NodeInCG3D
// ============================================================
// A chiplet node inside a 3D-mesh C-group.
//
// Port layout per node (total radix = 6 internal + 1 external):
//   Port 0: x-negative neighbor
//   Port 1: x-positive neighbor
//   Port 2: y-negative neighbor
//   Port 3: y-positive neighbor
//   Port 4: z-negative neighbor
//   Port 5: z-positive neighbor
//   Port 6: external link (local or global)
//
// Corner/edge/face nodes have fewer active internal links but
// same radix -- unused ports simply have no connected neighbor.
//
// Connection count comparison for k^3 chiplets:
//   2D mesh k*k:    2k(k-1)        links,  4 ports max
//   3D mesh k*k*k:  3k^2(k-1)      links,  6 ports max
//   FC k*k:         k^2(k^2-1)/2   links,  k^2-1 ports
//
// For k=4:
//   2D mesh 4x4:    24   links
//   3D mesh 4x4x4:  144  links,  6 ports -- sweet spot
//   FC 4x4:         120  links,  15 ports
// ============================================================
class NodeInCG3D : public Node {
 public:
  NodeInCG3D(int k_chiplet, int vc_num, int buffer_size, Channel internal_channel,
             Channel external_channel);

  void set_node(Chip* cgroup, NodeID id) override;

  CGroup3D* cgroup_;
  int node_id_in_cg_;
  int k_chiplet_;  // side length -- C-group is k x k x k

  // 3D coordinates within the C-group
  int x_, y_, z_;

  // Named port references for the 6 mesh directions + 1 external
  Buffer*& xneg_in_buffer_;
  Buffer*& xpos_in_buffer_;
  Buffer*& yneg_in_buffer_;
  Buffer*& ypos_in_buffer_;
  Buffer*& zneg_in_buffer_;
  Buffer*& zpos_in_buffer_;
  Buffer*& ext_in_buffer_;

  NodeID& xneg_link_node_;
  NodeID& xpos_link_node_;
  NodeID& yneg_link_node_;
  NodeID& ypos_link_node_;
  NodeID& zneg_link_node_;
  NodeID& zpos_link_node_;
  NodeID& ext_link_node_;

  Buffer*& xneg_link_buffer_;
  Buffer*& xpos_link_buffer_;
  Buffer*& yneg_link_buffer_;
  Buffer*& ypos_link_buffer_;
  Buffer*& zneg_link_buffer_;
  Buffer*& zpos_link_buffer_;
  Buffer*& ext_link_buffer_;

  // Convert (x,y,z) to flat node_id
  inline int xyz_to_id(int x, int y, int z) const {
    return x + y * k_chiplet_ + z * k_chiplet_ * k_chiplet_;
  }
};

// ============================================================
// CGroup3D
// ============================================================
// A C-group whose chiplets are arranged in a k x k x k 3D mesh.
//
// Key properties vs 2D mesh:
//   - 3x more internal links per chiplet (6 vs 4 max, but more
//     chiplets per C-group for same k)
//   - Diameter = 3(k-1) vs 2(k-1) for 2D mesh
//   - Bisection bandwidth = k^3 / 2 * 2 = k^3 links vs k^2/2*2
//     for 2D mesh -- much better scaling
//   - Each chiplet still has exactly 1 external port
//   - External ports = k^3 (one per chiplet) -- more than 2D mesh
//     (4k-4) for any k >= 3
//
// Physical motivation: wafer stacking / 3D integration is a real
// and growing technology (HBM, Cerebras, etc). A 3D mesh is a
// natural extension of the 2D mesh the paper already uses.
// ============================================================
class CGroup3D : public Chip {
 public:
  CGroup3D(int k_chiplet, int cgroup_radix, int vc_num, int buffer_size, Channel internal_channel,
           Channel external_channel);
  ~CGroup3D();

  void set_chip(System* dragonfly, int cgroup_id) override;

  inline NodeInCG3D* get_node(int chiplet_id) const {
    return static_cast<NodeInCG3D*>(Chip::get_node(NodeID(chiplet_id)));
  }
  inline NodeInCG3D* get_node(NodeID id) const {
    return static_cast<NodeInCG3D*>(Chip::get_node(id));
  }

  DragonflyChiplet3D* dragonfly_;
  int& num_chiplets_;
  int k_node_;
  int cgroup_radix_;
  int& cgroup_id_;
  int wgroup_id_;
};

// ============================================================
// DragonflyChiplet3D
// ============================================================
// Switch-less Dragonfly where each C-group is a 3D mesh
// instead of a 2D mesh.
//
// Compared with DragonflyChiplet (2D mesh):
//   - More chiplets per C-group for same k (k^3 vs k^2)
//   - Better bisection bandwidth scaling
//   - Shorter diameter relative to C-group size
//   - Each chiplet uses 6 internal ports vs 4 max in 2D
//   - Routing uses XYZ dimension-order instead of XY
//
// Compared with DragonflyChipletFC:
//   - Far fewer internal links (144 vs 120 for k=4, but 64 vs 16
//     chiplets -- much better scaling)
//   - Intra-C-group paths may need up to 3(k-1) hops vs 1 for FC
//   - Memory feasible at much larger k values
// ============================================================
class DragonflyChiplet3D : public System {
 public:
  DragonflyChiplet3D();
  ~DragonflyChiplet3D();

  void read_config() override;

  void connect_local();
  void connect_global();

  void routing_algorithm(Packet& s) const override;
  void MIN_routing(Packet& s) const;
  void XYZ_routing(Packet& s, NodeID dest, int vcb) const;

  inline NodeInCG3D* get_node(NodeID id) const {
    return static_cast<NodeInCG3D*>(System::get_node(id));
  }
  inline CGroup3D* get_cgroup(int cgroup_id) const {
    return static_cast<CGroup3D*>(chips_[cgroup_id]);
  }
  inline CGroup3D* get_cgroup(NodeID id) const {
    return static_cast<CGroup3D*>(get_chip(id.chip_id));
  }

  // External port is always port index 6 (after 6 mesh ports)
  inline Port get_port(int cgroup_id, int node_id) const {
    NodeInCG3D* chiplet = get_node(NodeID(node_id, cgroup_id));
    return chiplet->ports_[6];
  }

  std::pair<int, int> global_port_id_to_port_id(int global_port_id);

  // ---- topology parameters ----
  std::string algorithm_;
  int k_node_in_CG_;  // side length of k x k x k chiplet cube

  Channel internal_channel_;
  Channel external_channel_;

  // Derived counts
  int cgroup_radix_;      // = k^3, one external port per chiplet
  int num_nodes_per_cg_;  // k^3
  int num_chiplets_per_cg_;

  // Dragonfly hierarchy parameters
  int l_ports_per_cg_;
  int g_ports_per_cg_;
  int g_ports_per_wg_;
  int cgroup_per_wgroup_;
  int num_wgroup_;
  int& num_cgroup_;

  bool mis_routing_;
  int max_wgroups_;
  bool single_cgroup_;

  // Routing lookup tables
  std::map<int, int> port_node_map_;
  std::map<std::pair<int, int>, int> local_link_map_;
  std::map<std::pair<int, int>, Port> global_link_map_;

  std::vector<Chip*>& cgroups_;
};
