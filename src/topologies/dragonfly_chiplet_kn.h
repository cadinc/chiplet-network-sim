#pragma once
#include <cmath>
#include <vector>

#include "system.h"

class CGroupKN;
class DragonflyChipletKN;

class NodeInCGKN : public Node {
 public:
  NodeInCGKN(int n_dims, int vc_num, int buffer_size, Channel internal_channel,
             Channel external_channel);
  void set_node(Chip* cgroup, NodeID id) override;

  CGroupKN* cgroup_;
  int node_id_in_cg_;
  int n_dims_;
  int k_;
  bool is_external_;
  std::vector<int> coords_;

  Buffer*& ext_in_buffer_;
  NodeID& ext_link_node_;
  Buffer*& ext_link_buffer_;

  inline int port_neg(int d) const { return 2 * d; }
  inline int port_pos(int d) const { return 2 * d + 1; }
  inline int ext_port() const { return 2 * n_dims_; }

  bool is_boundary() const;
};

class CGroupKN : public Chip {
 public:
  CGroupKN(int k, int n_dims, int cgroup_radix, int vc_num, int buffer_size,
           Channel internal_channel, Channel external_channel);
  ~CGroupKN();

  void set_chip(System* dragonfly, int cgroup_id) override;

  inline NodeInCGKN* get_node(int chiplet_id) const {
    return static_cast<NodeInCGKN*>(Chip::get_node(NodeID(chiplet_id)));
  }
  inline NodeInCGKN* get_node(NodeID id) const {
    return static_cast<NodeInCGKN*>(Chip::get_node(id));
  }

  int coords_to_id(const std::vector<int>& coords) const;

  // torus_mode=false: pick boundary nodes (mesh); true: pick uniformly (torus)
  std::vector<int> select_external_nodes(bool torus_mode) const;

  DragonflyChipletKN* dragonfly_;
  int& num_chiplets_;
  int k_;
  int n_dims_;
  int cgroup_radix_;
  int& cgroup_id_;
  int wgroup_id_;

  std::vector<int> external_nodes_;
  std::map<int, int> node_to_ext_port_;
  std::map<int, int> ext_port_to_node_;
};

class DragonflyChipletKN : public System {
 public:
  DragonflyChipletKN();
  ~DragonflyChipletKN();

  void read_config() override;
  void connect_local();
  void connect_global();
  void routing_algorithm(Packet& s) const override;
  void MIN_routing(Packet& s) const;
  void XD_routing(Packet& s, NodeID dest, int vcb) const;

  inline NodeInCGKN* get_node(NodeID id) const {
    return static_cast<NodeInCGKN*>(System::get_node(id));
  }
  inline CGroupKN* get_cgroup(int cgroup_id) const {
    return static_cast<CGroupKN*>(chips_[cgroup_id]);
  }
  inline CGroupKN* get_cgroup(NodeID id) const {
    return static_cast<CGroupKN*>(get_chip(id.chip_id));
  }
  inline Port get_port(int cgroup_id, int node_id) const {
    NodeInCGKN* chiplet = get_node(NodeID(node_id, cgroup_id));
    return chiplet->ports_[chiplet->ext_port()];
  }

  std::pair<int, int> global_port_id_to_port_id(int global_port_id);

  std::string algorithm_;
  int k_node_in_CG_;
  int n_dims_;
  bool use_torus_;  // NEW: if true, add wrap-around links to form a torus C-group

  Channel internal_channel_;
  Channel external_channel_;

  int cgroup_radix_;
  int num_nodes_per_cg_;
  int num_chiplets_per_cg_;

  int l_ports_per_cg_;
  int g_ports_per_cg_;
  int g_ports_per_wg_;
  int cgroup_per_wgroup_;
  int num_wgroup_;
  int& num_cgroup_;

  bool mis_routing_;
  int max_wgroups_;
  bool single_cgroup_;

  std::map<int, int> port_node_map_;
  std::map<std::pair<int, int>, int> local_link_map_;
  std::map<std::pair<int, int>, Port> global_link_map_;

  std::vector<Chip*>& cgroups_;
};
