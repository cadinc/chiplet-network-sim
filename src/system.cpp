#include "system.h"

#include "dragonfly_chiplet.h"
#include "dragonfly_chiplet_fc.h"   // <-- new
#include "dragonfly_sw.h"
#include "multiple_chip_mesh.h"
#include "multiple_chip_torus.h"
#include "single_chip_mesh.h"
#include "traffic_manager.h"

System::System() {
  num_chips_ = 0;
  num_nodes_ = 0;
  num_cores_ = 0;

  // router parameters
  router_stages_ = param->router_stages;

  // simulation parameters
  timeout_time_ = param->timeout_threshold;
}

System* System::New(const std::string& topology) {
  System* sys_ptr;
  if (topology == "SingleChipMesh")
    sys_ptr = new SingleChipMesh;
  else if (topology == "MultiChipMesh")
    sys_ptr = new MultiChipMesh;
  else if (topology == "MultiChipTorus")
    sys_ptr = new MultiChipTorus;
  else if (topology == "DragonflySW")
    sys_ptr = new DragonflySW;
  else if (topology == "DragonflyChiplet")
    sys_ptr = new DragonflyChiplet;
  else if (topology == "DragonflyChipletFC")   // <-- new
    sys_ptr = new DragonflyChipletFC;
  else {
    std::cerr << "No such a topology!" << std::endl;
    return nullptr;
  }
  return sys_ptr;
}

void System::reset() {
  for (auto chip : chips_) {
    chip->reset();
  }
}

void System::onestage(Packet& p) {
  if (p.candidate_channels_.empty()) routing(p);
  if (!p.candidate_channels_.empty() && p.next_vc_.buffer == nullptr)
    vc_allocate(p);
  if (p.next_vc_.buffer != nullptr && p.switch_allocated_ == false)
    switch_allocate(p);
}

void System::twostage(Packet& p) {
  if (p.candidate_channels_.empty()) routing(p);
  if (!p.candidate_channels_.empty() && p.next_vc_.buffer == nullptr)
    vc_allocate(p);
  else if (p.next_vc_.buffer != nullptr && p.switch_allocated_ == false)
    switch_allocate(p);
}

void System::Threestage(Packet& p) {
  if (p.candidate_channels_.empty())
    routing(p);
  else if (!p.candidate_channels_.empty() && p.next_vc_.buffer == nullptr)
    vc_allocate(p);
  else if (p.next_vc_.buffer != nullptr && p.switch_allocated_ == false)
    switch_allocate(p);
}

void System::routing(Packet& p) const {
  assert(p.candidate_channels_.empty());
  routing_algorithm(p);
  assert(!p.candidate_channels_.empty());
}

void System::vc_allocate(Packet& p) const {
  VCInfo current_vc = p.head_trace();
  if (current_vc.buffer == nullptr ||
      current_vc.head_packet() == &p) {
    for (auto& vc : p.candidate_channels_) {
      if (vc.buffer->is_empty(vc.vcb))
        if (vc.buffer->allocate_buffer(vc.vcb, p.length_)) {
          p.next_vc_ = vc;
          return;
        }
    }
    for (auto& vc : p.candidate_channels_) {
      if (vc.buffer->allocate_buffer(vc.vcb, p.length_)) {
        p.next_vc_ = vc;
        return;
      }
    }
  }
}

void System::switch_allocate(Packet& p) {
  VCInfo current_vc = p.head_trace();
  if (current_vc.buffer == nullptr) {
    if (p.next_vc_.buffer->allocate_in_link(p)) {
      p.switch_allocated_ = true;
    }
  } else if (current_vc.head_packet() == &p) {
    if (current_vc.buffer->allocate_sw_link()) {
      if (p.next_vc_.buffer->allocate_in_link(p)) {
        p.switch_allocated_ = true;
      } else
        current_vc.buffer->release_sw_link();
    }
  }
}

void System::update(Packet& p) {
  assert(p.link_timer_ > 0 || p.destination_ != p.tail_trace().id);

  p.trans_timer_++;
  if (p.wait_timer_ == timeout_time_)
    TM->message_timeout_++;

  if (p.head_trace().id == p.source_ && p.process_timer_ > 0) {
    p.process_timer_--;
    return;
  }

  if (p.link_timer_ == 0) {
    if (p.head_trace().id != p.destination_) {
      if (router_stages_ == "OneStage") {
        onestage(p);
      } else if (router_stages_ == "TwoStage") {
        twostage(p);
      } else if (router_stages_ == "ThreeStage") {
        Threestage(p);
      } else {
        std::cerr << "No such a microarchitecture!" << std::endl;
      }
      if (!p.switch_allocated_) p.wait_timer_++;
    }
  } else {
    p.link_timer_--;
  }

  VCInfo temp1, temp2;
  int i = 0;

  if (p.switch_allocated_) {
    temp1 = p.next_vc_;
    p.wait_timer_ = 0;
    p.link_timer_ = p.next_vc_.buffer->channel_.latency;
#ifdef DEBUG
    TM->traffic_map_[temp1.buffer]++;
#endif
    if (temp1.buffer->channel_ == on_chip_channel)
      p.internal_hops_++;
    else if (temp1.buffer->channel_ == off_chip_parallel_channel)
      p.parallel_hops_++;
    else if (temp1.buffer->channel_ == off_chip_serial_channel)
      p.serial_hops_++;
    else
      p.other_hops_++;
    p.candidate_channels_.clear();
    p.next_vc_ = VCInfo();
    p.switch_allocated_ = false;
  } else {
    temp1 = p.head_trace();
    while (i < p.length_ && p.flit_trace_[i].id == temp1.id) i++;
  }

  if (i < p.length_) {
    temp2 = p.flit_trace_[i];
    int k = temp1.buffer->channel_.width;
    int j = 0;
    while (i < p.length_) {
      if (p.flit_trace_[i].id == temp2.id && j < k) {
        assert(p.flit_trace_[i].id != temp1.id);
        p.flit_trace_[i] = temp1;
        j++;
      } else {
        if (p.flit_trace_[i].id != temp2.id) {
          temp1 = temp2;
          temp2 = p.flit_trace_[i];
          k = temp1.buffer->channel_.width;
          j = 0;
          assert(p.flit_trace_[i].id != temp1.id);
          p.flit_trace_[i] = temp1;
          j++;
        } else {
          assert(j == k);
        }
      }
      i++;
    }
    if (temp2.id != p.tail_trace().id) {
      p.releaselink_ = true;
      p.leaving_vc_ = temp2;
      if (temp2.buffer != nullptr) {
        temp2.buffer->release_buffer(temp2.vcb, p.length_);
      }
    }
  }
  if (p.link_timer_ == 0 && p.tail_trace().id == p.destination_) {
    VCInfo dest_vc = p.tail_trace();
    dest_vc.buffer->release_buffer(dest_vc.vcb, p.length_);
    p.finished_ = true;
    TM->message_arrived_++;
    TM->total_cycles_ += p.trans_timer_;
    TM->total_parallel_hops_ += p.parallel_hops_;
    TM->total_serial_hops_ += p.serial_hops_;
    TM->total_internal_hops_ += p.internal_hops_;
    TM->total_other_hops_ += p.other_hops_;
    return;
  }
}