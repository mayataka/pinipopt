#ifndef PINIPOPT_FLOATING_BASE_HXX_
#define PINIPOPT_FLOATING_BASE_HXX_

#include "pinipopt/robot/floating_base.hpp"

namespace pinipopt {

inline FloatingBase::FloatingBase(const pinocchio::Model& model) 
  : has_floating_base_(false),
    passive_joint_indices_(),
    dimv_(model.nv),
    dim_passive_(0) {
  int total_dim_torque = -1; // Note that joint 0 is always universe.
  for (const auto& joint : model.joints) {
    if (joint.shortname() == "JointModelFreeFlyer") {
      passive_joint_indices_.push_back(total_dim_torque);
      passive_joint_indices_.push_back(total_dim_torque+1);
      passive_joint_indices_.push_back(total_dim_torque+2);
      passive_joint_indices_.push_back(total_dim_torque+3);
      passive_joint_indices_.push_back(total_dim_torque+4);
      passive_joint_indices_.push_back(total_dim_torque+5);
      total_dim_torque += 6;
      has_floating_base_ = true;
    }
    else {
      total_dim_torque += 1;
    }
  }
  if (has_floating_base_) {
    dim_passive_ = 6;
  }
}


inline FloatingBase::FloatingBase() 
  : has_floating_base_(false),
    passive_joint_indices_(),
    dimv_(0),
    dim_passive_(0) {
}


inline FloatingBase::~FloatingBase() {
}


inline int FloatingBase::dim_passive() const {
  return dim_passive_;
}


inline std::vector<int> FloatingBase::passive_joint_indices() const {
  return passive_joint_indices_;
}


inline bool FloatingBase::has_floating_base() const {
  return has_floating_base_;
}

} // namespace pinipopt 

#endif // PINIPOPT_FLOATING_BASE_HXX_ 