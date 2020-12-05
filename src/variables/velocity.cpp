#include "ipopt-pino/variables/velocity.hpp"


namespace ipoptpino {

Velocity::Velocity(const Robot& robot, const int time_stage) 
  : VariableSet(robot.dimu(), "v_"+std::to_string(time_stage)),
    v_(Eigen::VectorXd::Zero(robot.dimv())) {
}


void Velocity::SetVariables(const Eigen::VectorXd& v) {
  assert(v.size() == v_.size());
  v_ = v;
}


Eigen::VectorXd Velocity::GetValues() const {
  return v_;
}


ifopt::Composite::VecBound Velocity::GetBounds() const {
  ifopt::Composite::VecBound bounds(GetRows());
  for (auto& e : bounds) {
    e = ifopt::NoBound;
  }
  return bounds;
}

} // namespace ipoptpino