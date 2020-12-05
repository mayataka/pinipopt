#include "ipopt-pino/variables/torques.hpp"


namespace ipoptpino {

Torques::Torques(const Robot& robot, const int time_stage) 
  : VariableSet(robot.dimu(), "u_"+std::to_string(time_stage)),
    u_(Eigen::VectorXd::Zero(robot.dimu())) {
}


void Torques::SetVariables(const Eigen::VectorXd& u) {
  assert(u.size() == u_.size());
  u_ = u;
}


Eigen::VectorXd Torques::GetValues() const {
  return u_;
}


ifopt::Composite::VecBound Torques::GetBounds() const {
  ifopt::Composite::VecBound bounds(GetRows());
  for (auto& e : bounds) {
    e = ifopt::NoBound;
  }
  return bounds;
}

} // namespace ipoptpino