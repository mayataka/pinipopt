#include "ipopt-pino/variables/configuration.hpp"


namespace ipoptpino {

Configuration::Configuration(const Robot& robot, const int time_stage) 
  : VariableSet(robot.dimq(), "q_"+std::to_string(time_stage)),
    q_(Eigen::VectorXd::Zero(robot.dimq())) {
}


void Configuration::SetVariables(const Eigen::VectorXd& q) {
  assert(q.size() == q_.size());
  q_ = q;
}


Eigen::VectorXd Configuration::GetValues() const {
  return q_;
}


ifopt::Composite::VecBound Configuration::GetBounds() const {
  ifopt::Composite::VecBound bounds(GetRows());
  for (auto& e : bounds) {
    e = ifopt::NoBound;
  }
  return bounds;
}

} // namespace ipoptpino