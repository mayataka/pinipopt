#include "ipopt-pino/constraints/initial_state.hpp"


namespace ipoptpino {

InitialState::InitialState(const Robot& robot)
  : ConstraintSet(2*robot.dimv(), "initial_state"),
    robot_(robot),
    dimv_(robot.dimv()),
    q_str_("q_0"),
    v_str_("v_0"),
    q_(Eigen::VectorXd::Zero(robot.dimq())),
    v_(Eigen::VectorXd::Zero(robot.dimv())),
    q0_(Eigen::VectorXd::Zero(robot.dimq())),
    v0_(Eigen::VectorXd::Zero(robot.dimv())),
    dx_(Eigen::VectorXd::Zero(2*robot.dimv())) {
}


InitialState::InitialState(const Robot& robot, const Eigen::VectorXd& q0, 
                           const Eigen::VectorXd& v0)
  : ConstraintSet(2*robot.dimv(), "initial_state"),
    robot_(robot),
    dimv_(robot.dimv()),
    q_str_("q_0"),
    v_str_("v_0"),
    q_(Eigen::VectorXd::Zero(robot.dimq())),
    v_(Eigen::VectorXd::Zero(robot.dimv())),
    q0_(Eigen::VectorXd::Zero(robot.dimq())),
    v0_(Eigen::VectorXd::Zero(robot.dimv())),
    dx_(Eigen::VectorXd::Zero(2*robot.dimv())) {
  setInitialState(q0, v0);
}


void InitialState::setInitialState(const Eigen::VectorXd& q0, 
                                   const Eigen::VectorXd& v0) {
  assert(q0_.size() == q0.size());
  assert(v0_.size() == v0.size());
  q0_= q0;
  v0_= v0;
}


void InitialState::setVariables() {
  q_ = GetVariables()->GetComponent(q_str_)->GetValues();
  v_ = GetVariables()->GetComponent(v_str_)->GetValues();
}


void InitialState::updateConstraint() {
  dx_.head(dimv_) = q_ - q0_;
  dx_.tail(dimv_) = v_ - v0_;
}


void InitialState::updateJacobian() {
}


Eigen::VectorXd InitialState::GetValues() const {
  return dx_;
}


ifopt::Composite::VecBound InitialState::GetBounds() const {
  ifopt::Composite::VecBound bounds(GetRows());
  for (auto& e : bounds) {
    e = ifopt::BoundZero;
  }
  return bounds;
}


void InitialState::FillJacobianBlock(
    std::string var_set, ifopt::Component::Jacobian& jac_block) const {
  if (var_set == q_str_) {
    for (int i=0; i<dimv_; ++i) {
      jac_block.coeffRef(i, i) = 1;
    }
  }
  if (var_set == v_str_) {
    for (int i=0; i<dimv_; ++i) {
      jac_block.coeffRef(i, dimv_+i) = 1;
    }
  }
}

} // namespace ipoptpino