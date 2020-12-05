#include "pinipopt/constraints/state_equation.hpp"


namespace pinipopt {

StateEquation::StateEquation(const Robot& robot, const double dtau, 
                             const int time_stage)
  : ConstraintSet(2*robot.dimv(), "state_equation_"+std::to_string(time_stage)),
    robot_(robot),
    dtau_(dtau),
    time_stage_(time_stage),
    q_str_("q_"+std::to_string(time_stage)),
    v_str_("v_"+std::to_string(time_stage)),
    u_str_("u_"+std::to_string(time_stage)),
    q_next_str_("q_"+std::to_string(time_stage+1)),
    v_next_str_("v_"+std::to_string(time_stage+1)),
    q_(Eigen::VectorXd::Zero(robot.dimq())),
    v_(Eigen::VectorXd::Zero(robot.dimv())),
    u_(Eigen::VectorXd::Zero(robot.dimu())),
    q_next_(Eigen::VectorXd::Zero(robot.dimq())),
    v_next_(Eigen::VectorXd::Zero(robot.dimv())),
    dq_(Eigen::VectorXd::Zero(robot.dimv())),
    dv_(Eigen::VectorXd::Zero(robot.dimv())),
    dx_(Eigen::VectorXd::Zero(2*robot.dimv())),
    dFq_dq_(Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv())), 
    dFq_dv_(Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv())),
    dABA_dq_(Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv())),
    dABA_dv_(Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv())),
    dABA_du_(Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv())) {
}


void StateEquation::setVariables() {
  q_ = GetVariables()->GetComponent(q_str_)->GetValues();
  v_ = GetVariables()->GetComponent(v_str_)->GetValues();
  u_ = GetVariables()->GetComponent(u_str_)->GetValues();
  q_next_ = GetVariables()->GetComponent(q_next_str_)->GetValues();
  v_next_ = GetVariables()->GetComponent(v_next_str_)->GetValues();
}


void StateEquation::updateConstraint() {
  robot_.stateEquation(q_, v_, u_, dq_, dv_);
  const int dimv = robot_.dimv();
  robot_.subtractConfiguration(q_next_, q_, dx_.head(dimv));
  dx_.head(dimv).noalias() -= dtau_ * dq_;
  dx_.tail(dimv) = v_next_ - v_ - dtau_ * dv_;
}


void StateEquation::updateJacobian() {
  robot_.ABADerivatives(q_, v_, u_, dABA_dq_, dABA_dv_, dABA_du_);
}


Eigen::VectorXd StateEquation::GetValues() const {
  return dx_;
}


ifopt::Composite::VecBound StateEquation::GetBounds() const {
  ifopt::Composite::VecBound bounds(GetRows());
  for (auto& e : bounds) {
    e = ifopt::BoundZero;
  }
  return bounds;
}


void StateEquation::FillJacobianBlock(
    std::string var_set, ifopt::Component::Jacobian& jac_block) const {
  const int dimv = robot_.dimv();
  const int dimu = robot_.dimu();
  if (var_set == q_str_) {
    for (int i=0; i<dimv; ++i) {
      jac_block.coeffRef(i, i) = 1;
    }
    for (int i=0; i<dimv; ++i) {
      for (int j=0; j<dimv; ++j) {
        jac_block.coeffRef(dimv+i, j) = dABA_dq_.coeffRef(i, j);
      }
    }
  }
  if (var_set == v_str_) {
    for (int i=0; i<dimv; ++i) {
      jac_block.coeffRef(i, dimv+i) = 1;
    }
    for (int i=0; i<dimv; ++i) {
      for (int j=0; j<dimv; ++j) {
        jac_block.coeffRef(dimv+i, dimv+j) = dABA_dv_.coeffRef(i, j);
      }
    }
  }
  if (var_set == u_str_) {
    const int dimx = 2*dimv;
    for (int i=0; i<dimv; ++i) {
      for (int j=0; j<dimu; ++j) {
        jac_block.coeffRef(dimv+i, dimx+j) = dABA_du_.coeffRef(i, j);
      }
    }
  }
  if (var_set == q_next_str_) {

  }
  if (var_set == v_next_str_) {

  }
}


void StateEquation::InitVariableDependedQuantities(
    const ifopt::Composite::Ptr& x_init) {
}

} // namespace pinipopt