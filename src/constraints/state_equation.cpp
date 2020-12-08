#include "pinipopt/constraints/state_equation.hpp"


namespace pinipopt {

StateEquation::StateEquation(const Robot& robot, const double dtau, 
                             const int time_stage)
  : ConstraintSet(2*robot.dimv(), "state_equation_"+std::to_string(time_stage)),
    robot_mutable_(robot),
    dtau_(dtau),
    time_stage_(time_stage),
    q_str_("q_"+std::to_string(time_stage)),
    v_str_("v_"+std::to_string(time_stage)),
    u_str_("u_"+std::to_string(time_stage)),
    q_next_str_("q_"+std::to_string(time_stage+1)),
    v_next_str_("v_"+std::to_string(time_stage+1)),
    q_mutable_(Eigen::VectorXd::Zero(robot.dimq())),
    v_mutable_(Eigen::VectorXd::Zero(robot.dimv())),
    u_mutable_(Eigen::VectorXd::Zero(robot.dimu())),
    q_next_mutable_(Eigen::VectorXd::Zero(robot.dimq())),
    v_next_mutable_(Eigen::VectorXd::Zero(robot.dimv())),
    dq_mutable_(Eigen::VectorXd::Zero(robot.dimv())),
    dv_mutable_(Eigen::VectorXd::Zero(robot.dimv())),
    dx_mutable_(Eigen::VectorXd::Zero(2*robot.dimv())),
    dABA_dq_mutable_(Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv())),
    dABA_dv_mutable_(Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv())),
    dABA_du_mutable_(Eigen::MatrixXd::Zero(robot.dimv(), robot.dimu())) {
}


void StateEquation::computeValues() const {
  setVariables();
  robot_mutable_.stateEquation(q_mutable_, v_mutable_, u_mutable_, 
                               dq_mutable_, dv_mutable_);
  const int dimv = robot_mutable_.dimv();
  dx_mutable_.head(dimv) = q_mutable_ + dtau_ * dq_mutable_ - q_next_mutable_;
  dx_mutable_.tail(dimv) = v_mutable_ + dtau_ * dv_mutable_ - v_next_mutable_;
}


void StateEquation::computeJacobian() const {
  setVariables();
  robot_mutable_.ABADerivatives(q_mutable_, v_mutable_, u_mutable_, 
                                dABA_dq_mutable_, dABA_dv_mutable_, 
                                dABA_du_mutable_);
}


Eigen::VectorXd StateEquation::GetValues() const {
  return dx_mutable_;
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
  const int dimv = robot_mutable_.dimv();
  const int dimu = robot_mutable_.dimu();
  if (var_set == q_str_) {
    for (int i=0; i<dimv; ++i) {
      jac_block.coeffRef(i, i) = 1;
    }
    for (int i=0; i<dimv; ++i) {
      for (int j=0; j<dimv; ++j) {
        jac_block.coeffRef(dimv+i, j) = dtau_ * dABA_dq_mutable_.coeff(i, j);
      }
    }
  }
  if (var_set == v_str_) {
    for (int i=0; i<dimv; ++i) {
      jac_block.coeffRef(i, i) = dtau_;
    }
    for (int i=0; i<dimv; ++i) {
      for (int j=0; j<dimv; ++j) {
        jac_block.coeffRef(dimv+i, j) = dtau_ * dABA_dv_mutable_.coeff(i, j);
      }
      jac_block.coeffRef(dimv+i, i) += 1;
    }
  }
  if (var_set == u_str_) {
    for (int i=0; i<dimv; ++i) {
      for (int j=0; j<dimu; ++j) {
        jac_block.coeffRef(dimv+i, j) = dtau_ * dABA_du_mutable_.coeff(i, j);
      }
    }
  }
  if (var_set == q_next_str_) {
    for (int i=0; i<dimv; ++i) {
      jac_block.coeffRef(i, i) = -1;
    }
  }
  if (var_set == v_next_str_) {
    for (int i=0; i<dimv; ++i) {
      jac_block.coeffRef(dimv+i, i) = -1;
    }
  }
}


void StateEquation::setVariables() const {
  q_mutable_ = GetVariables()->GetComponent(q_str_)->GetValues();
  v_mutable_ = GetVariables()->GetComponent(v_str_)->GetValues();
  u_mutable_ = GetVariables()->GetComponent(u_str_)->GetValues();
  q_next_mutable_ = GetVariables()->GetComponent(q_next_str_)->GetValues();
  v_next_mutable_ = GetVariables()->GetComponent(v_next_str_)->GetValues();
}


// void StateEquation::computeViolation() const {
// }


// void StateEquation::computeJacobian() const {
// }


void StateEquation::InitVariableDependedQuantities(
    const ifopt::Composite::Ptr& x_init) {
}

} // namespace pinipopt