#include "pinipopt/cost/terminal_cost.hpp"


namespace pinipopt {

TerminalCost::TerminalCost(const Robot& robot, const int N) 
  : CostTerm("terminal_cost"),
    dimv_(robot.dimv()),
    q_str_("q_"+std::to_string(N)),
    v_str_("v_"+std::to_string(N)),
    q_ref_(Eigen::VectorXd::Zero(robot.dimq())),
    v_ref_(Eigen::VectorXd::Zero(robot.dimv())),
    q_weight_(Eigen::VectorXd::Zero(robot.dimv())),
    v_weight_(Eigen::VectorXd::Zero(robot.dimv())),
    q_mutable_(Eigen::VectorXd::Zero(robot.dimq())),
    v_mutable_(Eigen::VectorXd::Zero(robot.dimv())),
    lq_mutable_(Eigen::VectorXd::Zero(robot.dimv())),
    lv_mutable_(Eigen::VectorXd::Zero(robot.dimv())) {
}


void TerminalCost::set_q_ref(const Eigen::VectorXd& q_ref) {
  assert(q_ref.size() == q_ref_.size());
  q_ref_ = q_ref;
}


void TerminalCost::set_v_ref(const Eigen::VectorXd& v_ref) {
  assert(v_ref.size() == v_ref_.size());
  v_ref_ = v_ref;
}


void TerminalCost::set_q_weight(const Eigen::VectorXd& q_weight) {
  assert(q_weight.size() == q_weight_.size());
  q_weight_ = q_weight;
}


void TerminalCost::set_v_weight(const Eigen::VectorXd& v_weight) {
  assert(v_weight.size() == v_weight_.size());
  v_weight_ = v_weight;
}


double TerminalCost::GetCost() const {
  setVariables();
  return computeCost();
}


void TerminalCost::FillJacobianBlock(
    std::string var_set, ifopt::Component::Jacobian& jac_block) const {
  setVariables();
  computeJacobian();
  if (var_set == q_str_) {
    for (int i=0; i<dimv_; ++i) {
      jac_block.coeffRef(0, i) = lq_mutable_.coeff(i);
    }
  }
  if (var_set == v_str_) {
    for (int i=0; i<dimv_; ++i) {
      jac_block.coeffRef(0, i) = lv_mutable_.coeff(i);
    }
  }
}


void TerminalCost::setVariables() const {
  q_mutable_ = GetVariables()->GetComponent(q_str_)->GetValues();
  v_mutable_ = GetVariables()->GetComponent(v_str_)->GetValues();
}


double TerminalCost::computeCost() const {
  double cost = 0;
  cost += 0.5 * (q_weight_.array() * (q_mutable_-q_ref_).array() * (q_mutable_-q_ref_).array()).sum();
  cost += 0.5 * (v_weight_.array() * (v_mutable_-v_ref_).array() * (v_mutable_-v_ref_).array()).sum();
  return cost;
}


void TerminalCost::computeJacobian() const {
  lq_mutable_.array() = q_weight_.array() * (q_mutable_-q_ref_).array();
  lv_mutable_.array() = v_weight_.array() * (v_mutable_-v_ref_).array();
}


void TerminalCost::InitVariableDependedQuantities(const VariablesPtr& x_init) {
}

} // namespace pinipopt