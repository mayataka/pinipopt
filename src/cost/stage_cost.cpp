#include "pinipopt/cost/stage_cost.hpp"


namespace pinipopt {

StageCost::StageCost(const Robot& robot, const double dtau, 
                     const int time_stage) 
  : CostTerm("stage_cost_"+std::to_string(time_stage)),
    dtau_(dtau),
    dimv_(robot.dimv()),
    dimu_(robot.dimu()),
    q_str_("q_"+std::to_string(time_stage)),
    v_str_("v_"+std::to_string(time_stage)),
    u_str_("u_"+std::to_string(time_stage)),
    q_ref_(Eigen::VectorXd::Zero(robot.dimq())),
    v_ref_(Eigen::VectorXd::Zero(robot.dimv())),
    u_ref_(Eigen::VectorXd::Zero(robot.dimu())),
    q_weight_(Eigen::VectorXd::Zero(robot.dimv())),
    v_weight_(Eigen::VectorXd::Zero(robot.dimv())),
    u_weight_(Eigen::VectorXd::Zero(robot.dimu())),
    q_mutable_(Eigen::VectorXd::Zero(robot.dimq())),
    v_mutable_(Eigen::VectorXd::Zero(robot.dimv())),
    u_mutable_(Eigen::VectorXd::Zero(robot.dimu())),
    lq_mutable_(Eigen::VectorXd::Zero(robot.dimv())),
    lv_mutable_(Eigen::VectorXd::Zero(robot.dimv())),
    lu_mutable_(Eigen::VectorXd::Zero(robot.dimu())),
    cost_mutable_(0) {
}


void StageCost::set_q_ref(const Eigen::VectorXd& q_ref) {
  assert(q_ref.size() == q_ref_.size());
  q_ref_ = q_ref;
}


void StageCost::set_v_ref(const Eigen::VectorXd& v_ref) {
  assert(v_ref.size() == v_ref_.size());
  v_ref_ = v_ref;
}


void StageCost::set_u_ref(const Eigen::VectorXd& u_ref) {
  assert(u_ref.size() == u_ref_.size());
  u_ref_ = u_ref;
}


void StageCost::set_q_weight(const Eigen::VectorXd& q_weight) {
  assert(q_weight.size() == q_weight_.size());
  q_weight_ = q_weight;
}


void StageCost::set_v_weight(const Eigen::VectorXd& v_weight) {
  assert(v_weight.size() == v_weight_.size());
  v_weight_ = v_weight;
}


void StageCost::set_u_weight(const Eigen::VectorXd& u_weight) {
  assert(u_weight.size() == u_weight_.size());
  u_weight_ = u_weight;
}


void StageCost::computeValues() const {
  setVariables();
  cost_mutable_ = 0;
  cost_mutable_ += 0.5 * dtau_ * (q_weight_.array() * (q_mutable_-q_ref_).array() * (q_mutable_-q_ref_).array()).sum();
  cost_mutable_ += 0.5 * dtau_ * (v_weight_.array() * (v_mutable_-v_ref_).array() * (v_mutable_-v_ref_).array()).sum();
  cost_mutable_ += 0.5 * dtau_ * (u_weight_.array() * (u_mutable_-u_ref_).array() * (u_mutable_-u_ref_).array()).sum();
}


void StageCost::computeJacobian() const {
  setVariables();
  lq_mutable_.array() = dtau_ * q_weight_.array() * (q_mutable_-q_ref_).array();
  lv_mutable_.array() = dtau_ * v_weight_.array() * (v_mutable_-v_ref_).array();
  lu_mutable_.array() = dtau_ * u_weight_.array() * (u_mutable_-u_ref_).array();
}


double StageCost::GetCost() const {
  return cost_mutable_;
}


void StageCost::FillJacobianBlock(
    std::string var_set, ifopt::Component::Jacobian& jac_block) const {
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
  if (var_set == u_str_) {
    const int dimx = 2*dimv_;
    for (int i=0; i<dimu_; ++i) {
      jac_block.coeffRef(0, i) = lu_mutable_.coeff(i);
    }
  }
}


void StageCost::setVariables() const {
  q_mutable_ = GetVariables()->GetComponent(q_str_)->GetValues();
  v_mutable_ = GetVariables()->GetComponent(v_str_)->GetValues();
  u_mutable_ = GetVariables()->GetComponent(u_str_)->GetValues();
}


void StageCost::InitVariableDependedQuantities(const VariablesPtr& x_init) {
}

} // namespace pinipopt