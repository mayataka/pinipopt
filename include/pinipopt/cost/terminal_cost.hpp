#ifndef PINIPOPT_TERMINAL_COST_HPP_
#define PINIPOPT_TERMINAL_COST_HPP_

#include <string>
#include <memory>

#include "Eigen/Core"
#include "ifopt/cost_term.h"

#include "pinipopt/robot/robot.hpp"
#include "pinipopt/variables/configuration.hpp"
#include "pinipopt/variables/velocity.hpp"


namespace pinipopt {

class TerminalCost : public ifopt::CostTerm {
public:
  TerminalCost(const Robot& robot, const int N);

  virtual ~TerminalCost() = default;

  void set_q_ref(const Eigen::VectorXd& q_ref);

  void set_v_ref(const Eigen::VectorXd& v_ref);

  void set_q_weight(const Eigen::VectorXd& q_weight);

  void set_v_weight(const Eigen::VectorXd& v_weight);

  void computeValues() const override;

  void computeJacobian() const override;

  double GetCost() const override;

  void FillJacobianBlock(std::string var_set, 
                         ifopt::Component::Jacobian& jac_block) const override;

private:
  int N_, dimv_;
  std::string q_str_, v_str_;
  Eigen::VectorXd q_ref_, v_ref_, q_weight_, v_weight_;
  mutable Eigen::VectorXd q_mutable_, v_mutable_, lq_mutable_, lv_mutable_;
  mutable double cost_mutable_;

  void setVariables() const;

  // double computeCost() const;

  // void computeJacobian() const;

  void InitVariableDependedQuantities(const VariablesPtr& x_init) override; 

};

} // namespace pinipopt

#endif // PINIPOPT_TERMINAL_COST_HPP_ 