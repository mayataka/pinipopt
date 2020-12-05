#ifndef IPOPT_PINO_TERMINAL_COST_HPP_
#define IPOPT_PINO_TERMINAL_COST_HPP_

#include <string>
#include <memory>

#include "Eigen/Core"
#include "ifopt/cost_term.h"

#include "ipopt-pino/robot/robot.hpp"


namespace ipoptpino {

class TerminalCost : public ifopt::CostTerm {
public:
  TerminalCost(const Robot& robot, const int N);

  virtual ~TerminalCost() = default;

  void set_q_ref(const Eigen::VectorXd& q_ref);

  void set_v_ref(const Eigen::VectorXd& v_ref);

  void set_q_weight(const Eigen::VectorXd& q_weight);

  void set_v_weight(const Eigen::VectorXd& v_weight);

  void setVariables();

  void updateCost();

  void updateJacobian();

  double GetCost() const override;

  void FillJacobianBlock(std::string var_set, 
                         ifopt::Component::Jacobian& jac_block) const override;

private:
  int N_, dimv_;
  std::string q_str_, v_str_;
  Eigen::VectorXd q_ref_, v_ref_, q_weight_, v_weight_, q_, v_, lq_, lv_;
};

} // namespace ipoptpino

#endif // IPOPT_PINO_TERMINAL_COST_HPP_ 