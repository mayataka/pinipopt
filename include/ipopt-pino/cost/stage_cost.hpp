#ifndef IPOPT_PINO_STAGE_COST_HPP_
#define IPOPT_PINO_STAGE_COST_HPP_

#include <string>
#include <memory>

#include "Eigen/Core"
#include "ifopt/cost_term.h"

#include "ipopt-pino/robot/robot.hpp"


namespace ipoptpino {

class StageCost : public ifopt::CostTerm {
public:
  StageCost(const Robot& robot, const double dtau, const int time_stage);

  virtual ~StageCost() = default;

  void set_q_ref(const Eigen::VectorXd& q_ref);

  void set_v_ref(const Eigen::VectorXd& v_ref);

  void set_u_ref(const Eigen::VectorXd& u_ref);

  void set_q_weight(const Eigen::VectorXd& q_weight);

  void set_v_weight(const Eigen::VectorXd& v_weight);

  void set_u_weight(const Eigen::VectorXd& u_weight);

  void setVariables();

  void updateCost();

  void updateJacobian();

  double GetCost() const override;

  void FillJacobianBlock(std::string var_set, 
                         ifopt::Component::Jacobian& jac_block) const override;

private:
  double dtau_, cost_;
  int dimv_, dimu_;
  std::string q_str_, v_str_, u_str_;
  Eigen::VectorXd q_ref_, v_ref_, u_ref_, q_weight_, v_weight_, u_weight_, 
                  q_, v_, u_, lq_, lv_, lu_;

};
  
} // namespace ipoptpino

#endif // IPOPT_PINO_STAGE_COST_HPP_ 