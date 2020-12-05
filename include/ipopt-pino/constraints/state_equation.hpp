#ifndef IPOPT_PINO_STATE_EQUATION_HPP_
#define IPOPT_PINO_STATE_EQUATION_HPP_

#include <string>
#include <memory>

#include "Eigen/Core"
#include "Eigen/Sparse"
#include "ifopt/composite.h"
#include "ifopt/constraint_set.h"

#include "ipopt-pino/robot/robot.hpp"


namespace ipoptpino {

class StateEquation : public ifopt::ConstraintSet {
public:
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  StateEquation(const Robot& robot, const double dtau, const int time_stage);
  virtual ~StateEquation() = default;

  void setVariables();

  void updateConstraint();

  void updateJacobian();

  Eigen::VectorXd GetValues() const override;

  ifopt::Composite::VecBound GetBounds() const override;

  void FillJacobianBlock(std::string var_set, 
                         ifopt::Component::Jacobian& jac_block) const override;

private:
  Robot robot_;
  double dtau_;
  int time_stage_;
  std::string q_str_, v_str_, u_str_, q_next_str_, v_next_str_;
  Eigen::VectorXd q_, v_, u_, q_next_, v_next_, dq_, dv_, dx_;
  Eigen::MatrixXd dFq_dq_, dFq_dv_, dABA_dq_, dABA_dv_, dABA_du_;

  void InitVariableDependedQuantities(
      const ifopt::Composite::Ptr& x_init) override;

};
  
} // namespace ipoptpino

#endif // IPOPT_PINO_STATE_EQUATION_HPP_ 