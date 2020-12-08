#ifndef PINIPOPT_STATE_EQUATION_HPP_
#define PINIPOPT_STATE_EQUATION_HPP_

#include <string>
#include <memory>

#include "Eigen/Core"
#include "Eigen/Sparse"
#include "ifopt/composite.h"
#include "ifopt/constraint_set.h"

#include "pinipopt/robot/robot.hpp"


namespace pinipopt {

class StateEquation : public ifopt::ConstraintSet {
public:
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  StateEquation(const Robot& robot, const double dtau, const int time_stage);
  virtual ~StateEquation() = default;

  void computeValues() const override;

  void computeJacobian() const override;

  Eigen::VectorXd GetValues() const override;

  ifopt::Composite::VecBound GetBounds() const override;

  void FillJacobianBlock(std::string var_set, 
                         ifopt::Component::Jacobian& jac_block) const override;

private:
  mutable Robot robot_mutable_;
  double dtau_;
  int time_stage_;
  std::string q_str_, v_str_, u_str_, q_next_str_, v_next_str_;
  mutable Eigen::VectorXd q_mutable_, v_mutable_, u_mutable_, q_next_mutable_, 
                          v_next_mutable_, dq_mutable_, dv_mutable_, dx_mutable_;
  mutable Eigen::MatrixXd dABA_dq_mutable_, dABA_dv_mutable_, dABA_du_mutable_;

  void setVariables() const;

  // void computeViolation() const;

  // void computeJacobian() const;

  void InitVariableDependedQuantities(
      const ifopt::Composite::Ptr& x_init) override;

};
  
} // namespace pinipopt

#endif // PINIPOPT_STATE_EQUATION_HPP_ 