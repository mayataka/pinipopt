#ifndef PINIPOPT_INITIAL_STATE_HPP_
#define PINIPOPT_INITIAL_STATE_HPP_

#include <string>
#include <memory>

#include "Eigen/Core"
#include "Eigen/Sparse"
#include "ifopt/composite.h"
#include "ifopt/constraint_set.h"

#include "pinipopt/robot/robot.hpp"


namespace pinipopt {

class InitialState : public ifopt::ConstraintSet {
public:
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  InitialState(const Robot& robot);
  InitialState(const Robot& robot, const Eigen::VectorXd& q0, 
               const Eigen::VectorXd& v0);
  virtual ~InitialState() = default;

  void setInitialState(const Eigen::VectorXd& q0, const Eigen::VectorXd& v0);

  void computeValues() const override;

  void computeJacobian() const override;

  Eigen::VectorXd GetValues() const override;

  ifopt::Composite::VecBound GetBounds() const override;

  void FillJacobianBlock(std::string var_set, 
                         ifopt::Component::Jacobian& jac_block) const override;

private:
  Robot robot_;
  int dimv_;
  std::string q_str_, v_str_;
  Eigen::VectorXd q0_, v0_;
  mutable Eigen::VectorXd q_mutable_, v_mutable_, dx_mutable_;

  void setVariables() const;

  // void computeViolation() const;

  // void computeJacobian() const;

  void InitVariableDependedQuantities(
      const ifopt::Composite::Ptr& x_init) override;

};
  
} // namespace pinipopt

#endif // PINIPOPT_INITIAL_STATE_HPP_ 