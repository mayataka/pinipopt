#ifndef IPOPT_PINO_INITIAL_STATE_HPP_
#define IPOPT_PINO_INITIAL_STATE_HPP_

#include <string>
#include <memory>

#include "Eigen/Core"
#include "Eigen/Sparse"
#include "ifopt/composite.h"
#include "ifopt/constraint_set.h"

#include "ipopt-pino/robot/robot.hpp"


namespace ipoptpino {

class InitialState : public ifopt::ConstraintSet {
public:
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  InitialState(const Robot& robot);
  InitialState(const Robot& robot, const Eigen::VectorXd& q0, 
               const Eigen::VectorXd& v0);
  virtual ~InitialState() = default;

  void setInitialState(const Eigen::VectorXd& q0, const Eigen::VectorXd& v0);

  void setVariables();

  void updateConstraint();

  void updateJacobian();

  Eigen::VectorXd GetValues() const override;

  ifopt::Composite::VecBound GetBounds() const override;

  void FillJacobianBlock(std::string var_set, 
                         ifopt::Component::Jacobian& jac_block) const override;

private:
  Robot robot_;
  int dimv_;
  std::string q_str_, v_str_;
  Eigen::VectorXd q_, v_, q0_, v0_, dx_;
  Eigen::MatrixXd dFq_dq_, dFq_dv_;

  void InitVariableDependedQuantities(
      const ifopt::Composite::Ptr& x_init) override;

};
  
} // namespace ipoptpino

#endif // IPOPT_PINO_INITIAL_STATE_HPP_ 