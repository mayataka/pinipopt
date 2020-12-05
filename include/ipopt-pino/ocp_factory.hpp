#ifndef IPOPT_PINO_OCP_FACTORY_HPP_
#define IPOPT_PINO_OCP_FACTORY_HPP_

#include <memory>

#include "Eigen/Core"
#include "ifopt/variable_set.h"
#include "ifopt/constraint_set.h"
#include "ifopt/cost_term.h"

#include "ipopt-pino/robot/robot.hpp"
#include "ipopt-pino/variables/configuration.hpp"
#include "ipopt-pino/variables/velocity.hpp"
#include "ipopt-pino/variables/torques.hpp"
#include "ipopt-pino/constraints/initial_state.hpp"
#include "ipopt-pino/constraints/state_equation.hpp"
#include "ipopt-pino/cost/stage_cost.hpp"
#include "ipopt-pino/cost/terminal_cost.hpp"


namespace ipoptpino {

class OCPFactory {
public:
  using VariablePtrVec   = std::vector<ifopt::VariableSet::Ptr>;
  using ContraintPtrVec  = std::vector<ifopt::ConstraintSet::Ptr>;
  using CostPtrVec       = std::vector<ifopt::CostTerm::Ptr>;

  OCPFactory(const Robot& robot, const double T, const int N);
  virtual ~OCPFactory() = default;

  VariablePtrVec GetVariableSets() const;

  ContraintPtrVec GetConstraints() const;

  ContraintPtrVec GetConstraints(const Eigen::VectorXd& q0, 
                                 const Eigen::VectorXd& v0) const;

  CostPtrVec GetCosts() const;

private:
  Robot robot_;
  double T_, dtau_;
  int N_;

};
  
} // namespace ipoptpino

#endif // IPOPT_PINO_OCP_FACTORY_HPP_ 