#ifndef PINIPOPT_OCP_HPP_
#define PINIPOPT_OCP_HPP_

#include <vector>
#include <memory>

#include "Eigen/Core"
#include "ifopt/variable_set.h"
#include "ifopt/constraint_set.h"
#include "ifopt/cost_term.h"
#include "ifopt/problem.h"
#include "ifopt/ipopt_solver.h"

#include "pinipopt/robot/robot.hpp"
#include "pinipopt/variables/configuration.hpp"
#include "pinipopt/variables/velocity.hpp"
#include "pinipopt/variables/torques.hpp"
#include "pinipopt/constraints/initial_state.hpp"
#include "pinipopt/constraints/state_equation.hpp"
#include "pinipopt/cost/stage_cost.hpp"
#include "pinipopt/cost/terminal_cost.hpp"


namespace pinipopt {

class OCP {
public:
  using VariablePtrVec   = std::vector<ifopt::VariableSet::Ptr>;
  using ConstraintPtrVec  = std::vector<ifopt::ConstraintSet::Ptr>;
  using CostPtrVec       = std::vector<ifopt::CostTerm::Ptr>;

  OCP(const Robot& robot, const double T, const int N);
  virtual ~OCP() = default;

  void solve(const Eigen::VectorXd& q0, const Eigen::VectorXd& v0);

  void setIpoptOption(const std::string& name, const std::string& value);

  void setIpoptOption(const std::string& name, int value);

  void setIpoptOption(const std::string& name, double value);

  void createOCP(const Robot& robot, const int N, const double dtau);

  void createVariableSets(const Robot& robot, const int N, const double dtau);

  void createConstraints(const Robot& robot, const int N, const double dtau);

  void createCosts(const Robot& robot, const int N, const double dtau);

private:
  ifopt::Problem ocp_;
  ifopt::IpoptSolver ipopt_;
  Robot robot_;
  std::shared_ptr<InitialState> initial_state_constraints_;
  VariablePtrVec vars_;
  ConstraintPtrVec constraints_;
  CostPtrVec costs_;
  double T_, dtau_;
  int N_;

};
  
} // namespace pinipopt

#endif // PINIPOPT_OCP_HPP_ 