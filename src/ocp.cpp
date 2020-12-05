#include "pinipopt/ocp.hpp"


namespace pinipopt {

OCP::OCP(const Robot& robot, const double T, const int N) 
  : robot_(robot),
    T_(T), 
    dtau_(T/N),
    N_(N) {
  createOCP(robot, N, dtau_);
}


void OCP::solve(const Eigen::VectorXd& q0, const Eigen::VectorXd& v0) {
  initial_state_constraints_->setInitialState(q0, v0);
  ipopt_.Solve(ocp_);
}


void OCP::setIpoptOption(const std::string& name, const std::string& value) {
  ipopt_.SetOption(name, value);
}


void OCP::setIpoptOption(const std::string& name, int value) {
  ipopt_.SetOption(name, value);
}


void OCP::setIpoptOption(const std::string& name, double value) {
  ipopt_.SetOption(name, value);
}


void OCP::createOCP(const Robot& robot, const int N, const double dtau) {
  createVariableSets(robot, N, dtau);
  createConstraints(robot, N, dtau);
  createCosts(robot, N, dtau);
  ocp_ = ifopt::Problem();
  for (auto& e : vars_) {
    ocp_.AddVariableSet(e);
  }
  for (auto& e : constraints_) {
    ocp_.AddConstraintSet(e);
  }
  for (auto& e : costs_) {
    ocp_.AddCostSet(e);
  }
}


void OCP::createVariableSets(const Robot& robot, const int N, const double dtau) {
  vars_.clear();
  for (int i=0; i<N; ++i) {
    vars_.push_back(std::make_shared<Configuration>(robot, i));
    vars_.push_back(std::make_shared<Velocity>(robot, i));
    vars_.push_back(std::make_shared<Torques>(robot, i));
  }
  vars_.push_back(std::make_shared<Configuration>(robot, N));
  vars_.push_back(std::make_shared<Velocity>(robot, N));
}


void OCP::createConstraints(const Robot& robot, const int N, const double dtau) {
  constraints_.clear();
  initial_state_constraints_ = std::make_shared<InitialState>(robot);
  constraints_.push_back(initial_state_constraints_);
  for (int i=0; i<N; ++i) {
    constraints_.push_back(std::make_shared<StateEquation>(robot, dtau, i));
  }
}


void OCP::createCosts(const Robot& robot, const int N, const double dtau) {
  costs_.clear();
  for (int i=0; i<N; ++i) {
    costs_.push_back(std::make_shared<StageCost>(robot, dtau, i));
  }
  costs_.push_back(std::make_shared<TerminalCost>(robot, N));
}

} // namespace pinipopt