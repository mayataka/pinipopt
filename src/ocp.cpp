#include "pinipopt/ocp.hpp"


namespace pinipopt {

OCP::OCP(const Robot& robot, const double T, const int N) 
  : robot_(robot),
    T_(T), 
    dtau_(T/N),
    N_(N) {
  createOCP(robot, N, dtau_);
  ipopt_.SetOption("hessian_approximation", "limited-memory");
  ipopt_.SetOption("limited_memory_update_type", "bfgs");
}


void OCP::solve(const Eigen::VectorXd& q0, const Eigen::VectorXd& v0) {
  initial_state_constraint_->setInitialState(q0, v0);
  ipopt_.Solve(ocp_);
}


void OCP::set_q(const Eigen::VectorXd& q) {
  for (auto& e : q_) {
    e->SetVariables(q);
  }
}


void OCP::set_v(const Eigen::VectorXd& v) {
  for (auto& e : v_) {
    e->SetVariables(v);
  }
}


void OCP::set_u(const Eigen::VectorXd& u) {
  for (auto& e : u_) {
    e->SetVariables(u);
  }
}


void OCP::set_q_weight(const Eigen::VectorXd& q_weight) {
  for (auto& e : stage_costs_) {
    e->set_q_weight(q_weight);
  }
  terminal_cost_->set_q_weight(q_weight);
}


void OCP::set_v_weight(const Eigen::VectorXd& v_weight) {
  for (auto& e : stage_costs_) {
    e->set_v_weight(v_weight);
  }
  terminal_cost_->set_v_weight(v_weight);
}


void OCP::set_u_weight(const Eigen::VectorXd& u_weight) {
  for (auto& e : stage_costs_) {
    e->set_u_weight(u_weight);
  }
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
  q_.clear();
  v_.clear();
  u_.clear();
  vars_.clear();
  for (int i=0; i<N; ++i) {
    q_.push_back(std::make_shared<Configuration>(robot, i));
    v_.push_back(std::make_shared<Velocity>(robot, i));
    u_.push_back(std::make_shared<Torques>(robot, i));
    vars_.push_back(q_[i]);
    vars_.push_back(v_[i]);
    vars_.push_back(u_[i]);
  }
  q_.push_back(std::make_shared<Configuration>(robot, N));
  v_.push_back(std::make_shared<Velocity>(robot, N));
  vars_.push_back(q_[N]);
  vars_.push_back(v_[N]);
}


void OCP::createConstraints(const Robot& robot, const int N, const double dtau) {
  constraints_.clear();
  initial_state_constraint_ = std::make_shared<InitialState>(robot);
  constraints_.push_back(initial_state_constraint_);
  for (int i=0; i<N; ++i) {
    constraints_.push_back(std::make_shared<StateEquation>(robot, dtau, i));
  }
}


void OCP::createCosts(const Robot& robot, const int N, const double dtau) {
  stage_costs_.clear();
  for (int i=0; i<N; ++i) {
    stage_costs_.push_back(std::make_shared<StageCost>(robot, dtau, i));
  }
  terminal_cost_ = std::make_shared<TerminalCost>(robot, N);
  costs_.clear();
  for (auto e : stage_costs_) {
    costs_.push_back(e);
  }
  costs_.push_back(terminal_cost_);
}

} // namespace pinipopt