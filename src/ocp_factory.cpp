#include "ipopt-pino/ocp_factory.hpp"


namespace ipoptpino {

OCPFactory::OCPFactory(const Robot& robot, const double T, const int N) 
  : robot_(robot),
    T_(T), 
    dtau_(T/N),
    N_(N) {
}


OCPFactory::VariablePtrVec OCPFactory::GetVariableSets() const {
  VariablePtrVec ptr_vec;
  for (int i=0; i<N_; ++i) {
    ptr_vec.push_back(std::make_shared<Configuration>(robot_, i));
    ptr_vec.push_back(std::make_shared<Velocity>(robot_, i));
    ptr_vec.push_back(std::make_shared<Torques>(robot_, i));
  }
  ptr_vec.push_back(std::make_shared<Configuration>(robot_, N_));
  ptr_vec.push_back(std::make_shared<Velocity>(robot_, N_));
  return ptr_vec;
}


OCPFactory::ContraintPtrVec OCPFactory::GetConstraints() const {
  ContraintPtrVec ptr_vec;
  ptr_vec.push_back(std::make_shared<InitialState>(robot_));
  for (int i=0; i<N_; ++i) {
    ptr_vec.push_back(std::make_shared<StateEquation>(robot_, dtau_, i));
  }
  return ptr_vec;
}


OCPFactory::ContraintPtrVec OCPFactory::GetConstraints(
    const Eigen::VectorXd& q0, const Eigen::VectorXd& v0) const {
  ContraintPtrVec ptr_vec;
  ptr_vec.push_back(std::make_shared<InitialState>(robot_, q0, v0));
  for (int i=0; i<N_; ++i) {
    ptr_vec.push_back(std::make_shared<StateEquation>(robot_, dtau_, i));
  }
  return ptr_vec;
}


OCPFactory::CostPtrVec OCPFactory::GetCosts() const {
  CostPtrVec ptr_vec;
  for (int i=0; i<N_; ++i) {
    ptr_vec.push_back(std::make_shared<StageCost>(robot_, dtau_, i));
  }
  ptr_vec.push_back(std::make_shared<TerminalCost>(robot_, N_));
  return ptr_vec;
}

} // namespace ipoptpino