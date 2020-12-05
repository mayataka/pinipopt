#ifndef IPOPT_PINO_CONFIGURATION_HPP_
#define IPOPT_PINO_CONFIGURATION_HPP_

#include <string>
#include <cassert>

#include "Eigen/Core"
#include "ifopt/composite.h"
#include "ifopt/variable_set.h"

#include "ipopt-pino/robot/robot.hpp"


namespace ipoptpino {

class Configuration : public ifopt::VariableSet {
public:
  Configuration(const Robot& robot, const int time_stage);

  virtual ~Configuration() = default;

  void SetVariables(const Eigen::VectorXd& q) override;

  Eigen::VectorXd GetValues() const override;

  ifopt::Composite::VecBound GetBounds() const override;

private:
  Eigen::VectorXd q_;
};
  
} // namespace ipoptpino

#endif // IPOPT_PINO_CONFIGURATION_HPP_