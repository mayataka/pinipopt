#ifndef PINIPOPT_CONFIGURATION_HPP_
#define PINIPOPT_CONFIGURATION_HPP_

#include <string>
#include <cassert>

#include "Eigen/Core"
#include "ifopt/composite.h"
#include "ifopt/variable_set.h"

#include "pinipopt/robot/robot.hpp"


namespace pinipopt {

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
  
} // namespace pinipopt

#endif // PINIPOPT_CONFIGURATION_HPP_