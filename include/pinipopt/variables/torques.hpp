#ifndef PINIPOPT_TORQUES_HPP_
#define PINIPOPT_TORQUES_HPP_

#include <string>
#include <cassert>

#include "Eigen/Core"
#include "ifopt/composite.h"
#include "ifopt/variable_set.h"

#include "pinipopt/robot/robot.hpp"


namespace pinipopt {

class Torques : public ifopt::VariableSet {
public:
  Torques(const Robot& robot, const int time_stage);

  virtual ~Torques() = default;

  void SetVariables(const Eigen::VectorXd& u) override;

  Eigen::VectorXd GetValues() const override;

  ifopt::Composite::VecBound GetBounds() const override;

private:
  Eigen::VectorXd u_;
};
  
} // namespace pinipopt

#endif // PINIPOPT_TORQUES_HPP_ 