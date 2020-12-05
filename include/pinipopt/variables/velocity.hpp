#ifndef PINIPOPT_VELOCITY_HPP_
#define PINIPOPT_VELOCITY_HPP_

#include <string>
#include <cassert>

#include "Eigen/Core"
#include "ifopt/composite.h"
#include "ifopt/variable_set.h"

#include "pinipopt/robot/robot.hpp"


namespace pinipopt {

class Velocity : public ifopt::VariableSet {
public:
  Velocity(const Robot& robot, const int time_stage);

  virtual ~Velocity() = default;

  void SetVariables(const Eigen::VectorXd& v) override;

  Eigen::VectorXd GetValues() const override;

  ifopt::Composite::VecBound GetBounds() const override;

private:
  Eigen::VectorXd v_;
};
  
} // namespace pinipopt

#endif // PINIPOPT_VELOCITY_HPP_ 