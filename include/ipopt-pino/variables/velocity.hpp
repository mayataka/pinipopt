#ifndef IPOPT_PINO_VELOCITY_HPP_
#define IPOPT_PINO_VELOCITY_HPP_

#include <string>
#include <cassert>

#include "Eigen/Core"
#include "ifopt/composite.h"
#include "ifopt/variable_set.h"

#include "ipopt-pino/robot/robot.hpp"


namespace ipoptpino {

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
  
} // namespace ipoptpino

#endif // IPOPT_PINO_VELOCITY_HPP_ 