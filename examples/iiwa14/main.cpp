#include <iostream>
#include <string>
#include <memory>
#include <chrono>

#include "Eigen/Core"

#include "pinipopt/robot/robot.hpp"
#include "pinipopt/ocp.hpp"


int main() {
  const std::string urdf_file_name = "../urdf/iiwa14.urdf";
  pinipopt::Robot robot(urdf_file_name);

  const double T = 1;
  const int N = 20;
  const Eigen::VectorXd q = Eigen::VectorXd::Random(robot.dimq());
  const Eigen::VectorXd v = Eigen::VectorXd::Random(robot.dimv());

  pinipopt::OCP ocp(robot, T, N);
  ocp.setIpoptOption("linear_solver", "ma27");
  ocp.solve(q, v);

  return 0;
}