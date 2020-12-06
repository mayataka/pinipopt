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
  const Eigen::VectorXd q_weight = Eigen::VectorXd::Constant(robot.dimq(), 1);
  const Eigen::VectorXd v_weight = Eigen::VectorXd::Constant(robot.dimv(), 0.1);
  const Eigen::VectorXd u_weight = Eigen::VectorXd::Constant(robot.dimu(), 0.01);

  pinipopt::OCP ocp(robot, T, N);

  ocp.set_q_weight(q_weight);
  ocp.set_v_weight(v_weight);
  ocp.set_u_weight(u_weight);
  ocp.setIpoptOption("linear_solver", "ma27");

  const Eigen::VectorXd q = Eigen::VectorXd::Random(robot.dimq());
  const Eigen::VectorXd v = Eigen::VectorXd::Random(robot.dimv());
  ocp.solve(q, v);

  return 0;
}