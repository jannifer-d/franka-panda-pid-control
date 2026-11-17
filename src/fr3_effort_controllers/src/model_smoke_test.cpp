#include "fr3_effort_controllers/fr3_model.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: model_smoke_test <path_to_fr3.urdf>\n";
    return 1;
  }

  try {
    fr3_effort_controllers::FR3Model robot(argv[1]);

    // A valid test configuration within the FR3 joint limits.
    fr3_effort_controllers::FR3Model::JointVector q;
    q << 0.0, -0.5, 0.0, -1.8, 0.0, 1.5, 0.0;

    const auto gravity = robot.gravity(q);
    const auto pose = robot.forwardKinematics(q);
    const auto J = robot.jacobian(q);

    if (!gravity.allFinite() ||
        !pose.matrix().allFinite() ||
        !J.allFinite()) {
      throw std::runtime_error("A model calculation returned non-finite values");
    }

    std::cout << "\nFR3 MODEL SMOKE TEST PASSED\n";

    std::cout << "\nGravity compensation torque [Nm]:\n"
              << gravity.transpose() << "\n";

    std::cout << "\nEnd-effector position [m]:\n"
              << pose.translation().transpose() << "\n";

    std::cout << "\nEnd-effector rotation:\n"
              << pose.rotation() << "\n";

    std::cout << "\nEnd-effector Jacobian (6x7):\n"
              << J << "\n";

    std::cout << "\nJacobian dimensions: "
              << J.rows() << " x " << J.cols() << "\n";

    return 0;
  } catch (const std::exception & e) {
    std::cerr << "FR3 MODEL SMOKE TEST FAILED: "
              << e.what() << "\n";
    return 1;
  }
}
