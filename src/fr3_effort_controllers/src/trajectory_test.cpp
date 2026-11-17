#include "fr3_effort_controllers/quintic_trajectory.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

int main()
{
  using Trajectory =
    fr3_effort_controllers::QuinticTrajectory<7>;

  Trajectory trajectory;

  Trajectory::Vector start;
  Trajectory::Vector goal;

  start.setZero();
  goal << 0.2, -0.3, 0.1, -1.5, 0.2, 1.8, -0.1;

  constexpr double duration = 4.0;

  trajectory.reset(start, goal, duration);

  const auto initial = trajectory.sample(0.0);
  const auto middle = trajectory.sample(duration / 2.0);
  const auto final = trajectory.sample(duration);

  constexpr double tolerance = 1e-10;

  if ((initial.position - start).norm() > tolerance ||
      initial.velocity.norm() > tolerance ||
      initial.acceleration.norm() > tolerance) {
    throw std::runtime_error("Initial boundary condition failed");
  }

  if ((final.position - goal).norm() > tolerance ||
      final.velocity.norm() > tolerance ||
      final.acceleration.norm() > tolerance) {
    throw std::runtime_error("Final boundary condition failed");
  }

  const auto expected_middle = 0.5 * (start + goal);

  if ((middle.position - expected_middle).norm() > tolerance) {
    throw std::runtime_error("Midpoint position check failed");
  }

  std::cout << "QUINTIC TRAJECTORY TEST PASSED\n";

  std::cout << "\nStart position:\n"
            << initial.position.transpose() << "\n";

  std::cout << "\nMidpoint position:\n"
            << middle.position.transpose() << "\n";

  std::cout << "\nMidpoint velocity:\n"
            << middle.velocity.transpose() << "\n";

  std::cout << "\nMidpoint acceleration:\n"
            << middle.acceleration.transpose() << "\n";

  std::cout << "\nFinal position:\n"
            << final.position.transpose() << "\n";

  return 0;
}
