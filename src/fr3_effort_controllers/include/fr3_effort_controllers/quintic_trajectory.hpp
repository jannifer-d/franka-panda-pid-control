#pragma once

#include <stdexcept>

#include <Eigen/Dense>

namespace fr3_effort_controllers
{

template<int N>
class QuinticTrajectory
{
public:
  using Vector = Eigen::Matrix<double, N, 1>;

  struct Sample
  {
    Vector position;
    Vector velocity;
    Vector acceleration;
  };

  void reset(
    const Vector & start,
    const Vector & goal,
    double duration)
  {
    if (duration <= 0.0) {
      throw std::invalid_argument(
        "Trajectory duration must be positive");
    }

    start_ = start;
    goal_ = goal;
    duration_ = duration;
  }

  Sample sample(double time) const
  {
    if (duration_ <= 0.0) {
      throw std::runtime_error(
        "Trajectory has not been initialized");
    }

    Sample result;

    if (time <= 0.0) {
      result.position = start_;
      result.velocity.setZero();
      result.acceleration.setZero();
      return result;
    }

    if (time >= duration_) {
      result.position = goal_;
      result.velocity.setZero();
      result.acceleration.setZero();
      return result;
    }

    const double u = time / duration_;
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;

    const double s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;

    const double ds =
      30.0 * u2 - 60.0 * u3 + 30.0 * u4;

    const double dds =
      60.0 * u - 180.0 * u2 + 120.0 * u3;

    const Vector delta = goal_ - start_;

    result.position = start_ + s * delta;
    result.velocity = (ds / duration_) * delta;
    result.acceleration =
      (dds / (duration_ * duration_)) * delta;

    return result;
  }

private:
  Vector start_ = Vector::Zero();
  Vector goal_ = Vector::Zero();
  double duration_ = 0.0;
};

}  // namespace fr3_effort_controllers
