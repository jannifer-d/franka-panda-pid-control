#pragma once

#include <string>

#include <Eigen/Dense>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace fr3_effort_controllers
{

class FR3Model
{
public:
  static constexpr int kDof = 7;
  static constexpr const char * kEndEffectorFrame = "fr3_link8";

  using JointVector = Eigen::Matrix<double, kDof, 1>;
  using Jacobian = Eigen::Matrix<double, 6, kDof>;

  explicit FR3Model(const std::string & urdf_path);

  JointVector gravity(const JointVector & q);

  Eigen::Isometry3d forwardKinematics(const JointVector & q);

  Jacobian jacobian(const JointVector & q);

  const pinocchio::Model & model() const
  {
    return model_;
  }

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex end_effector_frame_id_;
};

}  // namespace fr3_effort_controllers
