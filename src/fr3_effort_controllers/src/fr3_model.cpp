#include "fr3_effort_controllers/fr3_model.hpp"

#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace fr3_effort_controllers
{

FR3Model::FR3Model(const std::string & urdf_path)
: model_(),
  data_(model_),
  end_effector_frame_id_(0)
{
  pinocchio::urdf::buildModel(urdf_path, model_);

  if (model_.nq != kDof || model_.nv != kDof) {
    throw std::runtime_error(
      "FR3 model must have exactly 7 configuration and velocity variables");
  }

  data_ = pinocchio::Data(model_);

  if (!model_.existFrame(kEndEffectorFrame)) {
    throw std::runtime_error(
      "End-effector frame not found: fr3_link8");
  }

  end_effector_frame_id_ = model_.getFrameId(kEndEffectorFrame);
}

FR3Model::JointVector FR3Model::gravity(const JointVector & q)
{
  pinocchio::computeGeneralizedGravity(model_, data_, q);
  return data_.g;
}

Eigen::Isometry3d FR3Model::forwardKinematics(const JointVector & q)
{
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  const auto & placement =
    data_.oMf[end_effector_frame_id_];

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = placement.rotation();
  transform.translation() = placement.translation();

  return transform;
}

FR3Model::Jacobian FR3Model::jacobian(const JointVector & q)
{
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);

  pinocchio::getFrameJacobian(
    model_,
    data_,
    end_effector_frame_id_,
    pinocchio::LOCAL_WORLD_ALIGNED,
    J);

  return J;
}

}  // namespace fr3_effort_controllers
