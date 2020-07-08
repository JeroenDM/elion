#include "elion_planner/constraint.h"

#include <Eigen/Geometry>
#include <ompl/base/Constraint.h>
#include <eigen_conversions/eigen_msg.h>
#include <cmath>

#include <ros/console.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>

#include "elion_planner/conversions.h"
#include "elion_planner/msg_parsing.h"

namespace elion
{
/******************************************
 * Base constraint
 * ****************************************/

BaseConstraint::BaseConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                               const unsigned int num_dofs, const unsigned int num_cons_)
  : robot_model_(robot_model), ob::Constraint(num_dofs, num_cons_)
{
  // Setup Moveit's robot model for kinematic calculations
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  joint_model_group_ = robot_state_->getJointModelGroup(group);

  ROS_INFO_STREAM("Creating constraints of shape (" << num_dofs << ", 3)");

  // use end-effector link by default TODO make this input
  link_name_ = joint_model_group_->getLinkModelNames().back();

  ROS_INFO_STREAM("Created OMPL constraints for link: " << link_name_);
}

void BaseConstraint::init(moveit_msgs::Constraints constraints)
{
  parseConstraintMsg(constraints);
}

Eigen::Isometry3d BaseConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state_->getGlobalLinkTransform(link_name_);
}

Eigen::MatrixXd BaseConstraint::geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state_->getJacobian(joint_model_group_);
}

void BaseConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  auto current_values = calcError(x);
  for (std::size_t i{ 0 }; i < bounds_.size(); ++i)
  {
    out[i] = bounds_[i].distance(current_values[i]);
  }
}

void BaseConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
{
  auto current_values = calcError(x);
  auto current_jacobian = calcErrorJacobian(x);

  for (std::size_t i{ 0 }; i < bounds_.size(); ++i)
  {
    if (current_values[i] > bounds_[i].upper + getTolerance())
    {
      out.row(i) = current_jacobian.row(i);
    }
    else if (current_values[i] < bounds_[i].lower - getTolerance())
    {
      out.row(i) = -current_jacobian.row(i);
    }
    else
    {
      out.row(i) = Eigen::VectorXd::Zero(n_);
    }
  }
}

/******************************************
 * Position constraints specific
 * ****************************************/

void PositionConstraint::parseConstraintMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  bounds_ = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));
  ROS_INFO_STREAM("Parsed x constraints" << bounds_[0]);
  ROS_INFO_STREAM("Parsed y constraints" << bounds_[1]);
  ROS_INFO_STREAM("Parsed z constraints" << bounds_[2]);

  // extract target / nominal value
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_ << position.x, position.y, position.z;

  // target_pose_ should replace target
  Eigen::Quaterniond temp_quat;
  tf::quaternionMsgToEigen(constraints.position_constraints[0].constraint_region.primitive_poses[0].orientation,
                           temp_quat);
  target_pose_ = Eigen::Translation3d(position.x, position.y, position.z) * temp_quat;
}

Eigen::VectorXd PositionConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_pose_.rotation().transpose() * (forwardKinematics(x).translation() - target_pose_.translation());
}

Eigen::MatrixXd PositionConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_pose_.rotation().transpose() * geometricJacobian(x).topRows(3);
}

/******************************************
 * Angle-axis error constraints
 * ****************************************/
void AngleAxisConstraint::parseConstraintMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  bounds_ = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));
  ROS_INFO_STREAM("Parsing angle-axis constraints");
  ROS_INFO_STREAM("Parsed rx / roll constraints" << bounds_[0]);
  ROS_INFO_STREAM("Parsed ry / pitch constraints" << bounds_[1]);
  ROS_INFO_STREAM("Parsed rz / yaw constraints" << bounds_[2]);

  // extract target / nominal value
  // for orientation we can save the target in different formats, probably quaternion is the best one here
  // we could use a 3 vector to be uniform with position constraints, but this makes us vulnerable to
  // singularities, wich could occur here as the target can be an arbitrary orientation
  tf::quaternionMsgToEigen(constraints.orientation_constraints.at(0).orientation, target_as_quat_);

  // so we could do this:
  // target_ = target_as_quat_.toRotationMatrix().eulerAngles(0, 1, 2);
  // but calcCurrentValues and calcCurrentJacobian use target_as_quat_
}

Eigen::VectorXd AngleAxisConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // I'm not sure yet whether I want the error expressed in the current ee_frame, or target_frame,
  // or world frame. This implementation expressed the error in the end-effector frame.
  Eigen::Matrix3d Rerror = forwardKinematics(x).rotation().transpose() * target_as_quat_;
  Eigen::AngleAxisd aa(Rerror);
  double angle = aa.angle();
  assert(std::abs(angle) < M_PI);
  return aa.axis() * angle;
}

// mysterious minus sign in Jacobian, found through trial and error...
Eigen::MatrixXd AngleAxisConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // Eigen::AngleAxisd aa {forwardKinematics(x).rotation().transpose() * target_as_quat_};

  Eigen::AngleAxisd aa{ forwardKinematics(x).rotation() };
  return -angularVelocityToAngleAxis(aa.angle(), aa.axis()) * geometricJacobian(x).bottomRows(3);
}

/******************************************
 * Roll, pitch, yaw orientation constraints
 * ****************************************/
void RPYConstraints::parseConstraintMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  bounds_ = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));
  ROS_INFO_STREAM("Parsed rx / roll constraints" << bounds_[0]);
  ROS_INFO_STREAM("Parsed ry / pitch constraints" << bounds_[1]);
  ROS_INFO_STREAM("Parsed rz / yaw constraints" << bounds_[2]);

  // extract target / nominal value
  // for orientation we can save the target in different formats, probably quaternion is the best one here
  // we could use a 3 vector to be uniform with position constraints, but this makes us vulnerable to
  // singularities, wich could occur here as the target can be an arbitrary orientation
  tf::quaternionMsgToEigen(constraints.orientation_constraints.at(0).orientation, target_as_quat_);

  // so we could do this:
  target_ = target_as_quat_.toRotationMatrix().eulerAngles(0, 1, 2);
  // but calcCurrentValues and calcCurrentJacobian use target_as_quat_
}

Eigen::VectorXd RPYConstraints::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // I'm not sure yet whether I want the error expressed in the current ee_frame, or target_frame,
  // or world frame. This implementation expressed the error in the end-effector frame.
  Eigen::Matrix3d error = forwardKinematics(x).rotation().transpose() * target_as_quat_;
  return error.eulerAngles(0, 1, 2);

  // alternative, calculate difference of euler angles and try to live with singularities
  // Eigen::Vector3d error = forwardKinematics(x).rotation().eulerAngles(0, 1, 2) - target_;
  // return error;
}

Eigen::MatrixXd RPYConstraints::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // use the euler angles of the current end-effector orientation expressed in the world frame
  // we need this to convert the geometric jacobian to an analytical one that can be used for rpy angles
  // the jacobian is expressed in the world frame, so should the rpy angles I suppose...
  auto rpy = forwardKinematics(x).rotation().eulerAngles(0, 1, 2);
  return angularVelocityToRPYRates(rpy[0], rpy[1]) * geometricJacobian(x).bottomRows(3);
}

/******************************************
 * Position and orienation constraints
 * Using angle axis error for orientation
 * ****************************************/
void PoseConstraints::parseConstraintMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  bounds_ = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));
  ROS_INFO_STREAM("Parsed x constraints" << bounds_[0]);
  ROS_INFO_STREAM("Parsed y constraints" << bounds_[1]);
  ROS_INFO_STREAM("Parsed z constraints" << bounds_[2]);

  // extract target / nominal value
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_ << position.x, position.y, position.z;

  auto ori_bounds = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));
  bounds_.push_back(ori_bounds[0]);
  bounds_.push_back(ori_bounds[1]);
  // IGNORE RZ CONSTRAINT
  tf::quaternionMsgToEigen(constraints.orientation_constraints.at(0).orientation, target_as_quat_);
  ROS_INFO_STREAM("Parsed rx / roll constraints" << bounds_[3]);
  ROS_INFO_STREAM("Parsed ry / pitch constraints" << bounds_[4]);
  // ROS_INFO_STREAM("Parsed rz / yaw constraints" << bounds_[5]);
}

Eigen::VectorXd PoseConstraints::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  Eigen::Matrix<double, 5, 1> error;
  auto fk_pose = forwardKinematics(x);
  error.topRows(3) = fk_pose.translation() - target_;

  Eigen::Matrix3d Rerror = fk_pose.rotation().transpose() * target_as_quat_;
  Eigen::AngleAxisd aa(Rerror);
  double angle = aa.angle();
  assert(std::abs(angle) < M_PI);
  // error.bottomRows(3) = aa.axis() * angle;
  error[3] = aa.axis()[0] * angle;
  error[4] = aa.axis()[1] * angle;

  return error;
}

/******************************************
 * Equality constraints on X and Z position
 * ****************************************/

void XZPositionConstraint::parseConstraintMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  ROS_INFO_STREAM("Creating equality constraints, no bounds will be parsed.");

  // extract target / nominal value
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_ << position.x, position.y, position.z;
}

void XZPositionConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  out[0] = forwardKinematics(x).translation().x() - target_.x();
  out[1] = forwardKinematics(x).translation().z() - target_.z();
}

void XZPositionConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
{
  out.row(0) = geometricJacobian(x).row(0);
  out.row(1) = geometricJacobian(x).row(2);
}

/*************************************************************************
 * Equality constraints on X and Y orientation (angle-axis representation)
 * ***********************************************************************/

void XYOrientationConstraint::parseConstraintMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  ROS_INFO_STREAM("Creating equality constraints on orientation, no bounds will be parsed.");

  // extract target / nominal value
  tf::quaternionMsgToEigen(constraints.orientation_constraints.at(0).orientation, target_as_quat_);
}

void XYOrientationConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x,
                                       Eigen::Ref<Eigen::VectorXd> out) const
{
  Eigen::AngleAxisd aa(forwardKinematics(x).rotation().transpose() * target_as_quat_);
  out[0] = (aa.axis() * aa.angle()).x();
  out[1] = (aa.axis() * aa.angle()).y();
}

void XYOrientationConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x,
                                       Eigen::Ref<Eigen::MatrixXd> out) const
{
  Eigen::AngleAxisd aa{ forwardKinematics(x).rotation() };
  auto J = geometricJacobian(x);
  out.row(0) = (-angularVelocityToAngleAxis(aa.angle(), aa.axis()) * J.bottomRows(3)).row(0);
  out.row(1) = (-angularVelocityToAngleAxis(aa.angle(), aa.axis()) * J.bottomRows(3)).row(1);
}

// void XYOrientationConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x,
//                                        Eigen::Ref<Eigen::MatrixXd> out) const
// {
//   ob::Constraint::jacobian(x, out);
// }

/******************************************
 * Factory
 * ****************************************/

std::shared_ptr<BaseConstraint> createConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                                                 moveit_msgs::Constraints constraints)
{
  std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();
  std::size_t num_pos_con = constraints.position_constraints.size();
  std::size_t num_ori_con = constraints.orientation_constraints.size();

  if (num_pos_con > 0 && num_ori_con > 0)
  {
    ROS_ERROR_STREAM("Combining position and orientation constraints results in ignoring the z orientation tolerance "
                     "value.");
    auto pose_con = std::make_shared<PoseConstraints>(robot_model, group, num_dofs);
    pose_con->init(constraints);
    return pose_con;
  }
  else if (num_pos_con > 0)
  {
    if (num_pos_con > 1)
    {
      ROS_ERROR_STREAM("Only a single position constraints supported. Using the first one.");
    }
    // auto pos_con = std::make_shared<PositionConstraint>(robot_model, group, num_dofs);
    auto pos_con = std::make_shared<XZPositionConstraint>(robot_model, group, num_dofs);
    pos_con->init(constraints);
    return pos_con;
  }
  else if (num_ori_con > 0)
  {
    if (num_ori_con > 1)
    {
      ROS_ERROR_STREAM("Only a single orientation constraints supported. Using the first one.");
    }

    if (constraints.name == OrientationErrorType::ANGLE_AXIS)
    {
      ROS_INFO_STREAM("Creating orientation constraints of type: " << constraints.name);
      // auto ori_con = std::make_shared<AngleAxisConstraint>(robot_model, group, num_dofs);
      auto ori_con = std::make_shared<XYOrientationConstraint>(robot_model, group, num_dofs);
      ori_con->init(constraints);
      return ori_con;
    }
    else if (constraints.name == OrientationErrorType::ROLL_PITCH_YAW)
    {
      ROS_INFO_STREAM("Creating orientation constraints of type: " << constraints.name);
      auto ori_con = std::make_shared<RPYConstraints>(robot_model, group, num_dofs);
      ori_con->init(constraints);
      return ori_con;
    }
    else
    {
      ROS_ERROR_STREAM("Unkown type of orientation constraint: " << constraints.name);
      return nullptr;
    }
    // auto ori_con = std::make_shared<RPYConstraints>(robot_model, group, constraints, num_dofs);
    // auto ori_con = std::make_shared<QuaternionConstraint>(robot_model, group, constraints, num_dofs);
    // ori_con->init(constraints);
  }
  else
  {
    ROS_ERROR_STREAM("No constraints found in planning request.");
    return nullptr;
  }
}

}  // namespace elion