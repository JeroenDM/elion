#pragma once

#include <Eigen/Geometry>

#include <ompl/base/Constraint.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/Constraints.h>

#include "elion_planner/bounds.h"

namespace elion
{
namespace ob = ompl::base;

// enum class OrientationErrorType
// {
//   ROLL_PITCH_YAW,
//   ANGLE_AXIS,
//   QUATERNION
// };

/** \brief Different calculation methods for orientation error.
 *
 * As I can only pass strings through MoveIt planning requests
 * it does not really work with enums of integral type.
 * Therefore I came up with this akward hack,
 * which I think is still better than magic with implicit casting.
 * This hack should result in almost the same syntax.
 *
 * Is there name collision danger with QUATERNION?
 * */
namespace OrientationErrorType
{
const std::string ROLL_PITCH_YAW{ "RollPitchYaw" };
const std::string ANGLE_AXIS{ "AngleAxis" };
const std::string QUATERION{ "Quaterion" };
}  // namespace OrientationErrorType

/** \brief Abstract base class for differen types of constraints
 *
 * They all have bounds on some kind of error function,
 * this error function is the thing requirering specialization.
 * In addition parsing of the constraint message from MoveIt is different.
 *
 * Other constraints have to override:
 * - parseConstraintMsg
 * - calcError
 * - calcErrorJacobian
 *
 * @@todo clean-up what is public / protected / private,
 * I was being lax for debugging.
 * */
class BaseConstraint : public ob::Constraint
{
public:
  BaseConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs,
                 const unsigned int num_cons_ = 3);

  /** \brief initialize constraint based on message content.
   *
   * This is needed, because we cannot call the pure virtual
   * parseConstraintsMsg method from the constructor of this class.
   * */
  void init(moveit_msgs::Constraints constraints);

  // some kind of factory method, but I can't get it to work yet
  // static std::shared_ptr<PositionConstraint> create(robot_model::RobotModelConstPtr robot_model,
  //                                                   const std::string& group, moveit_msgs::Constraints constraints,
  //                                                   const unsigned int num_dofs);

  /** OMPL's main constraint evaluation function.
   *
   *  OMPL requires you to override at least "function" which represents the constraint F(q) = 0
   * */
  virtual void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;

  /** Optionally you can also provide dF(q)/dq, the Jacobian of  the constriants.
   *
   * */
  virtual void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;

  // generic helper functions for robot kinematics.
  // @todo Are these actually const, as the robot state is modified? How come it works?
  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;
  Eigen::MatrixXd geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** Parse bounds on position and orientation parameters from MoveIt's constraint message.
   *
   * This is non-trivial given the often complex structure of these messages.
   * */
  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) = 0;

  /** Calculate the value of the parameter that is being constraint by the bounds.
   *
   * In this Position constraints case, it calculates the x, y and z position
   * of the end-effector.
   * */
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
  {
    ROS_ERROR_STREAM("Constraint method calcError was not overridded, so it should not be used.");
  }

  /** Calculate the Jacobian for the current parameters that are being constraints.
   *
   * @todo, maybe also provide output agruments similar to the jacobian function
   * so we can default to ob::Constraint::jacobian(x, out) when needed.
   *
   * This is the Jacobian without the correction due to the penalty function
   * (adding a minus sign or setting it to zero.)
   * */
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
  {
    ROS_ERROR_STREAM("Constraint method calcErrorJacobian was not overridded, so it should not be used.");
  }

protected:
  // MoveIt's robot representation for kinematic calculations
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  std::string link_name_;      /** Robot link the constraints are applied to. */
  std::vector<Bounds> bounds_; /** Upper and lower bounds on constrained variables. */
  Eigen::Vector3d target_;     /** target for equality constraints, nominal value for inequality constraints. */

  // New target is a full 3D pose, instead of only position or orientation
  // to allow for more general constraints.
  // Bounds are expressed on a deviation from the location of this frame,
  // expressed IN this local frame.
  Eigen::Isometry3d target_pose_;
};

class PositionConstraint : public BaseConstraint
{
public:
  PositionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }
  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) override;
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
};

/** \brief orientation constraints based on angle-axis error.
 *
 * (aka exponential coordinates)
 * This class overrides the jacobian method directly,
 * in order to use the default finite difference jacobian from OMPL.
 *
 * */
class AngleAxisConstraint : public BaseConstraint
{
public:
  AngleAxisConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                      const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }

  // void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override
  // {
  //   ob::Constraint::jacobian(x, out);
  // }

  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) override;
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;

private:
  Eigen::Quaterniond target_as_quat_;
};

/** \brief Orientation constraints based on roll, pitch, yaw error
 *
 * Constraints on roll, pitch, and yaw angle of the end-effector:
 * R = Rx(roll) * Ry(pitch) * Rz(yaw)
 *
 */
class RPYConstraints : public BaseConstraint
{
public:
  RPYConstraints(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }

  // void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override
  // {
  //   ob::Constraint::jacobian(x, out);
  // }

  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) override;
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;

private:
  Eigen::Quaterniond target_as_quat_;
};

/** \brief Position and orientation constraint
 *
 * Hardcoded implementation instead of using the class
 * ompl::base::ConstraintsIntersection
 *
 * */
class PoseConstraints : public BaseConstraint
{
public:
  PoseConstraints(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs, 5)
  {
  }

  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override
  {
    ob::Constraint::jacobian(x, out);
  }

  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) override;
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  // virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;

private:
  Eigen::Quaterniond target_as_quat_;
};

/** \brief hardcoded equality constraints on x and z position.
 *
 * To try out Atlas and Tangent Bundle constraint state space.
 * */
class XZPositionConstraint : public BaseConstraint
{
public:
  XZPositionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                       const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs, 2)
  {
  }
  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) override;
  virtual void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;
  virtual void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;
};

/** \brief hardcoded equality constraints on x and y orientation (angle-axis representation).
 *
 * To try out Atlas and Tangent Bundle constraint state space.
 * */
class XYOrientationConstraint : public BaseConstraint
{
public:
  XYOrientationConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                          const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs, 2)
  {
  }
  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) override;
  virtual void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;
  virtual void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;

private:
  Eigen::Quaterniond target_as_quat_;
};

/** \brief Factory to create constraints based on what is in the MoveIt constraint message. **/
std::shared_ptr<BaseConstraint> createConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                                                 moveit_msgs::Constraints constraints);

}  // namespace elion
