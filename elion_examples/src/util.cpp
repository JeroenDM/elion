#include "elion_examples/util.h"

#include <boost/scoped_ptr.hpp>
#include <jsoncpp/json/json.h>
#include <pluginlib/class_loader.h>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/Constraints.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <tf2_eigen/tf2_eigen.h>

namespace elion
{
void loadPlanningPlugin(ClassLoaderSPtr& planner_plugin_loader, planning_interface::PlannerManagerPtr& planner_instance,
                        robot_model::RobotModelPtr& robot_model, ros::NodeHandle& node_handle,
                        const std::string& base_class, const std::string& plugin_name)
{
  try
  {
    planner_plugin_loader.reset(
        new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", base_class));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
    {
      ss << classes[i] << " ";
    }
    ROS_ERROR_STREAM("Exception while loading planner '" << plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
}

moveit_msgs::PositionConstraint createPositionConstraint(const std::string& base_link, const std::string& ee_link_name,
                                                         std::vector<double>& dimensions,
                                                         std::vector<double>& nominal_position)
{
  moveit_msgs::PositionConstraint position_constraint;
  if (dimensions.size() != 3)
  {
    ROS_ERROR_STREAM("Position constraint dimensions should have length 3, not " << dimensions.size());
    return position_constraint;
  }

  if (nominal_position.size() != 3)
  {
    ROS_ERROR_STREAM("Position constraint nominal_position should have length 3, not " << nominal_position.size());
    return position_constraint;
  }

  shape_msgs::SolidPrimitive box_constraint;
  box_constraint.type = shape_msgs::SolidPrimitive::BOX;
  box_constraint.dimensions = dimensions; /* use -1 to indicate no constraints. */

  geometry_msgs::Pose box_pose;
  box_pose.position.x = nominal_position[0];
  box_pose.position.y = nominal_position[1];
  box_pose.position.z = nominal_position[2];
  box_pose.orientation.w = 1.0; /* Orientation of position constraints not implemented yet. */

  position_constraint.header.frame_id = base_link;
  position_constraint.link_name = ee_link_name;
  position_constraint.constraint_region.primitives.push_back(box_constraint);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);

  return position_constraint;
}

moveit_msgs::OrientationConstraint createOrientationConstraint(const std::string& base_link,
                                                               const std::string& ee_link_name,
                                                               std::vector<double>& rotation_tolerance,
                                                               geometry_msgs::Quaternion& nominal_orientation)
{
  moveit_msgs::OrientationConstraint orientation_constraint;
  if (rotation_tolerance.size() != 3)
  {
    ROS_ERROR_STREAM("Orientation constraint orienation_tolerance should have length 3, not "
                     << rotation_tolerance.size());
    return orientation_constraint;
  }
  orientation_constraint.header.frame_id = base_link;
  orientation_constraint.link_name = ee_link_name;
  orientation_constraint.orientation = nominal_orientation;
  orientation_constraint.absolute_x_axis_tolerance = rotation_tolerance[0];
  orientation_constraint.absolute_y_axis_tolerance = rotation_tolerance[1];
  orientation_constraint.absolute_z_axis_tolerance = rotation_tolerance[2];
  return orientation_constraint;
}

moveit_msgs::OrientationConstraint createOrientationConstraint(const std::string& base_link,
                                                               const std::string& ee_link_name,
                                                               std::vector<double>& rotation_tolerance,
                                                               tf2::Quaternion& nominal_orientation)
{
  auto quat = tf2::toMsg(nominal_orientation);
  return createOrientationConstraint(base_link, ee_link_name, rotation_tolerance, quat);
}

std::vector<double> jsonToVector(const Json::Value& json_value)
{
  std::vector<double> vec;
  if (!json_value.isArray())
  {
    ROS_ERROR_STREAM("Trying to read json string as vector, but it's not...");
  }
  else
  {
    for (auto val : json_value)
    {
      vec.push_back(val.asDouble());
    }
  }
  return vec;
}

geometry_msgs::Pose jsonToPoseMsg(const Json::Value& json_value)
{
  std::vector<double> position{ jsonToVector(json_value["xyz"]) };
  std::vector<double> orientation{ jsonToVector(json_value["rpy"]) };

  geometry_msgs::Pose pose;
  if (position.size() != 3)
  {
    ROS_ERROR_STREAM("Pose xyz (nominal position) should have length 3, not " << position.size());
    return pose;
  }

  if (orientation.size() != 3)
  {
    ROS_ERROR_STREAM("Pose rpy (nominal orientation) should have length 3, not " << orientation.size());
    return pose;
  }

  tf2::Quaternion quat;
  quat.setRPY(orientation[0], orientation[1], orientation[2]);
  pose.orientation = tf2::toMsg(quat);
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];

  return pose;
}

moveit_msgs::PositionConstraint readPositionConstraint(const Json::Value& con, const std::string fixed_frame)
{
  const std::string ee_link = con.get("link_name", {}).asString();
  std::vector<double> dimensions{ jsonToVector(con["dims"]) };
  std::vector<double> nominal_position{ jsonToVector(con["xyz"]) };
  std::vector<double> nominal_orientation{ jsonToVector(con["rpy"]) };

  moveit_msgs::PositionConstraint position_constraint;
  if (dimensions.size() != 3)
  {
    ROS_ERROR_STREAM("Position constraint dimensions should have length 3, not " << dimensions.size());
    return position_constraint;
  }

  if (nominal_position.size() != 3)
  {
    ROS_ERROR_STREAM("Position constraint xyz (nominal position) should have length 3, not "
                     << nominal_position.size());
    return position_constraint;
  }

  if (nominal_orientation.size() != 3)
  {
    ROS_ERROR_STREAM("Position constraint rpy (nominal orientation) should "
                     "have length 3, not "
                     << nominal_orientation.size());
    return position_constraint;
  }

  shape_msgs::SolidPrimitive box_constraint;
  box_constraint.type = shape_msgs::SolidPrimitive::BOX;
  box_constraint.dimensions = dimensions; /* use -1 to indicate no constraints. */

  geometry_msgs::Pose box_pose;
  box_pose.position.x = nominal_position[0];
  box_pose.position.y = nominal_position[1];
  box_pose.position.z = nominal_position[2];

  tf2::Quaternion nominal_orientation_quat;
  nominal_orientation_quat.setRPY(nominal_orientation[0], nominal_orientation[1], nominal_orientation[2]);
  box_pose.orientation = tf2::toMsg(nominal_orientation_quat);

  position_constraint.header.frame_id = fixed_frame;
  position_constraint.link_name = ee_link;
  position_constraint.constraint_region.primitives.push_back(box_constraint);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);

  return position_constraint;
}

moveit_msgs::OrientationConstraint readOrientationConstraints(const Json::Value& con, const std::string fixed_frame)
{
  const std::string ee_link = con.get("link_name", {}).asString();
  std::vector<double> nominal_orientation{ jsonToVector(con["rpy"]) };
  std::vector<double> tolerance{ jsonToVector(con["tolerance"]) };

  moveit_msgs::OrientationConstraint orientation_constraint;
  if (tolerance.size() != 3)
  {
    ROS_ERROR_STREAM("Orientation constraint tolerance should have length 3, not " << tolerance.size());
    return orientation_constraint;
  }

  if (nominal_orientation.size() != 3)
  {
    ROS_ERROR_STREAM("Orientation constraint rpy (nominal orientation) should "
                     "have length 3, not "
                     << nominal_orientation.size());
    return orientation_constraint;
  }

  orientation_constraint.header.frame_id = fixed_frame;
  orientation_constraint.link_name = ee_link;

  tf2::Quaternion nominal_orientation_quat;
  nominal_orientation_quat.setRPY(nominal_orientation[0], nominal_orientation[1], nominal_orientation[2]);
  orientation_constraint.orientation = tf2::toMsg(nominal_orientation_quat);

  orientation_constraint.absolute_x_axis_tolerance = tolerance[0];
  orientation_constraint.absolute_y_axis_tolerance = tolerance[1];
  orientation_constraint.absolute_z_axis_tolerance = tolerance[2];

  return orientation_constraint;
}

/** \brief Parse orientation constraints from json file read with the library
 * jsoncpp.
 *
 * The error type for orientation constraints is specified in a hacky way as the
 * name of the constraitns.
 * For the strings identifying the types, see elion_planner/constraints.h in the
 * namespace OrientationErrorType.
 *
 * */
moveit_msgs::Constraints readPathConstraints(const Json::Value& json_constraints, const std::string fixed_frame)
{
  moveit_msgs::Constraints path_constraints;
  for (auto constraint : json_constraints)
  {
    std::string constraint_type{ constraint.get("type", {}).asString() };
    ROS_INFO_STREAM("Reading constraints of type: " << constraint_type);
    if (constraint_type == "position")
    {
      path_constraints.position_constraints.push_back(readPositionConstraint(constraint, fixed_frame));
    }
    else if (constraint_type == "angle_axis")
    {
      path_constraints.orientation_constraints.push_back(readOrientationConstraints(constraint, fixed_frame));
      path_constraints.name = "AngleAxis";
    }
    else if (constraint_type == "roll_pitch_yaw")
    {
      path_constraints.orientation_constraints.push_back(readOrientationConstraints(constraint, fixed_frame));
      path_constraints.name = "RollPitchYaw";
    }
    else
    {
      ROS_ERROR_STREAM("Unknown type of constraints: " << constraint_type);
    }
  }
  return path_constraints;
}

Visuals::Visuals(const std::string& reference_frame, ros::NodeHandle& node_handle)
{
  rvt_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(reference_frame, "/visualization_marker_array");
  rvt_->loadRobotStatePub("/display_robot_state");
  rvt_->enableBatchPublishing();
  ros::Duration(0.5).sleep();
  rvt_->deleteAllMarkers();
  rvt_->trigger();
  display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}

/** publish a green box at the nominal position, with dimensions
 * according to the tolerances.
 *
 * Note: the nominal orientation of position constraints is not a thing yet.
 * */
void Visuals::showPositionConstraints(moveit_msgs::PositionConstraint pos_con)
{
  auto dims = pos_con.constraint_region.primitives.at(0).dimensions;
  // if dim = -1 -> make it large (could be workspace size in the future.)
  const double UNBOUNDED_SIZE{ 3.0 };
  for (auto& dim : dims)
  {
    if (dim == -1.0)
      dim = UNBOUNDED_SIZE;
  }
  rvt_->publishCuboid(pos_con.constraint_region.primitive_poses.at(0), dims.at(0), dims.at(1), dims.at(2),
                      rviz_visual_tools::BLUE);
  rvt_->trigger();
}

/** Display trajectory using the DisplayTrajectory publisher and
 * show end-effector path using moveit visual tools.
 * */
void Visuals::displaySolution(planning_interface::MotionPlanResponse res,
                              const robot_state::JointModelGroup* joint_model_group, bool withOrientation)
{
  moveit_msgs::DisplayTrajectory display_trajectory;

  // /* Visualize the trajectory */
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  rvt_->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  rvt_->trigger();
  display_publisher.publish(display_trajectory);

  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  if (withOrientation)
  {
    for (int k = 0; k < res.trajectory_->getWayPointCount(); ++k)
    {
      geometry_msgs::Pose pose_msg;
      const Eigen::Isometry3d& end_effector_transform =
          res.trajectory_->getWayPoint(k).getGlobalLinkTransform(link_model_names.back());
      pose_msg = tf2::toMsg(end_effector_transform);
      rvt_->publishAxis(pose_msg);
    }
    rvt_->trigger();
  }
}

planning_interface::MotionPlanRequest createPTPProblem(const std::vector<double>& start,
                                                       const std::vector<double>& goal,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group)
{
  planning_interface::MotionPlanRequest req;
  req.group_name = joint_model_group->getName();

  // fill out start state in request
  robot_state::RobotState start_state(robot_model);
  start_state.setJointGroupPositions(joint_model_group, start);
  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  // fill out goal state in request
  robot_state::RobotState goal_state(robot_model);
  goal_state.setJointGroupPositions(joint_model_group, goal);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  return req;
}

}  // namespace elion