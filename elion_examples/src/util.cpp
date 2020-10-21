#include "elion_examples/util.h"

#include <jsoncpp/json/json.h>
#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>
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
    if (!planner_instance->initialize(robot_model, "/move_group"))
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

planning_interface::MotionPlanRequest createPTPProblem(const std::vector<double>& start,
                                                       const std::vector<double>& goal,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       moveit_visual_tools::MoveItVisualTools& mvt)
{
  planning_interface::MotionPlanRequest req;
  req.group_name = joint_model_group->getName();

  // fill out start state in request
  robot_state::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  start_state.setJointGroupPositions(joint_model_group, start);
  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  // fill out goal state in request
  robot_state::RobotState goal_state(robot_model);
  goal_state.setToDefaultValues();
  goal_state.setJointGroupPositions(joint_model_group, goal);
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, 0.001);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // I don't know I nice way to publish two robot states at once with MoveIt
  // visual tools
  // Therefore I just put a pause in between to show them both in sequence.
  mvt.publishRobotState(start, joint_model_group, rviz_visual_tools::GREEN);
  mvt.trigger();
  ros::Duration(1.0).sleep();
  mvt.publishRobotState(goal, joint_model_group, rviz_visual_tools::ORANGE);
  mvt.trigger();

  return req;
}

planning_interface::MotionPlanRequest createPTPProblem(const std::vector<double>& start, geometry_msgs::Pose& goal_pose,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       moveit_visual_tools::MoveItVisualTools& mvt)
{
  planning_interface::MotionPlanRequest req;
  req.group_name = joint_model_group->getName();

  // fill out start state in request
  robot_state::RobotState start_state(robot_model);
  start_state.setJointGroupPositions(joint_model_group, start);
  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  // fill out goal state in request
  geometry_msgs::PoseStamped goal_pose_stamped;
  goal_pose_stamped.pose = goal_pose;
  goal_pose_stamped.header.frame_id = "world";

  moveit_msgs::Constraints goal =
      kinematic_constraints::constructGoalConstraints("gripper_reference", goal_pose_stamped, 0.001, 0.001);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(goal);

  // I don't know I nice way to publish two robot states at once with MoveIt
  // visual tools
  // Therefore I just put a pause in between to show them both in sequence.
  mvt.publishRobotState(start, joint_model_group, rviz_visual_tools::GREEN);
  mvt.trigger();
  mvt.publishAxisLabeled(goal_pose, "Goal Pose");
  mvt.trigger();

  return req;
}

planning_interface::MotionPlanRequest createPTPProblem(geometry_msgs::Pose& start_pose, geometry_msgs::Pose& goal_pose,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       moveit_visual_tools::MoveItVisualTools& mvt)
{
  mvt.publishAxis(start_pose);
  mvt.publishAxis(goal_pose);
  mvt.trigger();
  planning_interface::MotionPlanRequest req;
  req.group_name = joint_model_group->getName();

  // fill out start state in request
  robot_state::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  start_state.setToRandomPositions();
  bool success = start_state.setFromIK(joint_model_group, start_pose, 10.0);
  ROS_INFO_STREAM("Start pose IK: " << (success ? "succeeded." : "failed."));
  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  Eigen::VectorXd js(6);
  start_state.copyJointGroupPositions(joint_model_group, js);
  std::cout << "Start state: " << js.transpose() << std::endl;

  // fill out goal state in request
  robot_state::RobotState goal_state(start_state);
  goal_state.setToDefaultValues();
  success = goal_state.setFromIK(joint_model_group, goal_pose, 10.0);
  ROS_INFO_STREAM("Goal pose IK: " << (success ? "succeeded." : "failed."));
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, 0.001);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  goal_state.copyJointGroupPositions(joint_model_group, js);
  std::cout << "Goal state: " << js.transpose() << std::endl;

  // I don't know I nice way to publish two robot states at once with MoveIt
  // visual tools
  // Therefore I just put a pause in between to show them both in sequence.
  mvt.publishRobotState(start_state, rviz_visual_tools::GREEN);
  mvt.trigger();
  ros::Duration(1.5).sleep();
  mvt.publishRobotState(goal_state, rviz_visual_tools::ORANGE);
  mvt.trigger();

  return req;
}

}  // namespace elion
