#include <string>
#include <vector>

#include <ros/ros.h>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // toMsg(...)

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include "elion_examples/util.h"

/** Change this parameters for different robots or planning plugins. */
const std::string FIXED_FRAME = "base_link";
const std::string PLANNING_GROUP = "manipulator";

const std::string ROBOT_DESCRIPTION = "robot_description";
const std::string BASE_CLASS = "planning_interface::PlannerManager";
const std::string PLANNING_PLUGIN = "elion/ElionPlanner";
// const std::string PLANNING_PLUGIN = "ompl_interface/OMPLPlanner";

planning_interface::MotionPlanRequest createPTPProblem(geometry_msgs::Pose& start_pose, geometry_msgs::Pose& goal_pose,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       elion::Visuals& visuals)
{
  planning_interface::MotionPlanRequest req;
  req.group_name = PLANNING_GROUP;

  // fill out start state in request
  robot_state::RobotState start_state(robot_model);
  bool success = start_state.setFromIK(joint_model_group, start_pose);
  ROS_INFO_STREAM("Start pose IK: " << (success ? "succeeded." : "failed."));
  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  // fill out goal state in request
  robot_state::RobotState goal_state(start_state);
  success = goal_state.setFromIK(joint_model_group, goal_pose);
  ROS_INFO_STREAM("Goal pose IK: " << (success ? "succeeded." : "failed."));
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // I don't know I nice way to publish two robot states at once with MoveIt visual tools
  // Therefore I just put a pause in between to show them both in sequence.
  visuals.rvt_->publishRobotState(start_state, rviz_visual_tools::GREEN);
  visuals.rvt_->trigger();
  ros::Duration(1.5).sleep();
  visuals.rvt_->publishRobotState(goal_state, rviz_visual_tools::ORANGE);
  visuals.rvt_->trigger();

  return req;
}

inline double deg2rad(double angle_degrees)
{
  return angle_degrees * M_PI / 180.0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compl_example");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL
  // Planning scene setup
  // ^^^^^

  // The usual spiel to setup all MoveIt objects to manage robot state and planning scene
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  // I should probably use the planning scene monitor here
  // but I do not uderstand the planning scene monitor engough to use it.
  // planning_scene::PlanningScene planning_scene(robot_model);
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // Visualization
  // ^^^^^^^^^^^^^
  elion::Visuals visuals(FIXED_FRAME, node_handle);

  // Load the planning plugin
  // ^^^^^^^^^^^^^^^^^^^^^^^^

  elion::ClassLoaderSPtr planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  elion::loadPlanningPlugin(planner_plugin_loader, planner_instance, robot_model, node_handle, BASE_CLASS,
                            PLANNING_PLUGIN);

  // Create a motion planning request
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  geometry_msgs::Pose start_pose;
  tf2::Quaternion orientation;
  orientation.setEuler(0, deg2rad(135), deg2rad(90));
  start_pose.orientation = tf2::toMsg(orientation);
  start_pose.position.x = 0.98;
  start_pose.position.y = -0.5;
  start_pose.position.z = 0.02;

  geometry_msgs::Pose goal_pose;
  //   tf2::Quaternion orientation2;
  //   orientation2.setEuler(0, deg2rad(135), deg2rad(90));
  goal_pose.orientation = tf2::toMsg(orientation);
  goal_pose.position.x = 0.98;
  goal_pose.position.y = 0.5;
  goal_pose.position.z = 0.02;

  auto req = createPTPProblem(start_pose, goal_pose, robot_model, joint_model_group, visuals);
  req.allowed_planning_time = 5.0;

  // Create and fill in the path constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // position constraint to follow weld line
  const std::string ee_link = joint_model_group->getLinkModelNames().back();
  std::vector<double> dims{ 0.05, 1.0, 0.05 };
  std::vector<double> pos{ 0.98, 0.0, 0.02 };
  auto position_constraint = elion::createPositionConstraint(FIXED_FRAME, ee_link, dims, pos);

  req.path_constraints.position_constraints.push_back(position_constraint);
  visuals.showPositionConstraints(position_constraint);
  visuals.rvt_->trigger();

  // orientation constraints only allowing rotation around tool axis,
  // and some rotation in other directions.
  std::vector<double> tolerance{ 0.1, 0.1, -1.0 };
  auto orientation_constraint = elion::createOrientationConstraint(FIXED_FRAME, ee_link, tolerance, orientation);
  req.path_constraints.name = "AngleAxis";  // hack to specify orientation error type
  //   req.path_constraints.orientation_constraints.push_back(orientation_constraint);

  // Solve the problems
  // ^^^^^^^^^^^^^^^^^^
  bool success{ false };

  planning_interface::MotionPlanResponse res;
  auto context1 = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  if (context1)
  {
    success = context1->solve(res);
  }
  else
  {
    ROS_INFO_STREAM("Failed to create planning constext for the first problem.");
  }
  if (res.trajectory_)
  {
    ROS_INFO_STREAM("Path found for position constraints of length: " << res.trajectory_->getWayPointCount());
    visuals.displaySolution(res, joint_model_group, true);
  }

  ros::shutdown();
  return 0;
}