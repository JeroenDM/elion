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

void addObstacles(elion::Visuals& vis)
{
  geometry_msgs::Pose table_pose, box1_pose, box2_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.85;
  table_pose.position.y = 0.25;
  table_pose.position.z = 0.13;
  vis.rvt_->publishCuboid(table_pose, 0.5, 0.25, 0.01, rviz_visual_tools::GREEN);
  vis.rvt_->publishCollisionCuboid(table_pose, 0.5, 0.25, 0.01, "table", rviz_visual_tools::GREEN);

  box1_pose.orientation.w = 1.0;
  box1_pose.position.x = 0.85;
  box1_pose.position.y = -0.1;
  box1_pose.position.z = 0.08;
  vis.rvt_->publishCuboid(box1_pose, 0.1, 0.1, 0.1, rviz_visual_tools::GREEN);
  vis.rvt_->publishCollisionCuboid(box1_pose, 0.1, 0.1, 0.1, "box1", rviz_visual_tools::GREEN);

  box2_pose.orientation.w = 1.0;
  box2_pose.position.x = 0.85;
  box2_pose.position.y = -0.1;
  box2_pose.position.z = 0.32;
  vis.rvt_->publishCuboid(box2_pose, 0.1, 0.1, 0.1, rviz_visual_tools::GREEN);
  vis.rvt_->publishCollisionCuboid(box2_pose, 0.1, 0.1, 0.1, "box2", rviz_visual_tools::GREEN);
  vis.rvt_->trigger();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compl_example");
  ros::AsyncSpinner spinner(2);
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
  //   auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  //   planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // Visualization and planning scene
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  elion::Visuals visuals(FIXED_FRAME, node_handle);

  addObstacles(visuals);

  auto psm = visuals.rvt_->getPlanningSceneMonitor();

  // Load the planning plugin
  // ^^^^^^^^^^^^^^^^^^^^^^^^

  elion::ClassLoaderSPtr planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  elion::loadPlanningPlugin(planner_plugin_loader, planner_instance, robot_model, node_handle, BASE_CLASS,
                            PLANNING_PLUGIN);

  // Create a motion planning request
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // start = create_pose_msg([1.0, 0.2, 0.4, 0, 90, 0])
  // stop = create_pose_msg([1.0, -0.2, 0.4, 0, 90, 0])
  geometry_msgs::Pose start_pose;
  tf2::Quaternion orientation;
  orientation.setRPY(0, deg2rad(180), 0);
  start_pose.orientation = tf2::toMsg(orientation);
  start_pose.position.x = 0.8;
  start_pose.position.y = 0.2;
  start_pose.position.z = 0.2;

  geometry_msgs::Pose goal_pose;
  tf2::Quaternion orientation2;
  orientation2.setRPY(0, deg2rad(90), 0);
  goal_pose.orientation = tf2::toMsg(orientation2);
  goal_pose.position.x = 0.8;
  goal_pose.position.y = -0.2;
  goal_pose.position.z = 0.2;

  auto req = createPTPProblem(start_pose, goal_pose, robot_model, joint_model_group, visuals);
  req.allowed_planning_time = 100.0;

  // Create and fill in the path constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // position constraint to follow weld line
  const std::string ee_link = joint_model_group->getLinkModelNames().back();
  std::vector<double> dims{ 0.01, 0.4, 0.01 };
  std::vector<double> pos{ 0.8, 0.0, 0.2 };
  auto position_constraint = elion::createPositionConstraint(FIXED_FRAME, ee_link, dims, pos);

  req.path_constraints.position_constraints.push_back(position_constraint);
  visuals.showPositionConstraints(position_constraint);
  visuals.rvt_->trigger();

  // Solve the problems
  // ^^^^^^^^^^^^^^^^^^
  bool success{ false };

  planning_interface::MotionPlanResponse res;
  auto context1 = planner_instance->getPlanningContext(psm->getPlanningScene(), req, res.error_code_);
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