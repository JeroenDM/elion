#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // toMsg(...)

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include "elion_examples/util.h"

/** Change this parameters for different robots or planning plugins. */
const std::string FIXED_FRAME = "panda_link0";
const std::string PLANNING_GROUP = "panda_arm";

const std::string ROBOT_DESCRIPTION = "robot_description";
const std::string BASE_CLASS = "planning_interface::PlannerManager";
const std::string PLANNING_PLUGIN = "elion/ElionPlanner";
// const std::string PLANNING_PLUGIN = "ompl_interface/OMPLPlanner";

planning_interface::MotionPlanRequest createPTPProblem(const std::vector<double>& start,
                                                       const std::vector<double>& goal,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group)
{
  planning_interface::MotionPlanRequest req;
  req.group_name = PLANNING_GROUP;

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compl_example");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node_handle("~");

  elion::Visuals visuals(FIXED_FRAME, node_handle);
  auto psm = visuals.rvt_->getPlanningSceneMonitor();

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
  // auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  // planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // Load the planning plugin
  // ^^^^^^^^^^^^^^^^^^^^^^^^

  elion::ClassLoaderSPtr planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  elion::loadPlanningPlugin(planner_plugin_loader, planner_instance, robot_model, node_handle, BASE_CLASS,
                            PLANNING_PLUGIN);

  // Create a motion planning request
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Move from a joint space start to goal configurations.
  // These configurations are hardcoded below and chooses specifically to move the end-effector from position
  // [0.3, -0.3, 0.6] to position [0.3, 0.3, 0.7].
  //  This makes it easier to come up with path constraints for which the start and goal are valid.
  // clang-format off
  std::vector<double> start_joint_values
  {
      0.666988104319289,
    -0.9954030434136065,
    -1.1194235704518019,
    -1.9946073045682555, 
     -2.772101772642487,
     3.4631937276027194,
    -1.2160652080175647
  };
    std::vector<double> goal_joint_values
    {
     1.7301680303369467,
    -0.7342165592762893, 
    -0.5358506493073328,
     -2.214051132383283,
    -1.9148221683474542,
     1.8324940020482856,
     -1.588014538557859
  };
  // clang-format on

  // we will create two separate requests, one to test position constraints
  // and a second to test orientation constraints
  auto req1 = createPTPProblem(start_joint_values, goal_joint_values, robot_model, joint_model_group);
  auto req2 = createPTPProblem(start_joint_values, goal_joint_values, robot_model, joint_model_group);
  req1.allowed_planning_time = 5.0;
  req2.allowed_planning_time = 5.0;

  // Create and fill in the path constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We can add positoin constraints on the last link of the robot
  // (For the panda_arm, this is not really the end-effector, but we will pretend it is.)
  const std::string ee_link = joint_model_group->getLinkModelNames().back();
  std::vector<double> dims{ 0.1, 0.6, 0.1 };
  std::vector<double> pos{ 0.3, 0.0, 0.65 };
  auto position_constraint = elion::createPositionConstraint(FIXED_FRAME, ee_link, dims, pos);
  req1.path_constraints.position_constraints.push_back(position_constraint);

  // Alternatively we can add orientation constraints on this last link
  // (position and orientation constraints at the same time are not supported yet)
  tf2::Quaternion desired_orientation;
  desired_orientation.setRPY(0, M_PI_2, 0); /** Todo, use intrinsic xyz as in MoveIt's constraitns. */
  std::vector<double> tolerance{ 0.1, 0.1, -1.0 };
  auto orientation_constraint =
      elion::createOrientationConstraint(FIXED_FRAME, ee_link, tolerance, desired_orientation);
  req2.path_constraints.orientation_constraints.push_back(orientation_constraint);

  // for orientation constraints we use a really ugly hack to specify what type
  // of orientation error we want to use (there are 2 options at the moment).
  req2.path_constraints.name = "AngleAxis";
  // req.path_constraints.name = "RollPitchYaw";

  // Visualization
  // ^^^^^^^^^^^^^
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.6;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.5;
  visuals.rvt_->publishCollisionCuboid(box_pose, 0.3, 0.1, 1.0, "box_1", rviz_visual_tools::GREEN);
  visuals.rvt_->trigger();

  visuals.showPositionConstraints(req1.path_constraints.position_constraints.at(0));

  // I don't know I nice way to publish two robot states at once with MoveIt visual tools
  // Therefore I just put a pause in between to show them both in sequence.
  visuals.rvt_->publishRobotState(req1.start_state.joint_state.position, joint_model_group, rviz_visual_tools::GREEN);
  visuals.rvt_->trigger();
  ros::Duration(2.0).sleep();
  visuals.rvt_->publishRobotState(goal_joint_values, joint_model_group, rviz_visual_tools::ORANGE);
  visuals.rvt_->trigger();

  // TODO visualize orientation constraints?

  // Solve the problems
  // ^^^^^^^^^^^^^^^^^^
  bool success{ false };

  planning_interface::MotionPlanResponse res1;
  auto context1 = planner_instance->getPlanningContext(psm->getPlanningScene(), req1, res1.error_code_);
  if (context1)
  {
    success = context1->solve(res1);
  }
  else
  {
    ROS_INFO_STREAM("Failed to create planning constext for the first problem.");
  }
  if (res1.trajectory_)
  {
    ROS_INFO_STREAM("Path found for position constraints of length: " << res1.trajectory_->getWayPointCount());
    visuals.displaySolution(res1, joint_model_group, false);
  }

  planning_interface::MotionPlanResponse res2;
  auto context2 = planner_instance->getPlanningContext(psm->getPlanningScene(), req2, res2.error_code_);
  if (context2)
  {
    success = context2->solve(res2);
  }
  else
  {
    ROS_INFO_STREAM("Failed to create planning constext for the second problem.");
  }
  if (res2.trajectory_)
  {
    ros::Duration(2.0).sleep();  // wait to make sure the previous solution was shown
    ROS_INFO_STREAM("Path found for position constraints of length: " << res2.trajectory_->getWayPointCount());
    visuals.displaySolution(res2, joint_model_group, true);
  }

  ros::shutdown();
  return 0;
}