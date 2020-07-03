#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <jsoncpp/json/json.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include "elion_examples/util.h"

const std::string BASE_CLASS = "planning_interface::PlannerManager";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elion_examples");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  // Read a configuration file
  // ^^^^^^^^^^^^^^^^^^^^^^^^^

  std::string config_file_name{ "panda_1.json" };
  if (argc == 1)
  {
    ROS_INFO_STREAM("Running default planning example: " << config_file_name);
  }
  else if (argc == 2)
  {
    config_file_name = std::string(argv[1]);
    ROS_INFO_STREAM("Running planning example: " << config_file_name);
  }
  else
  {
    ROS_ERROR_STREAM("You can only supply one command line argument, the config file name.");
  }

  std::string path = ros::package::getPath("elion_examples");
  path += "/config/";
  path += config_file_name;
  ROS_INFO_STREAM("Reading planning setup configuration from: " << path);

  // read the config file
  Json::Value root;
  std::ifstream config_file(path, std::ifstream::binary);
  config_file >> root;

  // read the robot specific settings from the config file
  // ROS_INFO_STREAM("Planning configuration: \n" << root << "\n");
  const Json::Value robot_config{ root["config"] };
  const std::string robot_description{ robot_config.get("robot_description", {}).asString() };
  const std::string planning_group{ robot_config.get("planning_group", {}).asString() };
  const std::string fixed_frame{ robot_config.get("fixed_frame", {}).asString() };
  const std::string planning_plugin_name{ robot_config.get("planning_plugin_name", {}).asString() };

  // Setup MoveIt related handles to robot, planning scene, ...
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // The usual spiel to setup all MoveIt objects to manage robot state and planning scene
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(robot_description));
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

  elion::ClassLoaderSPtr planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  elion::loadPlanningPlugin(planner_plugin_loader, planner_instance, robot_model, node_handle, BASE_CLASS,
                            planning_plugin_name);

  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // Create the planning request
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^

  std::vector<double> start_joint_values{ elion::jsonToVector(root["start"]["values"]) };
  std::vector<double> goal_joint_values{ elion::jsonToVector(root["goal"]["values"]) };

  auto req1 = elion::createPTPProblem(start_joint_values, goal_joint_values, robot_model, joint_model_group);
  req1.path_constraints = elion::readPathConstraints(root["constraints"], fixed_frame);

  req1.allowed_planning_time = robot_config.get("allowed_planning_time", 5.0).asDouble();

  // Visualization
  // ^^^^^^^^^^^^^
  elion::Visuals visuals(fixed_frame, node_handle);

  // I don't know I nice way to publish two robot states at once with MoveIt visual tools
  // Therefore I just put a pause in between to show them both in sequence.
  visuals.rvt_->publishRobotState(req1.start_state.joint_state.position, joint_model_group, rviz_visual_tools::GREEN);
  visuals.rvt_->trigger();
  ros::Duration(0.5).sleep();
  visuals.rvt_->publishRobotState(goal_joint_values, joint_model_group, rviz_visual_tools::ORANGE);
  visuals.rvt_->trigger();

  visuals.rvt_->deleteAllMarkers();
  visuals.rvt_->trigger();

  if (req1.path_constraints.position_constraints.size() > 0)
    visuals.showPositionConstraints(req1.path_constraints.position_constraints.at(0));

  // Solve the problems
  // ^^^^^^^^^^^^^^^^^^
  bool success{ false };

  planning_interface::MotionPlanResponse res1;
  auto context1 = planner_instance->getPlanningContext(planning_scene, req1, res1.error_code_);
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
    visuals.displaySolution(res1, joint_model_group, true);
  }

  ros::shutdown();
  return 0;
}