#include <fstream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include <jsoncpp/json/json.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include "elion_examples/util.h"

const std::string BASE_CLASS = "planning_interface::PlannerManager";

planning_interface::MotionPlanRequest createPTPProblem(const std::vector<double>& start,
                                                       const std::vector<double>& goal,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       elion::Visuals& visuals)
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

  // I don't know I nice way to publish two robot states at once with MoveIt
  // visual tools
  // Therefore I just put a pause in between to show them both in sequence.
  visuals.rvt_->publishRobotState(start, joint_model_group, rviz_visual_tools::GREEN);
  visuals.rvt_->trigger();
  ros::Duration(1.0).sleep();
  visuals.rvt_->publishRobotState(goal, joint_model_group, rviz_visual_tools::ORANGE);
  visuals.rvt_->trigger();

  return req;
}

planning_interface::MotionPlanRequest createPTPProblem(geometry_msgs::Pose& start_pose, geometry_msgs::Pose& goal_pose,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       elion::Visuals& visuals)
{
  planning_interface::MotionPlanRequest req;
  req.group_name = joint_model_group->getName();

  // fill out start state in request
  robot_state::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  bool success = start_state.setFromIK(joint_model_group, start_pose, 10.0);
  ROS_INFO_STREAM("Start pose IK: " << (success ? "succeeded." : "failed."));
  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  // fill out goal state in request
  robot_state::RobotState goal_state(start_state);
  goal_state.setToDefaultValues();
  success = goal_state.setFromIK(joint_model_group, goal_pose, 10.0);
  ROS_INFO_STREAM("Goal pose IK: " << (success ? "succeeded." : "failed."));
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // I don't know I nice way to publish two robot states at once with MoveIt
  // visual tools
  // Therefore I just put a pause in between to show them both in sequence.
  visuals.rvt_->publishRobotState(start_state, rviz_visual_tools::GREEN);
  visuals.rvt_->trigger();
  ros::Duration(1.5).sleep();
  visuals.rvt_->publishRobotState(goal_state, rviz_visual_tools::ORANGE);
  visuals.rvt_->trigger();

  return req;
}

void readAndAddObstacles(const Json::Value& collision_objects, elion::Visuals& vis)
{
  for (auto object : collision_objects)
  {
    std::string name{ object.get("name", {}).asString() };
    std::vector<double> dimensions{ elion::jsonToVector(object["dims"]) };

    if (dimensions.size() != 3)
    {
      ROS_ERROR_STREAM("Collision object cubiod dimensions should have length 3, not " << dimensions.size());
    }

    geometry_msgs::Pose box1_pose;

    box1_pose = elion::jsonToPoseMsg(object);

    vis.rvt_->publishCollisionCuboid(box1_pose, dimensions[0], dimensions[1], dimensions[2], name,
                                     rviz_visual_tools::GREEN);
    vis.rvt_->trigger();
  }
}

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

  // The usual spiel to setup all MoveIt objects to manage robot state and
  // planning scene
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(robot_description));
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

  elion::ClassLoaderSPtr planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  elion::loadPlanningPlugin(planner_plugin_loader, planner_instance, robot_model, node_handle, BASE_CLASS,
                            planning_plugin_name);

  // auto planning_scene =
  // std::make_shared<planning_scene::PlanningScene>(robot_model);
  // planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group,
  // "ready");

  // Visualization
  // ^^^^^^^^^^^^^
  elion::Visuals visuals(fixed_frame, node_handle);

  visuals.rvt_->deleteAllMarkers();
  visuals.rvt_->trigger();

  auto psm = visuals.rvt_->getPlanningSceneMonitor();

  // Obstacles
  // ^^^^^^^^^

  readAndAddObstacles(root["collision_objects"], visuals);

  psm->getPlanningScene()->printKnownObjects();

  // Create the planning request
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // figure out whether start and goal state are given as joint values or
  // end-effector poses
  // TODO this could be moved to the createPTPProblem function
  std::string start_type{ root["start"].get("type", {}).asString() };
  std::string goal_type{ root["goal"].get("type", {}).asString() };

  moveit_msgs::MotionPlanRequest req1;
  if (start_type == "joint_values" && goal_type == "joint_values")
  {
    std::vector<double> start{ elion::jsonToVector(root["start"]["values"]) };
    std::vector<double> goal{ elion::jsonToVector(root["goal"]["values"]) };
    req1 = createPTPProblem(start, goal, robot_model, joint_model_group, visuals);
  }
  else if (start_type == "pose" && goal_type == "pose")
  {
    auto start_pose = elion::jsonToPoseMsg(root["start"]);
    auto goal_pose = elion::jsonToPoseMsg(root["goal"]);
    req1 = createPTPProblem(start_pose, goal_pose, robot_model, joint_model_group, visuals);
  }
  else
  {
    ROS_ERROR_STREAM("Unkown type of start or goal type: " << start_type << ", " << goal_type);
  }

  req1.path_constraints = elion::readPathConstraints(root["constraints"], fixed_frame);

  req1.allowed_planning_time = robot_config.get("allowed_planning_time", 5.0).asDouble();  // 5.0 default planning time
  req1.planner_id = robot_config.get("planner_id", "RRTConnect").asString();  // RRTConnect as default planner

  if (req1.path_constraints.position_constraints.size() > 0)
    visuals.showPositionConstraints(req1.path_constraints.position_constraints.at(0));

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
    visuals.displaySolution(res1, joint_model_group, true);
  }

  ros::shutdown();
  return 0;
}