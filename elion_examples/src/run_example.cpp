#include <fstream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include <jsoncpp/json/json.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include "elion_examples/util.h"

const std::string BASE_CLASS = "planning_interface::PlannerManager";

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
  mvt.publishRobotState(start, joint_model_group, rviz_visual_tools::GREEN);
  mvt.trigger();
  ros::Duration(1.0).sleep();
  mvt.publishRobotState(goal, joint_model_group, rviz_visual_tools::ORANGE);
  mvt.trigger();

  return req;
}

planning_interface::MotionPlanRequest createPTPProblem(geometry_msgs::Pose& start_pose, geometry_msgs::Pose& goal_pose,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       moveit_visual_tools::MoveItVisualTools& mvt)
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
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, 0.001);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

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

void readAndAddObstacles(const Json::Value& json_collision_objects,
                         moveit::planning_interface::PlanningSceneInterface& psi, const std::string& planning_frame)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  for (auto object : json_collision_objects)
  {
    std::string name{ object.get("name", {}).asString() };
    std::vector<double> dimensions{ elion::jsonToVector(object["dims"]) };

    if (dimensions.size() != 3)
    {
      ROS_ERROR_STREAM("Collision object cubiod dimensions should have length 3, not " << dimensions.size());
    }

    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame;
    collision_object.id = name;
    collision_object.operation = collision_object.ADD;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = dimensions;

    geometry_msgs::Pose box_pose;
    box_pose = elion::jsonToPoseMsg(object);

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    collision_objects.push_back(collision_object);
  }
  if (collision_objects.size() > 0)
  {
    psi.addCollisionObjects(collision_objects);
  }
}

bool isJsonFileName(const std::string& input)
{
  static const std::string ext{ ".json" };
  std::size_t found = input.find(ext);
  return found != std::string::npos;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elion_examples");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  // Parse command line options
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  std::string config_file_name{ "panda_pos_con.json" };
  std::string command_line_plugin_name{ "" };
  if (argc == 1)
  {
    ROS_INFO_STREAM("Running default planning example: " << config_file_name);
  }
  if (argc > 1)
  {
    auto first_argument = std::string(argv[1]);
    if (isJsonFileName(first_argument))
    {
      config_file_name = std::string(argv[1]);
      ROS_INFO_STREAM("Running planning example: " << config_file_name);
    }
    else
    {
      ROS_ERROR_STREAM("First argument must be a json file name.");
      ros::shutdown();
      return 0;
    }
  }
  if (argc > 2)
  {
    command_line_plugin_name = std::string(argv[2]);
    ROS_INFO_STREAM("Overriding the plugin name from the config file with: " << command_line_plugin_name);
  }
  if (argc > 3)
  {
    ROS_ERROR_STREAM("Only two command line arguments supported. Usage:\n  elion_run_example [optional] "
                     "<config_file_name> <planning_plugin_name>");
    ros::shutdown();
    return 0;
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
  std::string planning_plugin_name{ robot_config.get("planning_plugin_name", {}).asString() };

  if (command_line_plugin_name != "")
    planning_plugin_name = command_line_plugin_name;

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

  // Use the default planning scene published by the move group node.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description);
  bool has_planning_scene = psm->requestPlanningSceneState("/get_planning_scene");
  ROS_INFO_STREAM("Request planning scene " << (has_planning_scene ? "succeeded." : "failed."));
  psm->startSceneMonitor("/move_group/monitored_planning_scene");

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(fixed_frame, "/visualization_marker_array", psm);
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.deleteAllMarkers();
  ros::Duration(1.0).sleep();

  // auto psm = visuals.rvt_->getPlanningSceneMonitor();

  // Obstacles
  // ^^^^^^^^^

  readAndAddObstacles(root["collision_objects"], planning_scene_interface, fixed_frame);

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
    req1 = createPTPProblem(start, goal, robot_model, joint_model_group, visual_tools);
  }
  else if (start_type == "pose" && goal_type == "pose")
  {
    auto start_pose = elion::jsonToPoseMsg(root["start"]);
    auto goal_pose = elion::jsonToPoseMsg(root["goal"]);
    req1 = createPTPProblem(start_pose, goal_pose, robot_model, joint_model_group, visual_tools);
  }
  else
  {
    ROS_ERROR_STREAM("Unkown type of start or goal type: " << start_type << ", " << goal_type);
  }

  req1.path_constraints = elion::readPathConstraints(root["constraints"], fixed_frame);

  req1.allowed_planning_time = robot_config.get("allowed_planning_time", 5.0).asDouble();  // 5.0 default planning time
  req1.planner_id = robot_config.get("planner_id", "RRTConnect").asString();  // RRTConnect as default planner

  if (req1.path_constraints.position_constraints.size() > 0)
    elion::showPositionConstraints(req1.path_constraints.position_constraints.at(0), visual_tools);

  // Solve the problems
  // ^^^^^^^^^^^^^^^^^^
  bool success{ false };

  planning_interface::MotionPlanResponse res1;
  auto context1 = planner_instance->getPlanningContext(psm->getPlanningScene(), req1, res1.error_code_);
  if (context1 != nullptr)
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
    elion::displaySolution(res1, joint_model_group, visual_tools,
                           (req1.path_constraints.orientation_constraints.size() > 0));
  }

  ros::shutdown();
  return 0;
}