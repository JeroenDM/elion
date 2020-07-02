#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
// #include <pluginlib/class_loader.h>
// #include <boost/scoped_ptr.hpp>

#include <jsoncpp/json/json.h>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // toMsg(...)

#include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// #include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
// #include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include "elion_examples/util.h"

const std::string BASE_CLASS = "planning_interface::PlannerManager";

moveit_msgs::PositionConstraint readPositionConstraint(const Json::Value& con, const std::string fixed_frame)
{
  const std::string ee_link = con.get("link_name", {}).asString();
  std::vector<double> dims;
  for (auto val : con["dims"])
  {
    dims.push_back(val.asDouble());
  }
  std::vector<double> pos;
  for (auto val : con["xyz"])
  {
    pos.push_back(val.asDouble());
  }
  ROS_INFO_STREAM("End-effector link: " << ee_link);

  // add a non-identity orientation to the position constraints.
  // tf2::Quaternion position_constraints_quat;
  // position_constraints_quat.setRPY(0.1, 0.7, 0);
  // position_constraint.constraint_region.primitive_poses[0].orientation = tf2::toMsg(position_constraints_quat);

  return elion::createPositionConstraint(fixed_frame, ee_link, dims, pos);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elion_examples");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  std::string path = ros::package::getPath("elion_examples");
  path += "/config/";
  path += "panda_1.json";
  ROS_INFO_STREAM("Reading planning setup configuration from: " << path);

  Json::Value root;

  std::ifstream config_file(path, std::ifstream::binary);
  config_file >> root;

  ROS_INFO_STREAM("Planning configuration: \n" << root << "\n");

  const Json::Value robot_config{ root["config"] };

  const std::string robot_description{ robot_config.get("robot_description", {}).asString() };
  const std::string planning_group{ robot_config.get("planning_group", {}).asString() };
  const std::string fixed_frame{ robot_config.get("fixed_frame", {}).asString() };
  const std::string planning_plugin_name{ robot_config.get("planning_plugin_name", {}).asString() };

  ROS_INFO_STREAM(fixed_frame);
  ROS_INFO_STREAM(robot_description);
  ROS_INFO_STREAM(planning_group);
  ROS_INFO_STREAM(planning_plugin_name);

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

  const Json::Value start{ root["start"]["values"] };
  ROS_INFO_STREAM("is array: " << (start.isArray() ? "yes" : "no"));
  std::vector<double> start_joint_values;
  for (auto val : start)
  {
    start_joint_values.push_back(val.asDouble());
  }
  ROS_INFO_STREAM("ndof: " << start_joint_values.size());

  const Json::Value goal{ root["goal"]["values"] };
  ROS_INFO_STREAM("is array: " << (goal.isArray() ? "yes" : "no"));
  std::vector<double> goal_joint_values;
  for (auto val : goal)
  {
    goal_joint_values.push_back(val.asDouble());
  }
  ROS_INFO_STREAM("ndof: " << goal_joint_values.size());

  auto req1 = elion::createPTPProblem(start_joint_values, goal_joint_values, robot_model, joint_model_group);

  const std::string ee_link = joint_model_group->getLinkModelNames().back();
  std::vector<double> dims{ 0.1, 0.6, 0.1 };
  std::vector<double> pos{ 0.3, 0.0, 0.65 };
  auto position_constraint = readPositionConstraint(root["constraints"][0], fixed_frame);

  req1.allowed_planning_time = 10.0;

  req1.path_constraints.position_constraints.push_back(position_constraint);

  // Visualization
  // ^^^^^^^^^^^^^
  elion::Visuals visuals(fixed_frame, node_handle);
  visuals.showPositionConstraints(req1.path_constraints.position_constraints.at(0));

  // I don't know I nice way to publish two robot states at once with MoveIt visual tools
  // Therefore I just put a pause in between to show them both in sequence.
  visuals.rvt_->publishRobotState(req1.start_state.joint_state.position, joint_model_group, rviz_visual_tools::GREEN);
  visuals.rvt_->trigger();
  ros::Duration(0.5).sleep();
  visuals.rvt_->publishRobotState(goal_joint_values, joint_model_group, rviz_visual_tools::ORANGE);
  visuals.rvt_->trigger();

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
    visuals.displaySolution(res1, joint_model_group, false);
  }

  ros::shutdown();
  return 0;
}