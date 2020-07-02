#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>

const std::string ROBOT_DESCRIPTION{ "robot_description" };
const std::string PLANNING_GROUP{ "panda_arm" };

int main(int argc, char** argv)
{
  namespace mvt = moveit_visual_tools;
  namespace rvt = rviz_visual_tools;

  const std::string planning_scene_topic{ "/move_group/monitored_planning_scene" };

  ros::init(argc, argv, "psm_example");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  // The usual spiel to setup all MoveIt objects to manage robot state and planning scene
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  planning_scene::PlanningScenePtr planning_scene;
  planning_scene_monitor::PlanningSceneMonitorPtr psm;

  planning_scene.reset(new planning_scene::PlanningScene(robot_model));
  psm.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  ros::Duration(0.5).sleep();

  psm->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRW ps(psm);
  ps->getCurrentStateNonConst().update();
  planning_scene = ps->diff();
  planning_scene->decoupleParent();

  ros::Duration(0.5).sleep();

  geometry_msgs::Pose block_pose;
  block_pose.orientation.w = 1.0;
  block_pose.position.x = 1.0;

  mvt::MoveItVisualTools visuals("/panda_link0", "/visualization_marker_array", psm);

  visuals.publishCollisionBlock(block_pose, "block_1", 1.0, rvt::PINK);
  visuals.trigger();

  ROS_INFO_STREAM("Collision object is published.");
  ros::Duration(0.1).sleep();

  // planning_scene_monitor::LockedPlanningSceneRO ps(psm);
  ps->printKnownObjects();

  ros::shutdown();
  return 0;

  // planning_scene_monitor::PlanningSceneMonitorPtr psm;

  // // Create tf transform buffer and listener
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // // Regular version b/c the other one causes problems with recognizing end effectors
  // psm.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf_buffer, planning_scene_topic));
  // ros::spinOnce();
  // ros::Duration(0.1).sleep();
  // ros::spinOnce();

  // if (psm->getPlanningScene())
  // {
  //   psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
  //   planning_scene_topic); ROS_INFO_STREAM("Publishing planning scene on " << planning_scene_topic);
  // }
  // else
  // {
  //   ROS_ERROR_STREAM("Planning scene not configured");
  //   return false;
  // }

  // ROS_INFO_STREAM("Planning scene monitor setup finished.");

  // planning_scene_monitor::LockedPlanningSceneRO ps(psm);
  // ps->printKnownObjects();

  // mvt::MoveItVisualTools visuals("/panda_link0", "/visualization_marker_array");
  // visuals.setPlanningSceneTopic(planning_scene_topic);
  // bool success = visuals.loadPlanningSceneMonitor();
  // ROS_INFO_STREAM("Did I load the planning scene? " << (success ? "yes" : "no"));
  // auto psm = visuals.getPlanningSceneMonitor();

  // ros::Duration(0.1).sleep();
  // planning_scene_monitor::LockedPlanningSceneRO ps(psm);
  // ps->printKnownObjects();
}