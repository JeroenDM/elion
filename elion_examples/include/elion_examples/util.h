#ifndef ELION_EXAMPLES_UTIL_H
#define ELION_EXAMPLES_UTIL_H

#include <string>

#include <boost/scoped_ptr.hpp>
#include <pluginlib/class_loader.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/Constraints.h>
#include <ros/console.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/node_handle.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace elion
{
/** Make this ugly long type a bit shorter. */
typedef boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> ClassLoaderSPtr;

/** Pendantic loading of the planning plugin
 *
 * Really usefull for debugging configuration issues.
 */
void loadPlanningPlugin(ClassLoaderSPtr& planner_plugin_loader, planning_interface::PlannerManagerPtr& planner_instance,
                        robot_model::RobotModelPtr& robot_model, ros::NodeHandle& node_handle,
                        const std::string& base_class, const std::string& plugin_name);

moveit_msgs::PositionConstraint createPositionConstraint(const std::string& base_link, const std::string& ee_link_name,
                                                         std::vector<double>& dimensions,
                                                         std::vector<double>& nominal_position);

moveit_msgs::OrientationConstraint createOrientationConstraint(const std::string& base_link,
                                                               const std::string& ee_link_name,
                                                               std::vector<double>& rotation_tolerance,
                                                               geometry_msgs::Quaternion& nominal_orientation);

moveit_msgs::OrientationConstraint createOrientationConstraint(const std::string& base_link,
                                                               const std::string& ee_link_name,
                                                               std::vector<double>& rotation_tolerance,
                                                               tf2::Quaternion& nominal_orientation);

/** \brief Create joint space start -> joint space goal planning problem + visualization. **/
planning_interface::MotionPlanRequest createPTPProblem(const std::vector<double>& start,
                                                       const std::vector<double>& goal,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       moveit_visual_tools::MoveItVisualTools& mvt);

/** \brief Create joint space start -> end-effector pose goal planning problem + visualization.
 *
 * Uses the default IK solver to get joint space goal.
 * **/
planning_interface::MotionPlanRequest createPTPProblem(const std::vector<double>& start, geometry_msgs::Pose& goal_pose,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       moveit_visual_tools::MoveItVisualTools& mvt);

/** \brief Create end-effector pose start -> end-effector pose goal planning problem + visualization.
 *
 * Uses the default IK solver to get joint space start and goal.
 * **/
planning_interface::MotionPlanRequest createPTPProblem(geometry_msgs::Pose& start_pose, geometry_msgs::Pose& goal_pose,
                                                       robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group,
                                                       moveit_visual_tools::MoveItVisualTools& mvt);
}  // namespace elion
#endif  // ELION_EXAMPLES_UTIL_H
