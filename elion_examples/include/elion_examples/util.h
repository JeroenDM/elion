#ifndef ELION_EXAMPLES_UTIL_H
#define ELION_EXAMPLES_UTIL_H

#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <ros/console.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/Constraints.h>
#include <geometry_msgs/Quaternion.h>

#include <ros/node_handle.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>

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

/** Visuals groups everyting to do with showing stuff in Rviz
 * */
class Visuals
{
public:
  Visuals(const std::string& reference_frame, ros::NodeHandle& node_handle);

  /** publish a green box at the nominal position, with dimensions
   * according to the tolerances.
   *
   * Note: the nominal orientation of position constraints is not a thing yet.
   * */
  void showPositionConstraints(moveit_msgs::PositionConstraint pos_con);

  /** Display trajectory using the DisplayTrajectory publisher and
   * show end-effector path using moveit visual tools.
   * */
  void displaySolution(planning_interface::MotionPlanResponse res,
                       const robot_state::JointModelGroup* joint_model_group, bool withOrientation);

  moveit_visual_tools::MoveItVisualToolsPtr rvt_;
  ros::Publisher display_publisher;
};

}  // namespace elion

#endif  // ELION_EXAMPLES_UTIL_H