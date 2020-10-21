#include <string>

#include <ros/node_handle.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_interface/planning_interface.h>

namespace elion
{
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

void showPositionConstraints(moveit_msgs::PositionConstraint pos_con, moveit_visual_tools::MoveItVisualTools& mvt);
void displaySolution(planning_interface::MotionPlanResponse res, const robot_state::JointModelGroup* joint_model_group,
                     moveit_visual_tools::MoveItVisualTools& mvt, bool withOrientation);

}  // namespace elion
