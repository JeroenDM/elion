#include "elion_examples/rviz_util.h"

#include <tf2_eigen/tf2_eigen.h>

namespace elion
{
Visuals::Visuals(const std::string& reference_frame, ros::NodeHandle& node_handle)
{
  rvt_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(reference_frame, "/visualization_marker_array");
  rvt_->loadRobotStatePub("/display_robot_state");
  rvt_->enableBatchPublishing();
  ros::Duration(0.5).sleep();
  rvt_->deleteAllMarkers();
  rvt_->trigger();
  display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}

/** publish a green box at the nominal position, with dimensions
 * according to the tolerances.
 *
 * Note: the nominal orientation of position constraints is not a thing yet.
 * */
void Visuals::showPositionConstraints(moveit_msgs::PositionConstraint pos_con)
{
  auto dims = pos_con.constraint_region.primitives.at(0).dimensions;
  // if dim = -1 -> make it large (could be workspace size in the future.)
  const double UNBOUNDED_SIZE{ 3.0 };
  for (auto& dim : dims)
  {
    if (dim == -1.0)
      dim = UNBOUNDED_SIZE;
  }
  rvt_->publishCuboid(pos_con.constraint_region.primitive_poses.at(0), dims.at(0), dims.at(1), dims.at(2),
                      rviz_visual_tools::TRANSLUCENT_DARK);
  rvt_->trigger();
}

/** Display trajectory using the DisplayTrajectory publisher and
 * show end-effector path using moveit visual tools.
 * */
void Visuals::displaySolution(planning_interface::MotionPlanResponse res,
                              const robot_state::JointModelGroup* joint_model_group, bool withOrientation)
{
  moveit_msgs::DisplayTrajectory display_trajectory;

  // /* Visualize the trajectory */
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  rvt_->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  rvt_->trigger();
  display_publisher.publish(display_trajectory);

  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  if (withOrientation)
  {
    for (std::size_t k = 0; k < res.trajectory_->getWayPointCount(); ++k)
    {
      geometry_msgs::Pose pose_msg;
      const Eigen::Isometry3d& end_effector_transform =
          res.trajectory_->getWayPoint(k).getGlobalLinkTransform(link_model_names.back());
      pose_msg = tf2::toMsg(end_effector_transform);
      rvt_->publishAxis(pose_msg);
    }
    rvt_->trigger();
  }
}

void showPositionConstraints(moveit_msgs::PositionConstraint pos_con, moveit_visual_tools::MoveItVisualTools& mvt)
{
  auto dims = pos_con.constraint_region.primitives.at(0).dimensions;
  // if dim = -1 -> make it large (could be workspace size in the future.)
  const double UNBOUNDED_SIZE{ 3.0 };
  for (auto& dim : dims)
  {
    if (dim == -1.0)
      dim = UNBOUNDED_SIZE;
  }
  mvt.publishCuboid(pos_con.constraint_region.primitive_poses.at(0), dims.at(0), dims.at(1), dims.at(2),
                    rviz_visual_tools::TRANSLUCENT_DARK);
  mvt.trigger();
}

void displaySolution(planning_interface::MotionPlanResponse res, const robot_state::JointModelGroup* joint_model_group,
                     moveit_visual_tools::MoveItVisualTools& mvt, bool withOrientation)
{
  moveit_msgs::DisplayTrajectory display_trajectory;

  // /* Visualize the trajectory */
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  mvt.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  mvt.trigger();
  // display_publisher.publish(display_trajectory);
  // mvt.publishTrajectoryPath(res.trajectory_);

  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  if (withOrientation)
  {
    for (std::size_t k = 0; k < res.trajectory_->getWayPointCount(); ++k)
    {
      geometry_msgs::Pose pose_msg;
      const Eigen::Isometry3d& end_effector_transform =
          res.trajectory_->getWayPoint(k).getGlobalLinkTransform(link_model_names.back());
      pose_msg = tf2::toMsg(end_effector_transform);
      mvt.publishAxis(pose_msg);
    }
    mvt.trigger();
  }
}

}  // namespace elion
