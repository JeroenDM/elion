#ifndef COMPL_PLANNING_CONTEXT_H
#define COMPL_PLANNING_CONTEXT_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <elion_planner/planner.h>

namespace elion
{
// Macro that forward declares a class and defines the respective smartpointers through MOVEIT_DECLARE_PTR.
MOVEIT_CLASS_FORWARD(ElionPlanningContext);

class ElionPlanningContext : public planning_interface::PlanningContext
{
public:
  ElionPlanningContext(const std::string& name, const std::string& group, moveit::core::RobotModelConstPtr robot_model);

  ~ElionPlanningContext() = default;

  void clear() override;

  bool solve(planning_interface::MotionPlanResponse& res) override;

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

  robot_trajectory::RobotTrajectoryPtr createRobotTrajectoryFromSolution(std::vector<Eigen::VectorXd> path);

private:
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;  // Why is this a raw pointer everywhere in MoveIt?
  std::size_t num_dofs_;

  // save a single state to do forward kinematics (not ideal)
  moveit::core::RobotStatePtr robot_state_;

  // it would be nice to have a kinematics solver instance for the constraints
  // but I don't know how to initialize this member properly
  // const kinematics::KinematicsBaseConstPtr kinematics_solver_;

  // the actual planner goes here
  // TODO: This does not have to be a pointer probably
  ElionPlannerPtr elion_planner_;
};
}  // namespace elion_plugin

#endif  // COMPL_PLANNING_CONTEXT_H