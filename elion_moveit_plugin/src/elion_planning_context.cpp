#include "elion_moveit_plugin/elion_planning_context.h"

#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_state/conversions.h>

namespace elion
{
ElionPlanningContext::ElionPlanningContext(const std::string& name, const std::string& group,
                                           moveit::core::RobotModelConstPtr robot_model)
  : PlanningContext(name, group), robot_model_(robot_model), joint_model_group_(robot_model->getJointModelGroup(group))
{
  robot_state_.reset(new moveit::core::RobotState(robot_model));
  robot_state_->setToDefaultValues();
  elion_planner_ = ElionPlannerPtr(new elion::ElionPlanner());
  num_dofs_ = robot_model->getJointModelGroup(group)->getVariableCount();
}

void ElionPlanningContext::clear()
{
}

bool ElionPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // ROS_INFO_STREAM("Solving a motion planning request.");
  // TODO figure out how to do selection based on request content
  bool use_current_state{ false };
  Eigen::VectorXd start_joint_positions(num_dofs_);
  if (use_current_state)
  {
    auto start_state = planning_scene_->getCurrentState();
    start_state.copyJointGroupPositions(joint_model_group_, start_joint_positions);
  }
  else
  {
    // read start state from planning request
    robot_state::RobotState start_state(robot_model_);
    moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
    start_state.copyJointGroupPositions(joint_model_group_, start_joint_positions);
    // or cast joint positions from std vector to Eigen?
  }

  ROS_INFO_STREAM("Start state: " << start_joint_positions);

  // extract goal from planning request
  Eigen::VectorXd goal_joint_positions(num_dofs_);
  ROS_INFO_STREAM("num goal constraints: " << request_.goal_constraints.size());
  std::size_t joint_index{ 0 };
  for (auto& joint_constraint : request_.goal_constraints[0].joint_constraints)
  {
    ROS_INFO_STREAM("name: " << joint_constraint.joint_name << " value: " << joint_constraint.position);
    goal_joint_positions[joint_index] = joint_constraint.position;
    joint_index++;
  }
  ROS_INFO_STREAM("goal state: " << goal_joint_positions);

  elion_planner_->preSolve(robot_model_, joint_model_group_->getName(), getPlanningScene(), request_);

  double allowed_planning_time = request_.allowed_planning_time;
  if (allowed_planning_time == 0.0)
  {
    ROS_INFO_STREAM("Settinga allowed planning time to default value of 5 seconds.");
    allowed_planning_time = 5.0;
  }
  auto success = elion_planner_->solve(start_joint_positions, goal_joint_positions, allowed_planning_time);

  if (success)
  {
    elion_planner_->postSolve();
    res.trajectory_ = createRobotTrajectoryFromSolution(elion_planner_->getSolutionPath());
  }

  return success;
}

bool ElionPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  return false;
}

bool ElionPlanningContext::terminate()
{
  return true;
}

robot_trajectory::RobotTrajectoryPtr
ElionPlanningContext::createRobotTrajectoryFromSolution(std::vector<Eigen::VectorXd> path)
{
  if (path.size() == 0)
  {
    ROS_ERROR_STREAM("Cannot create robot trajectory from empty path.");
  }

  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, request_.group_name);

  for (std::size_t path_index = 0; path_index < path.size(); ++path_index)
  {
    size_t joint_index = 0;
    auto state = std::make_shared<moveit::core::RobotState>(planning_scene_->getCurrentState());
    for (const moveit::core::JointModel* jm : trajectory->getGroup()->getActiveJointModels())
    {
      assert(jm->getVariableCount() == 1);
      state->setVariablePosition(jm->getFirstVariableIndex(), path[path_index][joint_index++]);
    }
    trajectory->addSuffixWayPoint(state, 0.0);
  }
  return trajectory;
}
}  // namespace elion_plugin