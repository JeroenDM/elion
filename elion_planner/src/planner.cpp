#include "elion_planner/planner.h"

#include <iostream>

// #include <ompl/geometric/planners/kpiece/BKPIECE1.h> // this planner causes compiler warnings
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>

#include "elion_planner/collision_checking.h"

namespace elion
{
bool isValid(const ob::State* state)
{
  return true;
}

ElionPlanner::ElionPlanner()
{
  ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);  // TODO nut sure if this is working
}

void ElionPlanner::preSolve(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                            const planning_scene::PlanningSceneConstPtr& ps,
                            planning_interface::MotionPlanRequest request)
{
  num_dofs_ = robot_model->getJointModelGroup(group)->getVariableCount();

  state_space_ = std::make_shared<ob::RealVectorStateSpace>(num_dofs_);
  ob::RealVectorBounds bounds(num_dofs_);
  bounds.setLow(-2 * M_PI);
  bounds.setHigh(2 * M_PI);
  state_space_->setBounds(bounds);

  constraints_ = createConstraint(robot_model, group, request.path_constraints);
  if (!constraints_)
  {
    ROS_ERROR_STREAM("Failed to create constraints");
    return;
  }

  // Only projected state space works for how the constraints are forumulated
  // now.
  // Atlas and TangentBundle give exception:
  //   ompl::base::AtlasStateSpace::newChart(): Failed because manifold looks
  //   degenerate here.
  // constrained_state_space_ =
  // std::make_shared<ob::TangentBundleStateSpace>(state_space_, constraints_);
  // constrained_state_space_ =
  // std::make_shared<ob::AtlasStateSpace>(state_space_, constraints_);

  constrained_state_space_ = std::make_shared<ob::ProjectedStateSpace>(state_space_, constraints_);

  constrained_state_space_info_ = std::make_shared<ob::ConstrainedSpaceInformation>(constrained_state_space_);

  simple_setup_ = std::make_shared<og::SimpleSetup>(constrained_state_space_info_);

  planner_ = selectAndCreatePlanner(request.planner_id, constrained_state_space_info_);

  simple_setup_->setPlanner(planner_);

  simple_setup_->setStateValidityChecker(
      std::make_shared<MoveItCollisionChecker>(constrained_state_space_info_, robot_model, group, ps));
}

bool ElionPlanner::solve(const Eigen::Ref<const Eigen::VectorXd>& start_joint_positions,
                         const Eigen::Ref<const Eigen::VectorXd>& goal_joint_positions, double allowed_planning_time)
{
  ob::ScopedState<> start(constrained_state_space_);
  ob::ScopedState<> goal(constrained_state_space_);

  start->as<ob::ConstrainedStateSpace::StateType>()->copy(start_joint_positions);
  goal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal_joint_positions);
  simple_setup_->setStartAndGoalStates(start, goal);

  // for Atlas and TangentBundle, the start and goal states have to be anchored.
  // constrained_state_space_->as<ob::AtlasStateSpace>()->anchorChart(start.get());
  // constrained_state_space_->as<ob::AtlasStateSpace>()->anchorChart(goal.get());

  // solving it
  simple_setup_->setup();
  ob::PlannerStatus stat = simple_setup_->solve(allowed_planning_time);
  if (stat == ob::PlannerStatus::EXACT_SOLUTION)
  {
    return true;
  }
  else
  {
    OMPL_WARN("No solution found!");
    return false;
  }
}

bool ElionPlanner::checkSolution()
{
  if (solution_path_.size() < 2)
  {
    ROS_ERROR_STREAM("There is no solution path to check.");
    return false;
  }

  const double constraint_tolerance{ constraints_->getTolerance() };
  Eigen::VectorXd constraint_violation{ constraints_->getCoDimension() };

  for (auto& q : solution_path_)
  {
    constraints_->function(q, constraint_violation);
    for (Eigen::Index dim{ 0 }; dim < constraint_violation.size(); ++dim)
    {
      if (std::abs(constraint_violation[dim]) > constraint_tolerance)
      {
        ROS_INFO_STREAM("Constraints violated along the path for dimension " << dim << " at state: ");
        ROS_INFO_STREAM(q.transpose());
        ROS_INFO_STREAM(constraint_violation.transpose());
        return false;
      }
    }
  }
  return true;
}

void ElionPlanner::postSolve()
{
  simple_setup_->simplifySolution(5.);
  auto path = simple_setup_->getSolutionPath();
  path.interpolate();

  // path.printAsMatrix(std::cout);
  ROS_INFO_STREAM("Writing path from OMPL to generic format.");

  // write path to generic format indepenent from OMPL to pass it to ROS?
  solution_path_.clear();
  for (auto& state : path.getStates())
  {
    const Eigen::Map<Eigen::VectorXd>& x = *state->as<ob::ConstrainedStateSpace::StateType>();

    Eigen::VectorXd joint_position(x);
    solution_path_.push_back(joint_position);
  }

  // bool is_path_valid = path.check();
  bool is_path_valid = checkSolution();
  ROS_INFO_STREAM("Is OMPL interpolation valid? " << (is_path_valid ? "yes" : "no"));
}

ob::PlannerPtr ElionPlanner::selectAndCreatePlanner(const std::string& planner_id,
                                                    ob::ConstrainedSpaceInformationPtr space_info) const
{
  if (planner_id == "RRTConnect")
  {
    return std::make_shared<og::RRTConnect>(space_info);
  }
  else if (planner_id == "RRTstar")
  {
    return std::make_shared<og::RRTstar>(space_info);
  }
  else if (planner_id == "PRM")
  {
    return std::make_shared<og::PRM>(space_info);
  }
  else if (planner_id == "PRMstar")
  {
    return std::make_shared<og::PRMstar>(space_info);
  }
  else
  {
    ROS_ERROR_STREAM("Unkown planner id: " << planner_id);
    return nullptr;
  }
}

}  // namespace elion
