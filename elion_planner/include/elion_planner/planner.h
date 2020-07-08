/** OMPL interface class
 *
 * Maybe change name, is it an interface, a planner, ...?
 * I would like to introduce the symbol q for joint_positions. this would make
 * a lot of code cleaner.
 * */
#pragma once

#include <memory>

#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_scene/planning_scene.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Planner.h>

#include "elion_planner/constraint.h"

namespace elion
{
namespace ob = ompl::base;
namespace og = ompl::geometric;

MOVEIT_CLASS_FORWARD(ElionPlanner);

class ElionPlanner
{
public:
  ElionPlanner();

  /** \brief Create OMPL state space, simple setup and planner based on request. **/
  void preSolve(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                const planning_scene::PlanningSceneConstPtr& ps, planning_interface::MotionPlanRequest request);

  /** \brief Solve the planning problem, directly pass joint positions for start and goal in this minimal example.
   *
   * \todo Should this be part of the preSolve setup?
   * */
  bool solve(const Eigen::Ref<const Eigen::VectorXd>& start_joint_positions,
             const Eigen::Ref<const Eigen::VectorXd>& goal_joint_positions, double allowed_planning_time);

  /** \brief Simplify solution and convert from OMPL to generic format.
   *
   * @todo too much Eigen copy operations.
   **/
  void postSolve();

  /** \brief Check if the constraints are satisfied along the path `solution_path_`. **/
  bool checkSolution();

  /** \brief Get solution in a generic OMPL and ROS independent format. **/
  std::vector<Eigen::VectorXd> getSolutionPath()
  {
    return solution_path_;
  }

private:
  /** \brief Create OMPL planner based on the name of the planner (planner_id). **/
  ob::PlannerPtr selectAndCreatePlanner(const std::string& planner_id,
                                        ob::ConstrainedSpaceInformationPtr space_info) const;

  std::shared_ptr<ob::RealVectorStateSpace> state_space_;
  std::shared_ptr<BaseConstraint> constraints_;

  std::shared_ptr<ob::ProjectedStateSpace> constrained_state_space_;
  std::shared_ptr<ob::ConstrainedSpaceInformation> constrained_state_space_info_;
  std::shared_ptr<og::SimpleSetup> simple_setup_;
  std::shared_ptr<ob::Planner> planner_;

  // @todo, this can be a ROS specific format
  std::vector<Eigen::VectorXd> solution_path_;
  std::size_t num_dofs_; /**< initialized in preSolve method based on robot model. */
};
}  // namespace elion