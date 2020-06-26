#include "elion_planner/collision_checking.h"

#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

namespace elion
{
MoveItCollisionChecker::MoveItCollisionChecker(const ob::SpaceInformationPtr& si,
                                               robot_model::RobotModelConstPtr& robot_model, const std::string& group,
                                               const planning_scene::PlanningSceneConstPtr& ps)
  : ob::StateValidityChecker(si), ps_(ps), group_name_(group), tss_(robot_model)
{
  collision_request_simple_.group_name = group_name_;
}
bool MoveItCollisionChecker::isValid(const ob::State* state) const
{
  auto &&q = *state->as<ompl::base::ConstrainedStateSpace::StateType>();
  collision_detection::CollisionResult res;
  moveit::core::RobotState* robot_state = tss_.getStateStorage();
  robot_state->setJointGroupPositions(group_name_, q);
  ps_->checkCollision(collision_request_simple_, res, *robot_state);
  return !res.collision;
}
}  // namespace elion