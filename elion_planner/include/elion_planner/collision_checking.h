/** OMPL specific wrapper of MoveIt's collision
 * checking interface.
 *
 * TODO!
 * */
#ifndef ELION_COLLISION_CHECKING_H
#define ELION_COLLISION_CHECKING_H

#include <string>

#include <elion_planner/threadsafe_state_storage.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ompl/base/StateValidityChecker.h>

namespace elion
{
namespace ob = ompl::base;

/** \Brief interface to collision checking in planning scene
 *
 * The class is explicitly called collision checker because that is it's only
 * responsibility. In the existing OMPL interface in MoveIt, the state validity
 * checker has two extra responsibilities:
 *
 * - Check path constraints -> this is now part of the OMPL state space
 * - Check joint limits? -> these are now bounds on the OMPL state space
 * ( although the latter is not correctly implemented yet... )
 *
 * */
class MoveItCollisionChecker : public ob::StateValidityChecker
{
public:
  MoveItCollisionChecker(const ob::SpaceInformationPtr& si, robot_model::RobotModelConstPtr& robot_model,
                         const std::string& group, const planning_scene::PlanningSceneConstPtr& ps);
  virtual bool isValid(const ob::State* state) const;

private:
  std::string group_name_;
  TSStateStorage tss_;
  collision_detection::CollisionRequest collision_request_simple_;
  const planning_scene::PlanningSceneConstPtr& ps_;
};
}  // namespace elion

#endif  // ELION_COLLISION_CHECKING_H