/** OMPL specific wrapper of MoveIt's collision
 * checking interface.
 * 
 * TODO!
 * */
#ifndef ELION_COLLISION_CHECKING_H
#define ELION_COLLISION_CHECKING_H

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
  MoveItCollisionChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si)
  {
  }

  virtual bool isValid(const ob::State* state) const;
};
}  // namespace elion

#endif  // ELION_COLLISION_CHECKING_H