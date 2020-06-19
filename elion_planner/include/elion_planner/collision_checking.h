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

// define this class:
class MoveItStateValidityChecker : public ob::StateValidityChecker
{
public:
  MoveItStateValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si)
  {
  }

  virtual bool isValid(const ob::State* state) const;
};
}  // namespace elion

#endif  // ELION_COLLISION_CHECKING_H