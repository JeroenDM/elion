#include "elion_planner/collision_checking.h"

namespace elion
{
bool MoveItStateValidityChecker::isValid(const ob::State* state) const
{
  return true;
}
}  // namespace elion