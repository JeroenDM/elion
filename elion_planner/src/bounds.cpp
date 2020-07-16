#include "elion_planner/bounds.h"

namespace elion
{
double Bounds::distance(double value) const
{
  if (value <= lower)
    return lower - value;
  else if (value >= upper)
    return value - upper;
  else
    return 0.0;
}
}  // namespace elion

// TODO: not sure if this has to be outside namespace
std::ostream& operator<<(std::ostream& os, const elion::Bounds& bound)
{
  os << "Bounds: ";
  os << "( " << bound.lower;
  os << ", " << bound.upper << " )";
  return os;
}