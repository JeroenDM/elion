#include "elion_planner/bounds.h"

#include <cmath>

namespace elion
{

inline double quadratic_penalty(double error)
{
  return 0.5 * error * error;
}

double Bounds::distance(double value) const
{
  if (value <= lower)
    return quadratic_penalty(value - lower);
  else if (value >= upper)
    return quadratic_penalty(value - upper);
  else
    return 0.0;
}

double Bounds::derivative(double value) const
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