#pragma once

#include <iostream>
#include <limits>
#include <vector>
#include <moveit_msgs/Constraints.h>

namespace elion
{
/** \brief Represents upper and lower bound on a scalar value (double).
 *
 * Equality constraints can be represented by setting
 * the upper bound and lower bound almost equal.
 * I assume that it is better to not have them exactly equal
 * for numerical reasons. Not sure.
 * **/
struct Bounds
{
  double lower, upper;

  /** \brief Distance to region inside bounds
   *
   * Distance of a given value outside the bounds,
   * zero inside the bounds.
   *
   * Creates a penalty function that looks like this:
   *
   *  \         /
   *   \       /
   *    \_____/
   * (how does ascii art work??)
   * */
  double distance(double value) const;
  double derivative(double value) const;
};

}  // namespace elion

/** Pretty printing of bounds. **/
std::ostream& operator<<(std::ostream& os, const elion::Bounds& bound);