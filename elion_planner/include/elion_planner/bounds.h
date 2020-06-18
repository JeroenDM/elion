#ifndef ELION_BOUNDS_H
#define ELION_BOUNDS_H

#include <iostream>
#include <limits>
#include <vector>
#include <moveit_msgs/Constraints.h>

namespace elion
{
/** \Brief Represents bounds on a scalar value (double).
 *
 * Equality constraints can be represented by setting
 * the upper bound and lower bound almost equal.
 * I assume that it is better to not have them exactly equal
 * for numerical reasons. Not sure.
 * **/
struct Bounds
{
  double lower, upper;

  /** \Brief Distance to region inside bounds
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
};

}  // namespace elion

/** \Brief Pretty printing of bounds. **/
std::ostream& operator<<(std::ostream& os, const elion::Bounds& bound);

#endif  // ELION_BOUNDS_H