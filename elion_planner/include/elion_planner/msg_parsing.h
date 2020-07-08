#pragma once

#include <vector>
#include <moveit_msgs/Constraints.h>

#include "elion_planner/bounds.h"

namespace elion
{
/** Use Infinity unconstrained parameters
 *
 * This makes it easier to write generic code.
 * Otherwise we would have to leave out the bounds for specific
 * position or orientation values and writing the constraint
 * evaluation would be much more complex, within my range
 * of solution I can imagine (jeroendm).
 *
 * TODO: we could also use the workspace size, this could be
 * more appropriate than infinity.
 * */
const double INF = std::numeric_limits<double>::infinity();

/** Extract position constraints from the MoveIt message.
 *
 * Assumes there is a single primitive of type box.
 * Only the dimensions of this box are used here.
 * These are bounds on the deviation of the end-effector from
 * the desired position given as the position of the box in the field
 * constraint_regions.primitive_poses[0].position.
 *
 * TODO: also use the link name in future?
 * Now we assume the constraints are for the end-effector link.
 * */
std::vector<Bounds> positionConstraintMsgToBoundVector(moveit_msgs::PositionConstraint pos_con);

/** Extract orientation constraints from the MoveIt message
 *
 * These bounds are assumed to be centered around the nominal orientation / desired orientation
 * given in the field "orientation" in the message.
 * These bounds are therefore bounds on the orientation error between the desired orientation
 * and the current orientation of the end-effector.
 *
 * The three bounds x, y, and z, can be applied to different parameterizations of the rotation error.
 * (Roll, pithc, and yaw or exponential coordinates or something else.)
 *
 * */
std::vector<Bounds> orientationConstraintMsgToBoundVector(moveit_msgs::OrientationConstraint ori_con);

}  // namespace elion