#ifndef ELION_CONVERSIONS_H
#define ELION_CONVERSIONS_H

#include <Eigen/Geometry>

namespace elion
{
/** Inverse of the Conversion matrix from roll-pitch-yaw velocity to angular velocity.
 *
 * w = B(rpy) * rpy_dot
 * w = angular velocity, B = matrix returned by this function,
 * rpy = roll-pitch-yaw angles
 * rpy_dot = roll-pitch-yaw time derivatives.
 *
 * This function directly calculates B^-1
 * and contains a singularity for ry = +/- pi / 2
 *
 * from:
 * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
 *
 * */
Eigen::Matrix3d angularVelocityToRPYRates(double rx, double ry);

}  // namespace elion

#endif  // ELION_CONVERSIONS_H