#pragma once

#include <Eigen/Geometry>

namespace elion
{
/** \brief Inverse of the Conversion matrix from roll-pitch-yaw velocity to
 * angular velocity.
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

/** \brief Conversion matrix to go from angular velocity in the world frame to
 * angle axis equivalent.
 *
 * Modern Robotics, page 161.
 *
 * @todo figure out how this works, and can I avoid the matrix inverse?
 * */
Eigen::Matrix3d angularVelocityToAngleAxis(double angle, Eigen::Vector3d axis);

/** Get the sign of a number.
 * https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 * */
template <typename T>
int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

}  // namespace elion