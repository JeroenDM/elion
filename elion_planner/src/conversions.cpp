#include "elion_planner/conversions.h"

#include <Eigen/Geometry>

#include <ros/console.h>

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace elion
{
Eigen::Matrix3d angularVelocityToRPYRates(double rx, double ry)
{
  double TOLERANCE{ 1e-9 }; /* TODO what tolerance to use here? */
  Eigen::Matrix3d E;
  double cosy{ std::cos(ry) };

  // check for singular case
  if (std::abs(cosy) < TOLERANCE)
  {
    ROS_ERROR_STREAM("Singularity in orientation path constraints.");
  }

  double cosx{ std::cos(rx) };
  double sinx{ std::sin(rx) };
  double siny{ std::sin(ry) };
  E << 1, sinx * siny / cosy, -cosx * siny / cosy, 0, cosx, sinx, 0, -sinx / cosy, cosx / cosy;
  return E;
}

Eigen::Matrix3d angularVelocityToAngleAxis(double angle, Eigen::Vector3d axis)
{
  Eigen::Matrix3d E;

  // create short variable for readability of complex expression
  double t{ std::abs(angle) };
  Eigen::Vector3d r{ axis * sign(angle) };
  // double t{ angle };
  // Eigen::Vector3d r{ axis };
  Eigen::Matrix3d r_skew;
  r_skew << 0, -r[2], r[1], r[2], 0, -r[0], -r[1], r[0], 0;

  // double A, B;
  // A = (1 - std::cos(t)) / (t * t);
  // B = (t - std::sin(t)) / (t * t * t);

  // unclear whether the first sign should be positive or negative
  // E = Eigen::Matrix3d::Identity() + A * r_skew + B * r_skew * r_skew;
  // return E.inverse();

  double C;
  C = (1 - 0.5 * t * std::sin(t) / (1 - cos(t)));

  E = Eigen::Matrix3d::Identity() + 0.5 * r_skew + r_skew * r_skew / (t * t) * C;
  return E;
}
}  // namespace elion