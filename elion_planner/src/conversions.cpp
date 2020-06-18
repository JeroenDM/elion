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

}  // namespace elion