#include <elion_planner/conversions.h>

#include <Eigen/Dense>
#include <gtest/gtest.h>

using namespace Eigen;

/** Conversion of the angular velocity vector omega
 * to roll pitch yaw angles.
 * (intrinsic XYZ rotation angles.)
 *
 * Test the three unit vector angular velocities.
 * These should result in exactly the unit vector rpys.
 * At least because there are centered around Identity rotation,
 * which is why we pass two 0.0 to angularVelocityToRPYRates
 * */
TEST(AngularVelocityToRPYRates, unitX)
{
  Vector3d omega, rpy;
  omega = Vector3d::UnitX();
  rpy = elion::angularVelocityToRPYRates(0.0, 0.0) * omega;
  EXPECT_DOUBLE_EQ(rpy[0], omega[0]);
  EXPECT_DOUBLE_EQ(rpy[1], omega[1]);
  EXPECT_DOUBLE_EQ(rpy[2], omega[2]);
}

TEST(AngularVelocityToRPYRates, unitY)
{
  Vector3d omega, rpy;
  omega = Vector3d::UnitY();
  rpy = elion::angularVelocityToRPYRates(0.0, 0.0) * omega;
  EXPECT_DOUBLE_EQ(rpy[0], omega[0]);
  EXPECT_DOUBLE_EQ(rpy[1], omega[1]);
  EXPECT_DOUBLE_EQ(rpy[2], omega[2]);
}

TEST(AngularVelocityToRPYRates, unitZ)
{
  Vector3d omega, rpy;
  omega = Vector3d::UnitZ();
  rpy = elion::angularVelocityToRPYRates(0.0, 0.0) * omega;
  EXPECT_DOUBLE_EQ(rpy[0], omega[0]);
  EXPECT_DOUBLE_EQ(rpy[1], omega[1]);
  EXPECT_DOUBLE_EQ(rpy[2], omega[2]);
}

/** given the first column of the rotation matrix is 1, 0, 0,
 * the x component of the rotation should remain unchanged.
 *
 * TODO avoid singularities in the randomly generated vectors.
 * */
TEST(AngularVelocityToRPYRates, xComponent)
{
  Vector3d omega, rpy;
  for (int i{ 0 }; i < 10; ++i)
  {
    omega = Vector3d::Random();
    rpy = elion::angularVelocityToRPYRates(0.0, 0.0) * omega;
    EXPECT_DOUBLE_EQ(rpy[0], omega[0]);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}