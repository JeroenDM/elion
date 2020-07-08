#include <elion_planner/conversions.h>

#include <Eigen/Dense>
#include <gtest/gtest.h>

using namespace Eigen;

const int NUM_RANDOM_TESTS{ 10 };

TEST(AngularVelocityToRPYRates, numericalDerivative)
{
  const double h{ 1e-8 };  // interval with for forward finite difference

  for (int i{ 0 }; i < NUM_RANDOM_TESTS; ++i)
  {
    // create random orientation and angular velocity
    Matrix3d rotation{ Quaterniond::UnitRandom() };
    auto omega{ Vector3d::Random().normalized() };

    // use fixed values to make test repeatable
    // Matrix3d rotation{ AngleAxisd(0.3, Vector3d::UnitX()) * AngleAxisd(0.2, Vector3d::UnitY()) *
    //                    AngleAxisd(0.1, Vector3d::UnitZ()) };
    // Vector3d omega;
    // omega << 0.3, 0.0, 0.0;

    // apply angular velocity to the orientation for a time interval h
    Matrix3d rotation_plus_h = AngleAxisd(h, omega) * rotation;

    // calculate euler rates rates using forward difference approximation
    Vector3d rpy = rotation.eulerAngles(0, 1, 2);
    Vector3d rpy_dot_approx = (rotation_plus_h.eulerAngles(0, 1, 2) - rpy) / h;

    Vector3d rpy_dot_exact = elion::angularVelocityToRPYRates(rpy[0], rpy[1]) * omega;

    const double ERROR_TOLERANCE{ 1e-5 };
    EXPECT_LT(std::abs(rpy_dot_exact[0] - rpy_dot_approx[0]), ERROR_TOLERANCE);
    EXPECT_LT(std::abs(rpy_dot_exact[1] - rpy_dot_approx[1]), ERROR_TOLERANCE);
    EXPECT_LT(std::abs(rpy_dot_exact[2] - rpy_dot_approx[2]), ERROR_TOLERANCE);
  }
}

/** test conversion from angular velocity to angle axis velocity
 *
 * TODO: the numerical derivative in this tests seems to cause an issue.
 * The test does not work for a angular velocity vector that is not normalized.
 * (But it should work, because this vector can have an arbitrary norm.)
 * I suspect the issue is related to the calculation of the numerical derivative
 * based on forward finite differences, but I'm not sure.
 *
 * */
TEST(AngularVelocityToAARates, numericalDerivative)
{
  const double h{ 1e-8 };  // interval with for forward finite difference

  for (int i{ 0 }; i < NUM_RANDOM_TESTS; ++i)
  {
    // create random orientation and angular velocity
    Matrix3d rotation{ Quaterniond::UnitRandom() };
    // auto omega{ Vector3d::Random()};
    auto omega{ Vector3d::Random().normalized() };
    // omega *= 1.5;

    // use fixed values to make test repeatable
    // Matrix3d rotation{ AngleAxisd(1.0, Vector3d(1, 1, 1).normalized()) };
    // auto rotation = Matrix3d::Identity();
    // Matrix3d rotation{ AngleAxisd(M_PI_2, Vector3d::UnitX()) };
    // Matrix3d rotation{ AngleAxisd(1.0, Vector3d(std::sqrt(2) / 2, std::sqrt(2) / 2, 0.0)) };
    // Vector3d omega;
    // omega << 1.0, 0.0, 0.0;
    // omega << 0.0, 0.0, 0.1;

    // apply angular velocity to the orientation for a time interval h
    Vector3d normalized_axis = omega.normalized();
    Matrix3d rotation_plus_h = AngleAxisd(h, normalized_axis) * rotation;

    // calculate euler rates rates using forward difference approximation
    Eigen::AngleAxisd aa(rotation);
    Eigen::AngleAxisd aa_plus_h(rotation_plus_h);
    Vector3d aa_dot_approx = (aa_plus_h.axis() * aa_plus_h.angle() - aa.axis() * aa.angle()) / h;

    Vector3d aa_dot_exact = elion::angularVelocityToAngleAxis(aa.angle(), aa.axis()) * omega;

    std::cout << "\nOmega: " << omega.transpose() << "\n";
    std::cout << "Angle: " << aa.angle() << " | " << aa.axis().transpose() << "\n\n";
    std::cout << "Exact: " << aa_dot_exact.transpose() << std::endl;
    std::cout << "Approx: " << aa_dot_approx.transpose() << std::endl;

    // EXPECT_DOUBLE_EQ(aa_dot_exact[0], aa_dot_approx[0]);
    // EXPECT_DOUBLE_EQ(aa_dot_exact[1], aa_dot_approx[1]);
    // EXPECT_DOUBLE_EQ(aa_dot_exact[2], aa_dot_approx[2]);

    const double ERROR_TOLERANCE{ 1e-5 };
    EXPECT_LT(std::abs(aa_dot_exact[0] - aa_dot_approx[0]), ERROR_TOLERANCE);
    EXPECT_LT(std::abs(aa_dot_exact[1] - aa_dot_approx[1]), ERROR_TOLERANCE);
    EXPECT_LT(std::abs(aa_dot_exact[2] - aa_dot_approx[2]), ERROR_TOLERANCE);
  }
}

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