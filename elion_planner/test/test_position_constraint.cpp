#include <elion_planner/constraint.h>

#include <gtest/gtest.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/Constraints.h>

TEST(PositionConstraints, BasicTest)
{
  EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}