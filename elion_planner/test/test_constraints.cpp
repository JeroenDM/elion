#include <elion_planner/constraint.h>

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/Constraints.h>

// Names to read values from the parameter server
const std::string GROUP_PARAM = "group";
const std::string BASE_LINK = "base_link";
const std::string EE_LINK = "ee_link";

// Robot description almost always called "robot_description" and therefore hardcoded below
const std::string ROBOT_DESCRIPTION = "robot_description";

moveit_msgs::PositionConstraint createPositionConstraint(std::string& base_link, std::string& ee_link_name)
{
  shape_msgs::SolidPrimitive box_constraint;
  box_constraint.type = shape_msgs::SolidPrimitive::BOX;
  box_constraint.dimensions = { 0.05, 0.4, 0.05 }; /* use -1 to indicate no constraints. */

  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.9;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.2;
  box_pose.orientation.w = 1.0;

  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = base_link;
  position_constraint.link_name = ee_link_name;
  position_constraint.constraint_region.primitives.push_back(box_constraint);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);

  return position_constraint;
}

moveit_msgs::OrientationConstraint createOrientationConstraint(std::string& base_link, std::string& ee_link_name)
{
  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = base_link;
  orientation_constraint.link_name = ee_link_name;
  orientation_constraint.orientation.w = 1.0;
  orientation_constraint.absolute_x_axis_tolerance = 0.1;
  orientation_constraint.absolute_y_axis_tolerance = 0.1;
  orientation_constraint.absolute_z_axis_tolerance = -1.0;
  return orientation_constraint;
}

class TestConstraints : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle nh;
    if (nh.getParam(GROUP_PARAM, group_name_) && nh.getParam(BASE_LINK, base_link_name_) &&
        nh.getParam(EE_LINK, ee_link_name_))
    {
      robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(ROBOT_DESCRIPTION);
      robot_model_ = robot_model_loader_->getModel();
      num_dofs_ = robot_model_->getJointModelGroup(group_name_)->getVariableCount();
    }
    else
    {
      ROS_ERROR_STREAM("Failed to read " << GROUP_PARAM << " from parameter server.");
    }
  }
  void TearDown() override
  {
  }

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  std::string group_name_;
  std::string base_link_name_;
  std::string ee_link_name_;
  std::string robot_description_name_;
  std::size_t num_dofs_;
  std::shared_ptr<elion::BaseConstraint> constraint_;
};

TEST_F(TestConstraints, InitPositionConstraint)
{
  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.position_constraints.push_back(createPositionConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<elion::PositionConstraint>(robot_model_, group_name_, constraint_msgs, num_dofs_);
  constraint_->init(constraint_msgs);
}

TEST_F(TestConstraints, InitAngleAxisConstraint)
{
  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.orientation_constraints.push_back(createOrientationConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<elion::AngleAxisConstraint>(robot_model_, group_name_, constraint_msgs, num_dofs_);
  constraint_->init(constraint_msgs);
}

TEST_F(TestConstraints, InitRPYConstraint)
{
  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.orientation_constraints.push_back(createOrientationConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<elion::RPYConstraints>(robot_model_, group_name_, constraint_msgs, num_dofs_);
  constraint_->init(constraint_msgs);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elion_planner_test_constraints");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}