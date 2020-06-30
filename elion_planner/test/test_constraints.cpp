#include <elion_planner/constraint.h>

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

// Names to read values from the parameter server
const std::string GROUP_PARAM = "group";
const std::string BASE_LINK = "base_link";
const std::string EE_LINK = "ee_link";

const int NUM_RANDOM_TESTS{ 10 };

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

Eigen::Vector3d rotationToRPY(const Eigen::Matrix3d& r)
{
  Eigen::Vector3d xyz = r.eulerAngles(0, 1, 2);
  // in MoveIt some wrapping is done sometimes, but I'm not sure why or when to apply it yet
  // probably to convert the eigen convention: [0:pi]x[-pi:pi]x[-pi:pi] to a MoveIt convention?
  // but it looks the same to me
  // it does not seem to work in some cases.
  // xyz(0) = std::min(fabs(xyz(0)), M_PI - fabs(xyz(0)));
  // xyz(1) = std::min(fabs(xyz(1)), M_PI - fabs(xyz(1)));
  // xyz(2) = std::min(fabs(xyz(2)), M_PI - fabs(xyz(2)));
  return xyz;
}

Eigen::Vector3d poseToRPY(const Eigen::Isometry3d& p)
{
  return rotationToRPY(p.rotation());
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
      robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
      joint_model_group_ = robot_state_->getJointModelGroup(group_name_);
      num_dofs_ = joint_model_group_->getVariableCount();
    }
    else
    {
      ROS_ERROR_STREAM("Failed to read " << GROUP_PARAM << " from parameter server.");
    }
  }
  void TearDown() override
  {
  }

  const Eigen::Isometry3d fk(const Eigen::VectorXd q) const
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getGlobalLinkTransform(ee_link_name_);
  }

  Eigen::VectorXd getRandomState()
  {
    robot_state_->setToRandomPositions(joint_model_group_);
    Eigen::VectorXd q;
    robot_state_->copyJointGroupPositions(joint_model_group_, q);
    return q;
  }

  Eigen::MatrixXd numericalJacobianPosition(const Eigen::VectorXd q)
  {
    const double h{ 1e-6 }; /* step size for numerical derivation */

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, num_dofs_);

    // helper matrix for differentiation.
    Eigen::MatrixXd Ih = h * Eigen::MatrixXd::Identity(num_dofs_, num_dofs_);

    for (std::size_t dim{ 0 }; dim < num_dofs_; ++dim)
    {
      Eigen::Vector3d pos = fk(q).translation();
      Eigen::Vector3d pos_plus_h = fk(q + Ih.col(dim)).translation();
      Eigen::Vector3d col = (pos_plus_h - pos) / h;
      J.col(dim) = col;
    }
    return J;
  }

  Eigen::MatrixXd numericalJacobianRPY(const Eigen::VectorXd q)
  {
    const double h{ 1e-6 }; /* step size for numerical derivation */

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, num_dofs_);

    // helper matrix for differentiation.
    Eigen::MatrixXd Ih = h * Eigen::MatrixXd::Identity(num_dofs_, num_dofs_);

    for (std::size_t dim{ 0 }; dim < num_dofs_; ++dim)
    {
      auto rpy = poseToRPY(fk(q));
      auto rpy_plus_h = poseToRPY(fk(q + Ih.col(dim)));

      // // check for large jumps around singularities
      // if ((rpy_plus_h - rpy).norm() > 1e-3)
      // {
      //   std::cout << "!!! Probably rpy angles are close to singularity." << std::endl;
      // }

      Eigen::Vector3d col = (rpy_plus_h - rpy) / h;
      J.col(dim) = col;
    }
    return J;
  }

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
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

  constraint_ = std::make_shared<elion::PositionConstraint>(robot_model_, group_name_, num_dofs_);
  constraint_->init(constraint_msgs);
}

TEST_F(TestConstraints, PositionConstraintJacobian)
{
  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.position_constraints.push_back(createPositionConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<elion::PositionConstraint>(robot_model_, group_name_, num_dofs_);
  constraint_->init(constraint_msgs);

  double total_error{ 999.9 };
  const double ERROR_TOLERANCE{ 1e-4 }; /** High tolerance because of high finite difference error. **/

  for (int i{ 0 }; i < NUM_RANDOM_TESTS; ++i)
  {
    auto q = getRandomState();
    auto J_exact = constraint_->calcErrorJacobian(q);
    auto J_finite_diff = numericalJacobianPosition(q);

    // std::cout << "Analytical jacobian: \n";
    // std::cout << J_exact << std::endl;
    // std::cout << "Finite difference jacobian: \n";
    // std::cout << J_finite_diff << std::endl;

    total_error = (J_exact - J_finite_diff).lpNorm<1>();
    EXPECT_LT(total_error, ERROR_TOLERANCE);
  }
}

TEST_F(TestConstraints, PositionConstraintOMPLCheck)
{
  auto state_space = std::make_shared<ompl::base::RealVectorStateSpace>(num_dofs_);
  ompl::base::RealVectorBounds bounds(num_dofs_);

  // get joint limits from the joint model group
  auto joint_limits = joint_model_group_->getActiveJointModelsBounds();
  EXPECT_EQ(joint_limits.size(), num_dofs_);
  for (std::size_t i{ 0 }; i < num_dofs_; ++i)
  {
    EXPECT_EQ(joint_limits[i]->size(), 1);
    bounds.setLow(i, joint_limits[i]->at(0).min_position_);
    bounds.setHigh(i, joint_limits[i]->at(0).max_position_);
  }

  state_space->setBounds(bounds);

  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.position_constraints.push_back(createPositionConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<elion::PositionConstraint>(robot_model_, group_name_, num_dofs_);
  constraint_->init(constraint_msgs);

  auto constrained_state_space = std::make_shared<ompl::base::ProjectedStateSpace>(state_space, constraint_);

  // constrained_state_space->constrainedSanityChecks(ompl::base::ConstrainedStateSpace::SanityChecks::CONSTRAINED_STATESPACE_JACOBIAN);
  // I copied the Jacobian testing code from the sanityCheck below to investigate the problem

  for (int i{ 0 }; i < NUM_RANDOM_TESTS; ++i)
  {
    auto* s1 = constrained_state_space->allocState()->as<ompl::base::ProjectedStateSpace::StateType>();
    ompl::base::StateSamplerPtr ss = constrained_state_space->allocStateSampler();

    ss->sampleUniform(s1);

    Eigen::MatrixXd j_a(3, num_dofs_), j_n(3, num_dofs_);
    constraint_->jacobian(*s1, j_a);              // Implemented in elion approximation
    constraint_->Constraint::jacobian(*s1, j_n);  // Numerical approximation

    // eliminate issues with the numerical jacobian at the bounds of the constraints
    Eigen::Vector3d fun;
    constraint_->function(*s1, fun);
    for (std::size_t i{ 0 }; i < constraint_->getCoDimension(); ++i)
    {
      if (std::abs(fun[i]) < constraint_->getTolerance())
      {
        j_n.row(i) = Eigen::VectorXd::Zero(num_dofs_);
      }
    }

    // std::cout << "Test for joint values: \n";
    // std::cout << s1->transpose() << std::endl;
    // std::cout << "End-effector position: \n";
    // std::cout << fk(*s1).translation().transpose() << std::endl;
    // std::cout << "Analytical jacobian: \n";
    // std::cout << j_a << std::endl;
    // std::cout << "Finite difference jacobian: \n";
    // std::cout << j_n << std::endl;

    EXPECT_LT((j_a - j_n).norm(), constraint_->getTolerance());
  }
}

TEST_F(TestConstraints, InitAngleAxisConstraint)
{
  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.orientation_constraints.push_back(createOrientationConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<elion::AngleAxisConstraint>(robot_model_, group_name_, num_dofs_);
  constraint_->init(constraint_msgs);
}

TEST_F(TestConstraints, InitRPYConstraint)
{
  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.orientation_constraints.push_back(createOrientationConstraint(base_link_name_, ee_link_name_));

  constraint_ = std::make_shared<elion::RPYConstraints>(robot_model_, group_name_, num_dofs_);
  constraint_->init(constraint_msgs);
}

// TEST_F(TestConstraints, RPYConstraintJacobian)
// {
//   moveit_msgs::Constraints constraint_msgs;
//   constraint_msgs.orientation_constraints.push_back(createOrientationConstraint(base_link_name_, ee_link_name_));

//   constraint_ = std::make_shared<elion::RPYConstraints>(robot_model_, group_name_, num_dofs_);
//   constraint_->init(constraint_msgs);

//   double total_error{ 999.9 };
//   const double ERROR_TOLERANCE{ 1e-3 }; /** High tolerance because of high finite difference error?? **/

//   for (int i{ 0 }; i < NUM_RANDOM_TESTS; ++i)
//   {
//     auto q = getRandomState();
//     auto J_exact = constraint_->calcErrorJacobian(q);
//     auto J_finite_diff = numericalJacobianRPY(q);

//     std::cout << "Test for joint values: \n";
//     std::cout << q.transpose() << std::endl;
//     std::cout << "RPY anlges for test point: \n";
//     std::cout << poseToRPY(fk(q)).transpose() << std::endl;
//     std::cout << "Analytical jacobian: \n";
//     std::cout << J_exact << std::endl;
//     std::cout << "Finite difference jacobian: \n";
//     std::cout << J_finite_diff << std::endl;

//     // auto J_error = J_exact - J_finite_diff;

//     // for (std::size_t col{ 0 }; col < J_error.cols(); ++col)
//     // {
//     //   for (std::size_t row{ 0 }; row < J_error.rows(); ++row)
//     //   {
//     //     EXPECT_LT(std::abs(J_error(row, col)), ERROR_TOLERANCE);
//     //   }
//     // }

//     total_error = (J_exact - J_finite_diff).lpNorm<1>();
//     EXPECT_LT(total_error, ERROR_TOLERANCE);
//   }
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elion_planner_test_constraints");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}