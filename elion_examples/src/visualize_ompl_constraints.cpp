#include <moveit/ompl_interface/detail/ompl_constraints.h>

#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/util/Exception.h>

// // parameters for fanuc, kuka, ...
const std::string PLANNING_GROUP{ "manipulator" };
const std::string FIXED_FRAME{ "base_link" };
const std::string EE_LINK_NAME{ "tool0" };
const double BOX_SIZE[]{ 0.9, 0.0, 0.2 };
const double ROTATION_TOL{ 0.1 };

// // parameters for panda
// const std::string PLANNING_GROUP{ "panda_arm" };
// const std::string FIXED_FRAME{ "panda_link0" };
// const std::string EE_LINK_NAME{ "panda_link8" };
// const double BOX_SIZE[]{ 0.3, 0.0, 0.5 };
// const double ROTATION_TOL{ 0.5 };

namespace ompl
{
namespace base
{
typedef std::shared_ptr<RealVectorStateSpace> RealVectorStateSpacePtr;
}
}

namespace rvt = rviz_visual_tools;

/*****************************************
 * DECLARATIONS
 * ***************************************/

class Robot
{
public:
  Robot(const std::string planning_group, const std::string& base_link, const std::string& ee_link);
  void publishState(moveit_visual_tools::MoveItVisualTools& mvt, const Eigen::VectorXd& q);
  void publishState(moveit_visual_tools::MoveItVisualTools& mvt, const std::vector<double>& q);

  const moveit::core::RobotModelPtr getRobotModel()
  {
    return robot_model_;
  }

  const moveit::core::JointBoundsVector& getJointLimits()
  {
    return joint_model_group_->getActiveJointModelsBounds();
  }

  Eigen::Isometry3d fk(const std::vector<double>& joint_values)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
    return robot_state_->getGlobalLinkTransform(ee_link_);
  }

  Eigen::Isometry3d fk()
  {
    robot_state_->setToDefaultValues();
    return robot_state_->getGlobalLinkTransform(ee_link_);
  }

  std::string base_link_;
  std::string ee_link_;
  std::size_t num_dofs_;

private:
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

void publishPositionConstraintMsg(moveit_visual_tools::MoveItVisualTools& mvt, moveit_msgs::PositionConstraint pos_con);
/** \brief Helper function to create a specific position constraint. **/
moveit_msgs::PositionConstraint createPositionConstraintMsg(const std::string& base_link,
                                                            const std::string& ee_link_name);

moveit_msgs::OrientationConstraint createOrientationConstraint(const std::string& base_link,
                                                               const std::string& ee_link_name,
                                                               geometry_msgs::Quaternion& nominal_orientation);

void publishJacobianArrow(moveit_visual_tools::MoveItVisualTools& mvt, const Eigen::MatrixXd& jac,
                          const Eigen::VectorXd& q, const Eigen::Isometry3d& ee_pose);

bool project(ompl_interface::PositionConstraintPtr& constraint, Eigen::Ref<Eigen::VectorXd> x)
{
  // Newton's method
  unsigned int iter = 0;
  double norm = 0;
  Eigen::VectorXd f(constraint->getCoDimension());
  Eigen::MatrixXd j(constraint->getCoDimension(), constraint->getAmbientDimension());

  const double squaredTolerance = constraint->getTolerance() * constraint->getTolerance();

  constraint->function(x, f);
  while ((norm = f.squaredNorm()) > squaredTolerance && iter++ < constraint->getMaxIterations())
  {
    // std::cout << iter << "," << f.squaredNorm() << "\n";

    constraint->jacobian(x, j);
    x -= j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
    constraint->function(x, f);
  }

  return norm < squaredTolerance;
}

// copied from  https://github.com/Jmeyer1292/opw_kinematics
inline void harmonizeTowardZero(double* qs, const int num_dofs)
{
  const static double pi = M_PI;
  const static double two_pi = 2.0 * M_PI;

  for (int i = 0; i < num_dofs; i++)  // TODO: Unroll manually?
  {
    if (qs[i] > pi)
      qs[i] -= two_pi;
    else if (qs[i] < -pi)
      qs[i] += two_pi;
  }
}

/** Try to add or substract multiples of 2*pi to move a state inside the joint limits. **/
void applyJointLimits(double* q, const ompl::base::RealVectorStateSpacePtr& rvss)
{
  const static double two_pi = 2.0 * M_PI;
  auto bounds = rvss->getBounds();
  for (unsigned int i{ 0 }; i < rvss->getDimension(); ++i)
  {
    if (q[i] > bounds.high.at(i))
    {
      if ((q[i] - two_pi) >= bounds.low.at(i))
      {
        q[i] -= two_pi;
      }
    }
    else if (q[i] < bounds.low.at(i))
    {
      if ((q[i] + two_pi) <= bounds.high.at(i))
      {
        q[i] += two_pi;
      }
    }
  }
}

void tryDifferentMaxIterations(const ompl::base::ConstraintPtr& constraint,
                               const ompl::base::RealVectorStateSpacePtr rvss,
                               const ompl::base::ConstrainedStateSpacePtr css)
{
  ompl::base::StateSamplerPtr sampler = rvss->allocStateSampler();
  bool proj_success{ false };
  int success_counter{ 0 };
  int enforce_bounds_success_counter{ 0 };
  int clipped_success_counter{ 0 };
  const int num_runs{ 1 };

  auto* s1 = css->allocState()->as<ompl::base::ConstrainedStateSpace::StateType>();
  auto* s2 = css->allocState()->as<ompl::base::ConstrainedStateSpace::StateType>();
  // Eigen::Vector3d pos_error;
  Eigen::VectorXd pos_error(constraint->getCoDimension());
  ROS_INFO_STREAM("Constraints of shape " << constraint->getAmbientDimension() << " by "
                                          << constraint->getCoDimension());
  double squared_tolerance = constraint->getTolerance() * constraint->getTolerance();

  for (unsigned int max_iters{ 1 }; max_iters <= 100; max_iters += 1)
  {
    constraint->setMaxIterations(max_iters);

    for (int i{ 0 }; i < num_runs; ++i)
    {
      // use unconstrained sampling
      sampler->sampleUniform(s1->getState());

      // check if projection works out
      proj_success = constraint->project(s1);
      if (proj_success)
      {
        success_counter++;
      }
      else
      {
        continue;
      }
      Eigen::VectorXd q_projected = *s1;

      // clip to joint limits and check constraints
      s2 = css->cloneState(s1)->as<ompl::base::ConstrainedStateSpace::StateType>();
      css->enforceBounds(s2);
      Eigen::VectorXd q_clipped = *s2;
      constraint->function(q_clipped, pos_error);
      double tolerance_error = pos_error.squaredNorm();
      if (tolerance_error <= squared_tolerance && proj_success)
      {
        enforce_bounds_success_counter++;
      }
      else
      {
        // std::cout << "ENFORCING BOUNDS FAILED\n";
        // std::cout << (q_projected - q_clipped).transpose() << "\n";
      }
      // edit state to move multiples of 2 * pi back inside -pi, pi
      // harmonizeTowardZero(css->getValueAddressAtIndex(s1, 0), rvss->getDimension());
      applyJointLimits(css->getValueAddressAtIndex(s1, 0), rvss);
      Eigen::VectorXd q_harmonized = *s1;
      // clip to joint limits and check constraints again
      css->enforceBounds(s1);
      Eigen::VectorXd q_clipped_harmonized = *s1;
      constraint->function(q_clipped_harmonized, pos_error);
      tolerance_error = pos_error.squaredNorm();
      if (tolerance_error <= squared_tolerance && proj_success)
      {
        clipped_success_counter++;
      }

      // print different states
      // std::cout << "------------------------------------------\n";
      // std::cout << q_projected.transpose() << "\n";
      // std::cout << q_harmonized.transpose() << "\n";
      // std::cout << q_clipped.transpose() << "\n";
      // std::cout << q_clipped_harmonized.transpose() << "\n";
      // if (!proj_success)
      //   std::cout << "FAILED\n";
    }

    // or print counters to create graphs
    std::cout << max_iters << "," << success_counter << "," << num_runs;
    std::cout << "," << enforce_bounds_success_counter;
    std::cout << "," << clipped_success_counter << "\n";
    success_counter = 0;
    enforce_bounds_success_counter = 0;
    clipped_success_counter = 0;
  }

  css->freeState(s1);
  css->freeState(s2);
}

/*****************************************
 * MAIN
 * ***************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_ompl_constraints");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  // Planning scene setup
  // ^^^^^^^^^^^^^^^^^^^^
  Robot robot(PLANNING_GROUP, FIXED_FRAME, EE_LINK_NAME);

  // Visualization
  // ^^^^^^^^^^^^^
  moveit_visual_tools::MoveItVisualTools visual_tools(FIXED_FRAME, "/visualization_marker_array");
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.loadRemoteControl();
  visual_tools.loadMarkerPub(true);
  // visual_tools.loadTrajectoryPub();
  visual_tools.deleteAllMarkers();
  ros::Duration(1.0).sleep();

  // fixed location to show information
  geometry_msgs::Pose text_pose;
  text_pose.orientation.w = 1.0;
  text_pose.position.z = 1.0;

  // // Create constraint state space
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  auto state_space = std::make_shared<ompl::base::RealVectorStateSpace>(robot.num_dofs_);

  //  apply joint limits to real vector state space
  ompl::base::RealVectorBounds bounds(robot.num_dofs_);
  auto joint_limits = robot.getJointLimits();
  for (std::size_t i{ 0 }; i < robot.num_dofs_; ++i)
  {
    bounds.setLow(i, joint_limits[i]->at(0).min_position_);
    bounds.setHigh(i, joint_limits[i]->at(0).max_position_);
    std::cout << "Joint limit " << i << ": " << bounds.low[i] << " -> " << bounds.high[i] << "\n";
  }
  state_space->setBounds(bounds);

  // create the position constraint model
  // auto pos_con_msg = createPositionConstraintMsg(robot.base_link_, robot.ee_link_);
  // publishPositionConstraintMsg(visual_tools, pos_con_msg);
  // ros::Duration(0.1).sleep();  // give the publisher some time
  // moveit_msgs::Constraints constraint_msgs;
  // constraint_msgs.position_constraints.push_back(pos_con_msg);

  // auto pos_con =
  //     std::make_shared<ompl_interface::PositionConstraint>(robot.getRobotModel(), PLANNING_GROUP, robot.num_dofs_);
  // pos_con->init(constraint_msgs);

  // orientation constraints
  Eigen::Isometry3d ee_pose = robot.fk();
  geometry_msgs::Quaternion ee_orientation;
  tf::quaternionEigenToMsg(Eigen::Quaterniond(ee_pose.rotation()), ee_orientation);
  auto ori_con_msg = createOrientationConstraint(robot.base_link_, robot.ee_link_, ee_orientation);
  moveit_msgs::Constraints constraint_msgs;
  constraint_msgs.orientation_constraints.push_back(ori_con_msg);
  ompl_interface::BaseConstraintPtr ori_con =
      std::make_shared<ompl_interface::OrientationConstraint>(robot.getRobotModel(), PLANNING_GROUP, robot.num_dofs_);
  ori_con->init(constraint_msgs);

  auto ompl_constraint =
      std::make_shared<ompl_interface::JointLimitConstraint>(robot.getRobotModel(), PLANNING_GROUP, robot.num_dofs_);

  ompl::base::ConstraintIntersectionPtr ci;
  ci.reset(new ompl::base::ConstraintIntersection(robot.num_dofs_, { ori_con, ompl_constraint }));

  // and finally create the actual constrained state space
  // auto constrained_state_space = std::make_shared<ompl::base::ProjectedStateSpace>(state_space, pos_con);
  auto constrained_state_space = std::make_shared<ompl::base::ProjectedStateSpace>(state_space, ci);
  auto constrained_state_space_info =
      std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space);

  // // Investigate the uniform sampler
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  tryDifferentMaxIterations(ci, state_space, constrained_state_space);

  ros::shutdown();
  return 0;
}

/*****************************************
 * DEFINITIONS
 * ***************************************/

Robot::Robot(const std::string planning_group, const std::string& base_link, const std::string& ee_link)
  : base_link_(base_link), ee_link_(ee_link)
{
  robot_model_loader::RobotModelLoaderPtr robot_model_loader =
      std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  robot_model_ = robot_model_loader->getModel();
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();
  joint_model_group_ = robot_state_->getJointModelGroup(planning_group);
  num_dofs_ = joint_model_group_->getVariableCount();
}

void Robot::publishState(moveit_visual_tools::MoveItVisualTools& mvt, const Eigen::VectorXd& q)
{
  std::vector<double> joint_values(q.size());
  for (std::size_t i{ 0 }; i < joint_values.size(); ++i)
    joint_values[i] = q[i];
  mvt.publishRobotState(joint_values, joint_model_group_);
  mvt.trigger();
}

void Robot::publishState(moveit_visual_tools::MoveItVisualTools& mvt, const std::vector<double>& q)
{
  mvt.publishRobotState(q, joint_model_group_);
  mvt.trigger();
}

void publishPositionConstraintMsg(moveit_visual_tools::MoveItVisualTools& mvt, moveit_msgs::PositionConstraint pos_con)
{
  auto dims = pos_con.constraint_region.primitives.at(0).dimensions;
  // if dim = -1 -> make it large (could be workspace size in the future.)
  const double UNBOUNDED_SIZE{ 3.0 };
  for (auto& dim : dims)
  {
    if (dim == -1.0)
      dim = UNBOUNDED_SIZE;
  }
  bool did_publish = mvt.publishCuboid(pos_con.constraint_region.primitive_poses.at(0), dims.at(0), dims.at(1),
                                       dims.at(2), rviz_visual_tools::GREEN);
  mvt.trigger();
  ros::Duration(0.5).sleep();
}

/** \brief Helper function to create a specific position constraint. **/
moveit_msgs::PositionConstraint createPositionConstraintMsg(const std::string& base_link,
                                                            const std::string& ee_link_name)
{
  shape_msgs::SolidPrimitive box_constraint;
  box_constraint.type = shape_msgs::SolidPrimitive::BOX;
  box_constraint.dimensions = { 0.05, 0.4, 0.05 }; /* use -1 to indicate no constraints. */

  geometry_msgs::Pose box_pose;
  box_pose.position.x = BOX_SIZE[0];
  box_pose.position.y = BOX_SIZE[1];
  box_pose.position.z = BOX_SIZE[2];
  box_pose.orientation.w = 1.0;

  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = base_link;
  position_constraint.link_name = ee_link_name;
  position_constraint.constraint_region.primitives.push_back(box_constraint);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);

  return position_constraint;
}

moveit_msgs::OrientationConstraint createOrientationConstraint(const std::string& base_link,
                                                               const std::string& ee_link_name,
                                                               geometry_msgs::Quaternion& nominal_orientation)
{
  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = base_link;
  orientation_constraint.link_name = ee_link_name;
  orientation_constraint.orientation = nominal_orientation;
  orientation_constraint.absolute_x_axis_tolerance = ROTATION_TOL;
  orientation_constraint.absolute_y_axis_tolerance = ROTATION_TOL;
  orientation_constraint.absolute_z_axis_tolerance = ROTATION_TOL;
  return orientation_constraint;
}

void publishJacobianArrow(moveit_visual_tools::MoveItVisualTools& mvt, const Eigen::MatrixXd& jac,
                          const Eigen::VectorXd& q, const Eigen::Isometry3d& ee_pose)
{
  Eigen::Vector3d direction = jac * q;
  geometry_msgs::Point ee_origin;
  ee_origin.x = ee_pose.translation()[0];
  ee_origin.y = ee_pose.translation()[1];
  ee_origin.z = ee_pose.translation()[2];
  geometry_msgs::Point arrow_tip(ee_origin);
  ee_origin.x += direction[0];
  ee_origin.y += direction[1];
  ee_origin.z += direction[2];

  mvt.publishArrow(arrow_tip, ee_origin, rvt::RED, rvt::LARGE);
  mvt.trigger();
  ros::Duration(0.1).sleep();
}