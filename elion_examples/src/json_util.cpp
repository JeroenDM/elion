#include "elion_examples/json_util.h"

#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace elion
{
std::vector<double> jsonToVector(const Json::Value& json_value)
{
  std::vector<double> vec;
  if (!json_value.isArray())
  {
    ROS_ERROR_STREAM("Trying to read json string as vector, but it's not...");
  }
  else
  {
    for (auto val : json_value)
    {
      vec.push_back(val.asDouble());
    }
  }
  return vec;
}

geometry_msgs::Pose jsonToPoseMsg(const Json::Value& json_value)
{
  std::vector<double> position{ jsonToVector(json_value["xyz"]) };
  std::vector<double> orientation{ jsonToVector(json_value["rpy"]) };

  geometry_msgs::Pose pose;
  if (position.size() != 3)
  {
    ROS_ERROR_STREAM("Pose xyz (nominal position) should have length 3, not " << position.size());
    return pose;
  }

  if (orientation.size() != 3)
  {
    ROS_ERROR_STREAM("Pose rpy (nominal orientation) should have length 3, not " << orientation.size());
    return pose;
  }

  tf2::Quaternion quat;
  quat.setRPY(orientation[0], orientation[1], orientation[2]);
  pose.orientation = tf2::toMsg(quat);
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];

  return pose;
}

moveit_msgs::PositionConstraint readPositionConstraint(const Json::Value& con, const std::string fixed_frame)
{
  const std::string ee_link = con.get("link_name", {}).asString();
  std::vector<double> dimensions{ jsonToVector(con["dims"]) };
  std::vector<double> nominal_position{ jsonToVector(con["xyz"]) };
  std::vector<double> nominal_orientation{ jsonToVector(con["rpy"]) };

  moveit_msgs::PositionConstraint position_constraint;
  if (dimensions.size() != 3)
  {
    ROS_ERROR_STREAM("Position constraint dimensions should have length 3, not " << dimensions.size());
    return position_constraint;
  }

  if (nominal_position.size() != 3)
  {
    ROS_ERROR_STREAM("Position constraint xyz (nominal position) should have length 3, not "
                     << nominal_position.size());
    return position_constraint;
  }

  if (nominal_orientation.size() != 3)
  {
    ROS_ERROR_STREAM("Position constraint rpy (nominal orientation) should "
                     "have length 3, not "
                     << nominal_orientation.size());
    return position_constraint;
  }

  shape_msgs::SolidPrimitive box_constraint;
  box_constraint.type = shape_msgs::SolidPrimitive::BOX;
  box_constraint.dimensions = dimensions; /* use -1 to indicate no constraints. */

  geometry_msgs::Pose box_pose;
  box_pose.position.x = nominal_position[0];
  box_pose.position.y = nominal_position[1];
  box_pose.position.z = nominal_position[2];

  tf2::Quaternion nominal_orientation_quat;
  nominal_orientation_quat.setRPY(nominal_orientation[0], nominal_orientation[1], nominal_orientation[2]);
  box_pose.orientation = tf2::toMsg(nominal_orientation_quat);

  position_constraint.header.frame_id = fixed_frame;
  position_constraint.link_name = ee_link;
  position_constraint.constraint_region.primitives.push_back(box_constraint);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);

  return position_constraint;
}

moveit_msgs::OrientationConstraint readOrientationConstraints(const Json::Value& con, const std::string fixed_frame)
{
  const std::string ee_link = con.get("link_name", {}).asString();
  std::vector<double> nominal_orientation{ jsonToVector(con["rpy"]) };
  std::vector<double> tolerance{ jsonToVector(con["tolerance"]) };

  moveit_msgs::OrientationConstraint orientation_constraint;
  if (tolerance.size() != 3)
  {
    ROS_ERROR_STREAM("Orientation constraint tolerance should have length 3, not " << tolerance.size());
    return orientation_constraint;
  }

  if (nominal_orientation.size() != 3)
  {
    ROS_ERROR_STREAM("Orientation constraint rpy (nominal orientation) should "
                     "have length 3, not "
                     << nominal_orientation.size());
    return orientation_constraint;
  }

  orientation_constraint.header.frame_id = fixed_frame;
  orientation_constraint.link_name = ee_link;

  tf2::Quaternion nominal_orientation_quat;
  nominal_orientation_quat.setRPY(nominal_orientation[0], nominal_orientation[1], nominal_orientation[2]);
  orientation_constraint.orientation = tf2::toMsg(nominal_orientation_quat);

  orientation_constraint.absolute_x_axis_tolerance = tolerance[0];
  orientation_constraint.absolute_y_axis_tolerance = tolerance[1];
  orientation_constraint.absolute_z_axis_tolerance = tolerance[2];

  return orientation_constraint;
}

moveit_msgs::Constraints readPathConstraints(const Json::Value& json_constraints, const std::string fixed_frame)
{
  moveit_msgs::Constraints path_constraints;
  for (auto constraint : json_constraints)
  {
    std::string constraint_type{ constraint.get("type", {}).asString() };
    ROS_INFO_STREAM("Reading constraints of type: " << constraint_type);
    if (constraint_type == "position")
    {
      path_constraints.position_constraints.push_back(readPositionConstraint(constraint, fixed_frame));
    }
    else if (constraint_type == "angle_axis")
    {
      path_constraints.orientation_constraints.push_back(readOrientationConstraints(constraint, fixed_frame));
      path_constraints.name = "AngleAxis";
    }
    else if (constraint_type == "roll_pitch_yaw")
    {
      path_constraints.orientation_constraints.push_back(readOrientationConstraints(constraint, fixed_frame));
      path_constraints.name = "RollPitchYaw";
    }
    else
    {
      ROS_ERROR_STREAM("Unknown type of constraints: " << constraint_type);
    }
  }
  return path_constraints;
}

void readAndAddObstacles(const Json::Value& json_collision_objects,
                         moveit::planning_interface::PlanningSceneInterface& psi, const std::string& planning_frame)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  for (auto object : json_collision_objects)
  {
    std::string name{ object.get("name", {}).asString() };
    std::vector<double> dimensions{ jsonToVector(object["dims"]) };

    if (dimensions.size() != 3)
    {
      ROS_ERROR_STREAM("Collision object cubiod dimensions should have length 3, not " << dimensions.size());
    }

    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame;
    collision_object.id = name;
    collision_object.operation = collision_object.ADD;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = dimensions;

    geometry_msgs::Pose box_pose;
    box_pose = jsonToPoseMsg(object);

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);

    collision_objects.push_back(collision_object);
  }
  if (collision_objects.size() > 0)
  {
    psi.addCollisionObjects(collision_objects);
  }
}

bool isJsonFileName(const std::string& input)
{
  static const std::string ext{ ".json" };
  std::size_t found = input.find(ext);
  return found != std::string::npos;
}

}  // namespace elion
