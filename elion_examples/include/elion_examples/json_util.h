#ifndef ELION_EXAMPLES_JSON_UTIL_H
#define ELION_EXAMPLES_JSON_UTIL_H

#include <vector>
#include <jsoncpp/json/json.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace elion
{
std::vector<double> jsonToVector(const Json::Value& json_value);

geometry_msgs::Pose jsonToPoseMsg(const Json::Value& json_value);

moveit_msgs::PositionConstraint readPositionConstraint(const Json::Value& con, const std::string fixed_frame);

moveit_msgs::OrientationConstraint readOrientationConstraints(const Json::Value& con, const std::string fixed_frame);

/** \brief Parse orientation constraints from json file read with the library jsoncpp.
 *
 * The error type for orientation constraints is specified in a hacky way as the
 * name of the constraitns.
 * For the strings identifying the types, see elion_planner/constraints.h in the
 * namespace OrientationErrorType.
 *
 * */
moveit_msgs::Constraints readPathConstraints(const Json::Value& json_constraints, const std::string fixed_frame);

void readAndAddObstacles(const Json::Value& json_collision_objects,
                         moveit::planning_interface::PlanningSceneInterface& psi, const std::string& planning_frame);

bool isJsonFileName(const std::string& input);

}  // namespace elion

#endif  // ELION_EXAMPLES_JSON_UTIL_H
