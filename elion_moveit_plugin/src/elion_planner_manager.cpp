#include <string>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include "elion_moveit_plugin/elion_planning_context.h"

namespace elion
{
class ElionPlannerManager : public planning_interface::PlannerManager
{
public:
  ElionPlannerManager() : planning_interface::PlannerManager()
  {
  }

  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override
  {
    for (const std::string& group : model->getJointModelGroupNames())
    {
      planning_contexts_[group] =
          ElionPlanningContextPtr(new ElionPlanningContext("chomp_planning_context", group, model));
    }
    return true;
  }

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
  {
    return true;
  }

  std::string getDescription() const override
  {
    return "COMPL";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("ProjectedStateSpace");
    algs.push_back("AtlasStateSpace");
    algs.push_back("TangentBundleStateSpace");
  }

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const moveit_msgs::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const override

  {
    if (req.group_name.empty())
    {
      ROS_ERROR("No group specified to plan for");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      ROS_ERROR("No planning scene supplied as input");
      error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    // (jeroendm) I don't understand yet why we need the two lines below
    planning_scene::PlanningScenePtr ps = planning_scene->diff();
    ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create(), true);

    const ElionPlanningContextPtr& context = planning_contexts_.at(req.group_name);
    context->setPlanningScene(ps);
    context->setMotionPlanRequest(req);

    return context;
  }

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override
  {
  }

private:
  std::map<std::string, ElionPlanningContextPtr> planning_contexts_;
};
}  // namespace compl_interface

CLASS_LOADER_REGISTER_CLASS(elion::ElionPlannerManager, planning_interface::PlannerManager);