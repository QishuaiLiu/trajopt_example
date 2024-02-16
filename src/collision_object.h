#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene/planning_scene.h>

namespace CollisionObject
{
class scene
{
  scene() = default;
  bool addObject();
}
}  // namespace CollisionObject
