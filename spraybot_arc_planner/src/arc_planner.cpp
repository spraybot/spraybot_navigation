/*
 *  Author: Vrushali Patil (soundarya.patil@gmail.com)
 *
 */

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include "spraybot_arc_planner/arc_planner.hpp"

namespace spraybot_arc_planner
{

void ArcPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  double transform_tolerance = 0.1;

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".transform_tolerance", transform_tolerance);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
}

void ArcPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void ArcPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void ArcPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

bool ArcPlanner::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in transformPose: %s", ex.what());
  }
  return false;
}

nav_msgs::msg::Path ArcPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Lambda to transform a PoseStamped to global frame
  auto transformToGlobalPose = [&](const geometry_msgs::msg::PoseStamped & pose) {
      geometry_msgs::msg::PoseStamped transformed_pose;
      transformPose(global_frame_, pose, transformed_pose);
      return transformed_pose;
    };

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  auto poseToYaw = [](const geometry_msgs::msg::PoseStamped p)
    {
      tf2::Quaternion q(
        p.pose.orientation.x,
        p.pose.orientation.y,
        p.pose.orientation.z,
        p.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
    };

  geometry_msgs::msg::PoseStamped transformed_goal = transformToGlobalPose(goal);
  geometry_msgs::msg::PoseStamped transformed_start = start;

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  double distance = std::hypot(
    transformed_goal.pose.position.x - transformed_start.pose.position.x,
    transformed_goal.pose.position.y - transformed_start.pose.position.y);

  double radius = 0.762;

  if (distance > 2 * radius) {
    RCLCPP_ERROR(
      node_->get_logger(), "Computing a circular path to the goal is not possible, too small turning radius, the distance between start and goal waypoint is %f and the set radius is %f.", distance,
      radius);
  }
  double theta = 2 * asin(distance / 2 / radius);
  double alpha =
    atan2(
    (transformed_goal.pose.position.y - transformed_start.pose.position.y),
    (transformed_goal.pose.position.x - transformed_start.pose.position.x));
  double phi = ((M_PI - theta) / 2) - alpha;
  double yaw = poseToYaw(start);
  double c = cos(phi);
  double s = sin(phi);
  int total_number_of_loop = theta / interpolation_resolution_;
  int dir {0};
  if (((yaw >= (-M_PI)) && (yaw <= (-M_PI / 2))) or ((yaw >= M_PI / 2) && (yaw <= M_PI))) {
    dir = 1;
  }

  if (((dir == 1) && (transformed_goal.pose.position.y > transformed_start.pose.position.y)) ||
    ((dir == 0) && (transformed_goal.pose.position.y < transformed_start.pose.position.y)))
  {
    for (int i = 0; i < total_number_of_loop; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = transformed_start.pose.position.x + (radius * c) - radius * cos(
        phi + i * interpolation_resolution_);
      pose.pose.position.y = transformed_start.pose.position.y - (radius * s) + radius * sin(
        phi + i * interpolation_resolution_);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }
  } else if (((dir == 1) &&
    (transformed_goal.pose.position.y < transformed_start.pose.position.y)) ||
    ((dir == 0) && (transformed_goal.pose.position.y > transformed_start.pose.position.y)))
  {
    for (int i = total_number_of_loop; i > 0; --i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = transformed_goal.pose.position.x - (radius * c) + radius * cos(
        phi + i * interpolation_resolution_);
      pose.pose.position.y = transformed_goal.pose.position.y + (radius * s) - radius * sin(
        phi + i * interpolation_resolution_);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }
  }

  global_path.poses.push_back(transformed_goal);
  return global_path;
}

}  // namespace spraybot_arc_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spraybot_arc_planner::ArcPlanner, nav2_core::GlobalPlanner)