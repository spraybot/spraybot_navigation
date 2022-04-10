/*********************************************************************
 *
 * Author: Vrushali Patil (soundarya.patil@gmail.com)
 *
 *********************************************************************/

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
    node_, name_ + ".turn_radius", rclcpp::ParameterValue(
      1.0));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".transform_tolerance", transform_tolerance);
  node_->get_parameter(name_ + ".turn_radius", turn_radius_);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
}

void ArcPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type ArcPlanner",
    name_.c_str());
}

void ArcPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type ARcPlanner",
    name_.c_str());
}

void ArcPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type ARcPlanner",
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
  global_path.header.stamp = transformed_goal.header.stamp = node_->now();
  global_path.header.frame_id = transformed_goal.header.frame_id = global_frame_;

  const auto & start_pos = transformed_start.pose.position;
  const auto & goal_pos = transformed_goal.pose.position;

  double chord_len = std::hypot(
    goal_pos.x - start_pos.x,
    goal_pos.y - start_pos.y);

  if (chord_len > 2.0 * turn_radius_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Computing a circular path to the goal is not possible");
    RCLCPP_ERROR(
      node_->get_logger(), "The chord length is %f and the set radius is %f.", chord_len,
      turn_radius_);
    return global_path;
  }

  // Angle subtended at the center of the circle
  double theta = 2 * asin((chord_len / 2) / turn_radius_);
  // Angle subtended by the start and goal pose
  double alpha =
    atan2(
    (goal_pos.y - start_pos.y),
    (goal_pos.x - start_pos.x));
  // Sum of all angles of a triangle, theta + (alpha + phi)*2 = 180
  double phi = ((M_PI - theta) / 2) - alpha;
  double goal_yaw = poseToYaw(goal);
  double start_yaw = poseToYaw(start);
  // Flag to decide if clockwise or anti-clockwise path should be taken
  bool clockwise = false;

  if ((goal_yaw >= -M_PI && goal_yaw <= -M_PI / 2) || (goal_yaw >= M_PI / 2 &&
    goal_yaw <= M_PI))
  {
    clockwise = !clockwise;
  }

  int number_of_waypoints = theta / interpolation_resolution_;
  if ((!clockwise && goal_pos.y > start_pos.y) ||
    (clockwise && goal_pos.y < start_pos.y))
  {
    for (int i = 0; i < number_of_waypoints; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = start_pos.x + (turn_radius_ * cos(phi)) - turn_radius_ *
        cos(
        phi + i * interpolation_resolution_);
      pose.pose.position.y = start_pos.y - (turn_radius_ * sin(phi)) + turn_radius_ *
        sin(
        phi + i * interpolation_resolution_);
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }
  } else if ((!clockwise && goal_pos.y < start_pos.y) ||   // NOLINT
    (clockwise && goal_pos.y > start_pos.y))
  {
    for (int i = number_of_waypoints; i > 0; --i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = goal_pos.x - (turn_radius_ * cos(phi)) + turn_radius_ *
        cos(
        phi + i * interpolation_resolution_);
      pose.pose.position.y = goal_pos.y + (turn_radius_ * sin(phi)) - turn_radius_ *
        sin(
        phi + i * interpolation_resolution_);

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
