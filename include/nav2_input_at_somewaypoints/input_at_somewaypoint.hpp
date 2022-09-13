#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_SOMEWAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_SOMEWAYPOINT_HPP_
#pragma once

#include <string>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"

namespace nav2_waypoint_follower
{

/**
 * @brief Simple plugin based on WaypointTaskExecutor, lets robot to wait for a
 *        user input at waypoint arrival.
 */
class InputAtSomeWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
/**
 * @brief Construct a new Input At Waypoint Arrival object
 *
 */
  InputAtSomeWaypoint();

  /**
   * @brief Destroy the Input At Waypoint Arrival object
   *
   */
  ~InputAtSomeWaypoint();

  /**
   * @brief declares and loads parameters used
   * @param parent parent node
   * @param plugin_name name of plugin
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);

  /**
   * @brief Processor
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return if task execution failed
   */
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

protected:
  /**
   * @brief Processor callback
   * @param msg Empty message
   */
  void Cb(const std_msgs::msg::Bool::SharedPtr msg);

  bool input_received_;
  bool is_enabled_;
  std::vector<int64_t> index;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  rclcpp::Clock::SharedPtr clock_;
  std::mutex mutex_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__INPUT_AT_WAYPOINT_HPP_