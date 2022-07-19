#include "nav2_input_at_somewaypoints/input_at_somewaypoint.hpp"

#include <string>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{

using std::placeholders::_1;

InputAtSomeWaypoint::InputAtSomeWaypoint()
: input_received_(false),
  is_enabled_(true),
  index(0)
{
}

InputAtSomeWaypoint::~InputAtSomeWaypoint()
{
}

void InputAtSomeWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();

  if (!node) {
    throw std::runtime_error{"Failed to lock node in input at waypoint plugin!"};
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  std::string input_topic;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".index",
    rclcpp::ParameterValue(2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".input_topic",
    rclcpp::ParameterValue("input_at_somewaypoint/input"));
  node->get_parameter(plugin_name + ".index", index);
  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".input_topic", input_topic);

  RCLCPP_INFO(
    logger_, "InputAtSomeWaypoint: Subscribing to input topic %s.", input_topic.c_str());
  subscription_ = node->create_subscription<std_msgs::msg::Empty>(
    input_topic, 1, std::bind(&InputAtSomeWaypoint::Cb, this, _1));
}

void InputAtSomeWaypoint::Cb(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  input_received_ = true;
}

bool InputAtSomeWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }

  input_received_ = false;

  rclcpp::Rate r(50);
  bool input_received = false;
  if(curr_waypoint_index != index){
      return true;
  }
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      input_received = input_received_;
    }

    if (input_received) {
      return true;
    }

    r.sleep();
  }

  RCLCPP_WARN(
    logger_, "Unable to get external input at wp %i. Moving on.", curr_waypoint_index);
  return false;
}

}  // namespace nav2_waypoint_follower

PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::InputAtSomeWaypoint,
  nav2_core::WaypointTaskExecutor)