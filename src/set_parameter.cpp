#include <chrono>
#include <vector>
#include <string>
#include <utility>
#include <limits>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/node_utils.hpp"
#include "node_parameter_handler/set_parameter.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace node_parameter_handler
{

SetParameterCustom::SetParameterCustom(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("node_parameter_handler", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating node_parameter_handler server");
}

SetParameterCustom::~SetParameterCustom()
{
}

nav2_util::CallbackReturn
SetParameterCustom::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring node_parameter_handler interface");

  // declare parameter
  declare_parameter("service_name", rclcpp::ParameterValue("/local_costmap/local_costmap/set_parameters_atomically"));
  get_parameter("service_name", srv_name_);

  // create client
  client_ = node->create_client<rcl_interfaces::srv::SetParametersAtomically>(srv_name_);

  // create dynamic parameter handler
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SetParameterCustom::dynamicParametersCallback, this, _1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SetParameterCustom::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SetParameterCustom::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SetParameterCustom::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  client_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SetParameterCustom::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult
SetParameterCustom::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    // If we are trying to change the parameter of a plugin we can just skip it at this point
    // as they handle parameter changes themselves and don't need to lock the mutex
    if (name.find('.') != std::string::npos) {
      continue;
    }

    if (!dynamic_params_lock_.try_lock()) {
      RCLCPP_WARN(
        get_logger(),
        "Unable to dynamically change Parameters while the node is currently running");
      result.successful = false;
      result.reason =
        "Unable to dynamically change Parameters while the node is currently running";
      return result;
    }

    if (type == ParameterType::PARAMETER_STRING) {
      if (name == "service_name") {
        change_service_name(parameter.as_string());
      }
    }
    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

void SetParameterCustom::change_service_name(const std::string & new_srv_name)
{
  // Ensure we are in a state that allows this operation
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(get_logger(), "Cannot change service name when node is not active");
    return;
  }

  if (client_) {
    client_.reset();
  }

  auto node = shared_from_this();
  
  // update service name and create new client
  srv_name_ = new_srv_name;
  client_ = node->create_client<rcl_interfaces::srv::SetParametersAtomically>(srv_name_);
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(node_parameter_handler::SetParameterCustom)
