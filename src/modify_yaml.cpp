#include <chrono>
#include <vector>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/node_utils.hpp"
#include "node_parameter_handler/modify_yaml.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace node_parameter_handler
{

ModifyYaml::ModifyYaml(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("modify_yaml_handler", "", options), modifier_(nullptr), root_pkg_name_(""),
  root_param_folder_name_(""), search_deep_(true), save_install_(true), save_home_(true)
{
  RCLCPP_INFO(get_logger(), "Creating modify_yaml_handler server");

  // create class object
  modifier_ = std::make_unique<Modifier>(&root_pkg_name_, &root_param_folder_name_);
}

ModifyYaml::~ModifyYaml()
{
}

nav2_util::CallbackReturn
ModifyYaml::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring modify_yaml_handler interface");

  // declare parameter
  declare_parameter("pkg_name", rclcpp::ParameterValue("node_parameter_handler"));
  declare_parameter("params_folder", rclcpp::ParameterValue("params"));
  declare_parameter("search_deep", rclcpp::ParameterValue(true));
  declare_parameter("save_install", rclcpp::ParameterValue(true));
  declare_parameter("save_home", rclcpp::ParameterValue(true));
  get_parameter("pkg_name", root_pkg_name_);
  get_parameter("params_folder", root_param_folder_name_);
  get_parameter("search_deep", search_deep_);
  get_parameter("save_install", save_install_);
  get_parameter("save_home", save_home_);

  // create dynamic parameter handler
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&ModifyYaml::dynamicParametersCallback, this, _1));

  modifier_->start(search_deep_, save_install_, save_home_);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ModifyYaml::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ModifyYaml::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ModifyYaml::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
ModifyYaml::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult
ModifyYaml::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
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
      if (name == "pkg_name") {
        root_pkg_name_ = parameter.as_string();
      } else if (name == "params_folder") {
        root_param_folder_name_ = parameter.as_string();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "search_deep") {
        search_deep_ = parameter.as_bool();
      } else if (name == "save_install") {
        save_install_ = parameter.as_bool();
      } else if (name == "save_home") {
        save_home_ = parameter.as_bool();
      }
    }
    dynamic_params_lock_.unlock();
  }

  result.successful = true;
  return result;
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(node_parameter_handler::ModifyYaml)
