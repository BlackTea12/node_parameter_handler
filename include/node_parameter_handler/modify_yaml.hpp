#ifndef MODIFY_YAML_CUSTOM_HPP_
#define MODIFY_YAML_CUSTOM_HPP_

#include <memory>
#include <string>
#include <thread>
#include <mutex>

#include "nav2_util/lifecycle_node.hpp"
#include "node_parameter_handler/file_reader.hpp"

namespace node_parameter_handler
{
/**
 * @class node_parameter_handler::ModifyYaml
 */
class ModifyYaml : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for node_parameter_handler::ModifyYaml
   * @param options Additional options to control creation of the node.
   */
  explicit ModifyYaml(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for node_parameter_handler::ModifyYaml
   */
  ~ModifyYaml();

protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

private:
  ///@brief modifer pointer
  std::unique_ptr<Modifier> modifier_;
  ///@brief root package and parameter folder name
  std::string root_pkg_name_, root_param_folder_name_;
  ///@brief search and saving options
  bool search_deep_, save_install_, save_home_;
};

}  // namespace node_parameter_handler

#endif  // MODIFY_YAML_CUSTOM_HPP_
