#ifndef SET_PARAMETER_CUSTOM_HPP_
#define SET_PARAMETER_CUSTOM_HPP_

#include <memory>
#include <string>
#include <thread>
#include <mutex>

#include "nav2_util/lifecycle_node.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

namespace node_parameter_handler
{

/**
 * @class node_parameter_handler::SetParameterCustom
 */
class SetParameterCustom : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for node_parameter_handler::SetParameterCustom
   * @param options Additional options to control creation of the node.
   */
  explicit SetParameterCustom(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for node_parameter_handler::SetParameterCustom
   */
  ~SetParameterCustom();

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
  /**
   * @brief Change service name process
   * @param new_srv_name new service name to change
   */
  void change_service_name(const std::string &new_srv_name);

  ///@brief client, to set parameter from another node atomically 
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr client_;
  ///@brief service name
  std::string srv_name_;
};

}  // namespace node_parameter_handler

#endif  // SET_PARAMETER_CUSTOM_HPP_
