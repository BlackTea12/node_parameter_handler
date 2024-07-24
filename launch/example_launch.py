import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.actions import ComposableNodeContainer
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
  bringup_dir = get_package_share_directory('node_parameter_handler')
  params_dir = os.path.join(bringup_dir, 'params')

  log_level = LaunchConfiguration('log_level')
  use_sim_time = LaunchConfiguration('use_sim_time')
  namespace = LaunchConfiguration('namespace')
  use_namespace = LaunchConfiguration('use_namespace')
  autostart = LaunchConfiguration('autostart')
  default_params_file = LaunchConfiguration('default_params_file')
  
  param_substitutions = {
    'use_sim_time': use_sim_time,
    'use_namespace': use_namespace,
    'autostart': autostart
  }

  configured_params = ParameterFile(
    RewrittenYaml(
      source_file=default_params_file,
      root_key=namespace,
      param_rewrites=param_substitutions,
      convert_types=True),
    allow_substs=True)
  
  declare_namespace_cmd = DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Top-level namespace')
  declare_use_namespace_cmd = DeclareLaunchArgument(
    'use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
  declare_autostart_cmd = DeclareLaunchArgument(
    'autostart', default_value='true',
    description='Automatically startup the nav2 stack')
  declare_log_level_cmd = DeclareLaunchArgument(
    'log_level', default_value='info',
    description='log level')
  declare_default_params_file_cmd = DeclareLaunchArgument(
    'default_params_file',
    default_value=os.path.join(params_dir, 'example.yaml'))

  lifecycle_nodes = [ 'node_parameter_handler' ]

  container = ComposableNodeContainer(
    name='node_parameter_handler_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_isolated',
    output='screen',
    arguments=['--ros-args', '--log-level', log_level],
    parameters=[configured_params],
    composable_node_descriptions=[
      ComposableNode(
        name='node_parameter_handler',
        package='node_parameter_handler',
        plugin='node_parameter_handler::SetParameterCustom',
        parameters=[configured_params]),
      ComposableNode(
          package='nav2_lifecycle_manager',
          plugin='nav2_lifecycle_manager::LifecycleManager',
          name='lifecycle_manager_navigation',
          parameters=[{'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': lifecycle_nodes}])
    ]
  )

  namespace_action = GroupAction(
    actions=[
      PushRosNamespace(
        condition=IfCondition(use_namespace),
        namespace=namespace),
        container
    ]
  )

  # load_nodes = GroupAction(
  #   actions=[
  #     PushRosNamespace(
  #       condition=IfCondition(use_namespace),
  #       namespace=namespace),
  #     ComposableNode(
  #       package='node_parameter_handler',
  #       plugin='node_parameter_handler::SetParameterCustom',
  #       parameters=[configured_params]),
  #     Node(
  #       package='rclcpp_components',
  #       executable='component_container_isolated',
  #       name='node_parameter_handler_container',
  #       output='screen',
  #       arguments=['--ros-args', '--log-level', log_level],
  #       parameters=[configured_params]),
  #   ]
  # )

  # Create the launch description and populate
  ld = LaunchDescription()

  # Set environment variables
  ld.add_action(declare_log_level_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_default_params_file_cmd)
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(namespace_action)

  return ld
