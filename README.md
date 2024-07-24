# node_parameter_handler
Handling specific node's parameter by service


### how to use

    ros2 launch node_parameter_handler example_launch.py

`service_name` must be some node's **srv/SetParametersAtomically** service type. By directly making a client by this name, we expect to change parameters.


### same when entering command in terminal

(example)

    ros2 service call /local_costmap/local_costmap/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'inflation_layer.inflation_radius', value: {type: 3, double_value: 1.0}}]}"


Here, type value is defined in range of 1 to 9. 
1. bool_value
2. integer_value
3. double_value
4. string_value
5. byte_array_value
6. bool_array_value
7. integer_array_value
8. double_array_value
9. string_array_value

`name` is your parameter in string. It is in the another node without the node itself name.