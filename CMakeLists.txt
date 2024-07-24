cmake_minimum_required(VERSION 3.8)
project(node_parameter_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_common REQUIRED)
find_package(ament_index_cpp REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name ${PROJECT_NAME})

add_executable(${executable_name}
  src/main_node.cpp
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED 
  src/set_parameter.cpp
)

set(library_direct_name ${PROJECT_NAME}_core_direct)

add_library(${library_direct_name} SHARED 
  src/file_reader.cpp
  src/modify_yaml.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  nav2_common
  nav2_util
)

ament_target_dependencies(${library_name}
  ${dependencies}
)
ament_target_dependencies(${library_direct_name}
  ${dependencies}
  ament_index_cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "node_parameter_handler::SetParameterCustom")
rclcpp_components_register_nodes(${library_direct_name} "node_parameter_handler::ModifyYaml")

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${library_direct_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
