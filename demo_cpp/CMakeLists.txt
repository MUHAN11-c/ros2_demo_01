cmake_minimum_required(VERSION 3.8)
project(demo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(demo_cpp_service_node src/demo_cpp_service.cpp)
ament_target_dependencies(demo_cpp_service_node rclcpp geometry_msgs turtlesim custom_interfaces) 

add_executable(turtlesim_pose_sub_node src/turtlesim_pose_sub.cpp)
ament_target_dependencies(turtlesim_pose_sub_node rclcpp geometry_msgs turtlesim custom_interfaces) 

add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(person_node src/person.cpp)
ament_target_dependencies(person_node rclcpp)

add_executable(turtlesim_pose_cpptopic_node src/turtlesim_pose_cpptopic.cpp)
ament_target_dependencies(turtlesim_pose_cpptopic_node rclcpp geometry_msgs turtlesim)

add_executable(demo_cpp_client_node src/demo_cpp_client.cpp)
ament_target_dependencies(demo_cpp_client_node rclcpp custom_interfaces turtlesim)



install(TARGETS
  cpp_node
  person_node
  turtlesim_pose_cpptopic_node
  turtlesim_pose_sub_node
  demo_cpp_service_node
  demo_cpp_client_node
  DESTINATION lib/${PROJECT_NAME})



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
