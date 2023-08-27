cmake_minimum_required(VERSION 3.8)
project(wpr_simulation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

add_executable(wpb_home_mani_sim src/wpb_home_mani_sim.cpp)
ament_target_dependencies(wpb_home_mani_sim 
"rclcpp" "std_msgs" "sensor_msgs"
)

add_executable(3_publisher_node demo_cpp/3_publisher_node.cpp)
ament_target_dependencies(3_publisher_node 
"rclcpp" "std_msgs"
)

add_executable(3_subscriber_node demo_cpp/3_subscriber_node.cpp)
ament_target_dependencies(3_subscriber_node 
"rclcpp" "std_msgs"
)

add_executable(4_velocity_command demo_cpp/4_velocity_command.cpp)
ament_target_dependencies(4_velocity_command 
"rclcpp" "geometry_msgs"
)

add_executable(demo_mani_ctrl demo_cpp/demo_mani_ctrl.cpp)
ament_target_dependencies(demo_mani_ctrl
"rclcpp" "sensor_msgs"
)

add_executable(demo_nav2_client demo_cpp/demo_nav2_client.cpp)
ament_target_dependencies(demo_nav2_client
  "rclcpp" "rclcpp_action" "nav2_msgs"
)

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

install(DIRECTORY config launch demo_launch worlds models meshes rviz src map
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS
    wpb_home_mani_sim
    3_publisher_node 3_subscriber_node
    4_velocity_command
    demo_mani_ctrl  
    demo_nav2_client
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)

ament_package()
