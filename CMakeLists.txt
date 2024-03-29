cmake_minimum_required(VERSION 3.8)
project(wpr_simulation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object.msg" 
  DEPENDENCIES 
  "std_msgs"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(wpb_home_mani_sim src/wpb_home_mani_sim.cpp)
ament_target_dependencies(wpb_home_mani_sim 
"rclcpp" "std_msgs" "sensor_msgs"
)

add_executable(keyboard_vel_cmd src/keyboard_vel_cmd.cpp)
ament_target_dependencies(keyboard_vel_cmd 
"rclcpp" "geometry_msgs"
)

add_executable(ball_random_move src/ball_random_move.cpp)
ament_target_dependencies(ball_random_move 
"rclcpp" "geometry_msgs" 
)

add_executable(objects_publisher src/objects_publisher.cpp)
ament_target_dependencies(objects_publisher
"rclcpp" "sensor_msgs" "pcl_conversions" "pcl_ros" "std_msgs" "visualization_msgs"
)
target_link_libraries(objects_publisher "${cpp_typesupport_target}")

add_executable(grab_object_sim src/grab_object_sim.cpp)
ament_target_dependencies(grab_object_sim
"rclcpp" "sensor_msgs" "pcl_conversions" "pcl_ros"
)
target_link_libraries(grab_object_sim "${cpp_typesupport_target}")

add_executable(3_my_node demo_cpp/3_my_node.cpp)
ament_target_dependencies(3_my_node 
"rclcpp"
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

add_executable(5_lidar_data demo_cpp/5_lidar_data.cpp)
ament_target_dependencies(5_lidar_data 
"rclcpp" "sensor_msgs"
)

add_executable(5_lidar_behavior demo_cpp/5_lidar_behavior.cpp)
ament_target_dependencies(5_lidar_behavior 
"rclcpp" "sensor_msgs" "geometry_msgs"
)

add_executable(6_imu_data demo_cpp/6_imu_data.cpp)
ament_target_dependencies(6_imu_data 
"rclcpp" "sensor_msgs" "tf2" 
)

add_executable(6_imu_behavior demo_cpp/6_imu_behavior.cpp)
ament_target_dependencies(6_imu_behavior 
"rclcpp" "sensor_msgs" "tf2" "geometry_msgs"
)

add_executable(8_waypoint_navigation demo_cpp/8_waypoint_navigation.cpp)
ament_target_dependencies(8_waypoint_navigation 
"rclcpp" "std_msgs"
)

add_executable(9_cv_image demo_cpp/9_cv_image.cpp)
ament_target_dependencies(9_cv_image 
"rclcpp" "sensor_msgs" "sensor_msgs" "cv_bridge" "image_transport" "OpenCV"
)

add_executable(9_cv_hsv demo_cpp/9_cv_hsv.cpp)
ament_target_dependencies(9_cv_hsv 
"rclcpp" "sensor_msgs" "sensor_msgs" "cv_bridge" "image_transport" "OpenCV"
)

add_executable(9_cv_follow demo_cpp/9_cv_follow.cpp)
ament_target_dependencies(9_cv_follow 
"rclcpp" "sensor_msgs" "sensor_msgs" "cv_bridge" "image_transport" "OpenCV"
)

add_executable(9_cv_face_detect demo_cpp/9_cv_face_detect.cpp)
ament_target_dependencies(9_cv_face_detect 
"rclcpp" "sensor_msgs" "sensor_msgs" "cv_bridge" "image_transport" "OpenCV"
)

add_executable(10_pc_data demo_cpp/10_pc_data.cpp)
ament_target_dependencies(10_pc_data
"rclcpp" "sensor_msgs" "pcl_conversions" "pcl_ros"
)

add_executable(10_pc_objects demo_cpp/10_pc_objects.cpp)
ament_target_dependencies(10_pc_objects
"rclcpp" "sensor_msgs" "pcl_conversions" "pcl_ros"
)

add_executable(11_mani_ctrl demo_cpp/11_mani_ctrl.cpp)
ament_target_dependencies(11_mani_ctrl
"rclcpp" "sensor_msgs"
)

add_executable(11_grab_object demo_cpp/11_grab_object.cpp)
ament_target_dependencies(11_grab_object
"rclcpp" "sensor_msgs" "pcl_conversions" "pcl_ros"
)
target_link_libraries(11_grab_object "${cpp_typesupport_target}")

add_executable(12_fetch demo_cpp/12_fetch.cpp)
ament_target_dependencies(12_fetch 
"rclcpp" "std_msgs" "sensor_msgs"
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

install(DIRECTORY config launch demo_launch worlds models meshes rviz src msg maps
  DESTINATION share/${PROJECT_NAME})

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/face_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    wpb_home_mani_sim keyboard_vel_cmd ball_random_move objects_publisher grab_object_sim
    3_publisher_node 3_subscriber_node 
    4_velocity_command 
    5_lidar_data 5_lidar_behavior 
    6_imu_data 6_imu_behavior 
    8_waypoint_navigation 
    9_cv_image 9_cv_hsv 9_cv_follow 9_cv_face_detect
    10_pc_data 10_pc_objects 
    11_mani_ctrl 11_grab_object
    12_fetch
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
