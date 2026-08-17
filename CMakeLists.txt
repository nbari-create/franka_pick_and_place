cmake_minimum_required(VERSION 3.8)
project(myworkcell_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dipendenze ────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fake_ar_publisher REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ── Genera interfacce ROS (srv) ───────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LocalizePart.srv"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── vision_node ───────────────────────────────────────────────────────────────
add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node
  rclcpp fake_ar_publisher tf2_ros tf2_geometry_msgs geometry_msgs)
target_link_libraries(vision_node "${cpp_typesupport_target}")

# ── myworkcell_node ───────────────────────────────────────────────────────────
add_executable(myworkcell_node src/myworkcell_node.cpp)
ament_target_dependencies(myworkcell_node
  rclcpp moveit_ros_planning_interface geometry_msgs)
target_link_libraries(myworkcell_node "${cpp_typesupport_target}")

# ── main_node ─────────────────────────────────────────────────────────────────
add_executable(main_node src/main.cpp src/moveit_node.cpp)

target_include_directories(main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS})

ament_target_dependencies(main_node
  rclcpp sensor_msgs geometry_msgs cv_bridge
  image_transport moveit_ros_planning_interface
  tf2 tf2_ros tf2_geometry_msgs OpenCV
  control_msgs rclcpp_action trajectory_msgs
  moveit_msgs shape_msgs std_msgs  ament_index_cpp)

target_link_libraries(main_node
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2::tf2
  tf2_ros::tf2_ros)




# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  vision_node
  myworkcell_node
  main_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)   # ← decommentata e DESTINATION corretta

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)   # ← decommentata e DESTINATION corretta

# ── Test ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()