cmake_minimum_required(VERSION 3.8)
project(moveit_control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(moveit_cpp_ur_node src/moveit_cpp_ur_node.cpp)

target_include_directories(moveit_cpp_ur_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(moveit_cpp_ur_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(moveit_cpp_ur_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2
  tf2_geometry_msgs
)

install(TARGETS moveit_cpp_ur_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


add_executable(cartesian_control src/cartesian_control.cpp)

target_compile_features(cartesian_control PUBLIC c_std_99 cxx_std_17)

target_include_directories(cartesian_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cartesian_control
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2
  tf2_geometry_msgs
)

install(TARGETS cartesian_control
  DESTINATION lib/${PROJECT_NAME}
)

