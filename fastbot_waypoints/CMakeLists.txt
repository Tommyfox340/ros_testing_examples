cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the custom action interface 
rosidl_generate_interfaces(${PROJECT_NAME}
	"action/Waypoint.action"
	DEPENDENCIES  std_msgs geometry_msgs)
# Export runtime dependency so downstream packages can use the generated interfaces
ament_export_dependencies(rosidl_default_runtime)


# Create an executable target for fastbot_action_server and link it with dependencies  
# including our custom action interface
add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server
	rclcpp
	rclcpp_action
	nav_msgs
	geometry_msgs
	tf2)
if(COMMAND rosidl_get_typesupport_target)
  # Get the typesupport target and link it (adds include dirs + build order)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastbot_action_server ${cpp_typesupport_target})
else()
  rosidl_target_interfaces(fastbot_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

# this installs the executable in the lib folder to be seen by setup.bash to be seen by ros2
install(TARGETS 
 fastbot_action_server
 DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # ============================================
  # Option 1: Standalone C++ GTest
  # Requires: Gazebo + action server running separately
  # Run with: colcon test --packages-select fastbot_waypoints
  # ============================================
  set(TESTFILES
      test/main.cpp
      test/test_waypoint_reached.cpp
  )
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    rclcpp_action
    nav_msgs
    geometry_msgs
    tf2
    action_msgs
  )
  # Link with generated action interface
  if(COMMAND rosidl_get_typesupport_target)
    rosidl_get_typesupport_target(cpp_ts ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${PROJECT_NAME}_test ${cpp_ts})
  else()
    rosidl_target_interfaces(${PROJECT_NAME}_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()

  # ============================================
  # Option 2: Launch test (starts action server + resets Gazebo)
  # Requires: Only Gazebo running separately
  # Run with: launch_test test/test_waypoint_launch.py
  # ============================================
  # Build gtest EXE but do NOT register it as a CTest test
  # ament_add_gtest_executable(${PROJECT_NAME}_gtest_integration
  #   test/main.cpp
  #   test/test_waypoint_reached.cpp
  # )
  # add_launch_test(
  #   test/test_waypoint_integration_launch.py
  #   TIMEOUT 120
  #   ENV "GTEST_EXE=$<TARGET_FILE:${PROJECT_NAME}_gtest_integration>"
  # )

endif()

ament_package()
