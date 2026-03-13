cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
   DEPENDENCIES std_msgs sensor_msgs
)

# Create executable
add_executable(robot_patrol_node src/patrol_main.cpp src/patrol.cpp)
add_executable(robot_patrol_service src/direction_service.cpp)
add_executable(test_service src/test_service.cpp)
add_executable(patrol_with_service src/patrol_main.cpp src/patrol_with_service.cpp)

target_include_directories(robot_patrol_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(robot_patrol_service PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(test_service PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(patrol_with_service PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add dependencies
ament_target_dependencies(robot_patrol_node rclcpp sensor_msgs geometry_msgs rosidl_default_runtime)
ament_target_dependencies(robot_patrol_service rclcpp sensor_msgs geometry_msgs rosidl_default_runtime)
ament_target_dependencies(test_service rclcpp sensor_msgs geometry_msgs rosidl_default_runtime)
ament_target_dependencies(patrol_with_service rclcpp sensor_msgs geometry_msgs rosidl_default_runtime)

# Tell CMake that robot_patrol_service uses the generated service interface
rosidl_target_interfaces(robot_patrol_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(robot_patrol_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(test_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(patrol_with_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install executables
install(TARGETS
  robot_patrol_node
  robot_patrol_service
  test_service
  patrol_with_service
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

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
