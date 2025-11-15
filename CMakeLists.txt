cmake_minimum_required(VERSION 3.8)
project(odin1_ros2_driver)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
set(dependencies
  ament_cmake
  ament_index_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  std_srvs
  cv_bridge
  message_filters
  Threads
  Eigen3
  yaml-cpp
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  rosidl_default_generators
)

foreach (dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach ()

set(SDK_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/lib/)
set(SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/include/)

# Determine architecture and select appropriate SDK library
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(SDK_LIBRARY ${SDK_LIBS_DIR}/liblydHostApi_amd.a)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64|armv8)$")
    set(SDK_LIBRARY ${SDK_LIBS_DIR}/liblydHostApi_arm.a)
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Check if SDK library exists
if(NOT EXISTS ${SDK_LIBRARY})
    message(FATAL_ERROR "SDK library not found: ${SDK_LIBRARY}")
endif()

message(STATUS "Using SDK library: ${SDK_LIBRARY}")

set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDK_INCLUDE_DIR}
  ${OpenCV_INCLUDED_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(COMMON_LIBRARIES
  ${SDK_LIBRARY}
  ${OpenCV_LIBS}
  yaml-cpp
  usb-1.0
  Threads::Threads
  rt
  dl
)

# generate ros2 msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Export the interface dependencies before building targets
ament_export_dependencies(rosidl_default_runtime)

# Get typesupport target
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Create shared library for main component
add_library(odin1_ros2_driver_component SHARED src/odin1_driver.cpp)
ament_target_dependencies(odin1_ros2_driver_component ${dependencies})
target_include_directories(odin1_ros2_driver_component PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(odin1_ros2_driver_component ${COMMON_LIBRARIES} ${cpp_typesupport_target})
rclcpp_components_register_nodes(odin1_ros2_driver_component "odin1_ros2_driver::Odin1Driver")

# Create standalone executable
add_executable(odin1_ros2_driver_node src/odin1_driver_node.cpp)
ament_target_dependencies(odin1_ros2_driver_node ${dependencies})
target_include_directories(odin1_ros2_driver_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(odin1_ros2_driver_node odin1_ros2_driver_component)

install(TARGETS odin1_ros2_driver_component odin1_ros2_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz SDK description
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
