cmake_minimum_required(VERSION 3.8)
project(camera_driver)

# ------------------------
# Compiler options
# ------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------
# Dependencies
# ------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


# ZED Open Capture is a system library (installed in /usr/local)
find_library(ZED_OPEN_CAPTURE_LIB zed_open_capture REQUIRED)

# ------------------------
# Executable
# ------------------------
add_executable(zed_open_capture_imu_node
  src/zed_open_capture_node.cpp
)

target_compile_definitions(zed_open_capture_imu_node
  PRIVATE
    SENSORS_MOD_AVAILABLE
)

target_include_directories(zed_open_capture_imu_node
  PRIVATE
    /usr/local/include/zed-open-capture
    /usr/include/hidapi
)

add_executable(imu_to_tf_node
  src/imu_to_tf_node.cpp
)

ament_target_dependencies(imu_to_tf_node
  rclcpp
  sensor_msgs
  tf2_ros
  geometry_msgs
)

install(TARGETS imu_to_tf_node
  DESTINATION lib/${PROJECT_NAME}
)


target_link_libraries(zed_open_capture_imu_node
  ${ZED_OPEN_CAPTURE_LIB}
)

ament_target_dependencies(zed_open_capture_imu_node
  rclcpp
  sensor_msgs
)

# ------------------------
# Install
# ------------------------
install(TARGETS zed_open_capture_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------
# Package export
# ------------------------
ament_package()
