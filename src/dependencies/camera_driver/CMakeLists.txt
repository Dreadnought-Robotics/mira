cmake_minimum_required(VERSION 3.8)
project(camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(image_transport REQUIRED)

# Find system libraries
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-server-1.0)
pkg_check_modules(UDEV REQUIRED libudev)

add_executable(camera_driver_exe src/camera_driver_node.cpp)
add_executable(camera_driver_recv src/camera_driver_recv.cpp)
target_include_directories(camera_driver_exe PRIVATE
  ${GST_INCLUDE_DIRS}
  ${UDEV_INCLUDE_DIRS}
)
target_include_directories(camera_driver_recv PRIVATE
  ${GST_INCLUDE_DIRS}
  ${UDEV_INCLUDE_DIRS}
)
target_link_libraries(camera_driver_recv
  ${rclcpp_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GST_LIBRARIES}
  ${UDEV_LIBRARIES}
)
target_link_libraries(camera_driver_exe
  ${rclcpp_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GST_LIBRARIES}
  ${UDEV_LIBRARIES}
)
ament_target_dependencies(camera_driver_exe
  rclcpp
  image_transport
  sensor_msgs
  std_msgs
)

ament_target_dependencies(camera_driver_recv
  rclcpp
  image_transport
  sensor_msgs
  std_msgs
)

# Install executable
install(TARGETS camera_driver_exe
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS camera_driver_recv
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
