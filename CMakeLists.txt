cmake_minimum_required(VERSION 3.8)
project(deploadblade)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(librealsense2 REQUIRED)
find_package(realsense2 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# === ADD THIS BELOW ===
#add_executable(depth_roi_publisher src/depth_roi_publisher.cpp)

#ament_target_dependencies(depth_roi_publisher
#  rclcpp
#  sensor_msgs
#  cv_bridge
#  image_transport
#  OpenCV
#)


#target_link_libraries(depth_roi_publisher
#  ${OpenCV_LIBS}
#  realsense2
#)
add_executable(depth_processor scripts/depth_processor.cpp)
ament_target_dependencies(depth_processor rclcpp cv_bridge image_transport sensor_msgs OpenCV)

install(TARGETS
  depth_processor
  DESTINATION lib/${PROJECT_NAME}
)

# This is for the depth camera
add_executable(depth_publisher src/depth_publisher.cpp)
ament_target_dependencies(depth_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  realsense2
)
target_link_libraries(depth_publisher
  realsense2
)

install(TARGETS
  depth_publisher
  DESTINATION lib/${PROJECT_NAME}
)
# This is for the loadcell
add_executable(loadcell_publisher src/load_publisher.cpp)
ament_target_dependencies(loadcell_publisher rclcpp std_msgs)
target_link_libraries(loadcell_publisher serial)
install(TARGETS
  loadcell_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# for motor control
add_executable(motor_control src/motor_control.cpp)
ament_target_dependencies(motor_control rclcpp)
install(TARGETS
  motor_control
  DESTINATION lib/${PROJECT_NAME}
)