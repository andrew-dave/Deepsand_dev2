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
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

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

# for teleop robot client
add_executable(teleop_client src/teleop_client.cpp)
target_link_libraries(teleop_client gpiodcxx)
ament_target_dependencies(teleop_client rclcpp geometry_msgs)
install(TARGETS
  teleop_client
  DESTINATION lib/${PROJECT_NAME}
)

# for teleop host
add_executable(teleop_host src/teleop_host.cpp)
target_link_libraries(teleop_host SDL2)
ament_target_dependencies(teleop_host rclcpp geometry_msgs)

install(TARGETS teleop_host DESTINATION lib/${PROJECT_NAME})

ament_package()