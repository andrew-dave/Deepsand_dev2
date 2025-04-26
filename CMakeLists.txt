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
# find_package(librealsense2 REQUIRED)
# find_package(realsense2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(realsense2 QUIET)

# Try finding libgpiod - for host
pkg_check_modules(GPIOD QUIET libgpiod)
if(GPIOD_FOUND)
  message(STATUS "Found libgpiod")
  add_definitions(-DUSE_GPIOD)
else()
  message(WARNING "libgpiod not found, GPIO features will be disabled.")
endif()

# Try finding libserial -for host
pkg_check_modules(LIBSERIAL QUIET libserial)
if(LIBSERIAL_FOUND)
  message(STATUS "Found libserial")
  add_definitions(-DUSE_LIBSERIAL)
else()
  message(WARNING "libserial not found, serial features will be disabled.")
endif()

# For host 
if(realsense2_FOUND)
  message(STATUS "Found RealSense2")
  add_definitions(-DUSE_REALSENSE)
else()
  message(WARNING "RealSense2 SDK not found. Some features will be disabled.")
endif()


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
if(realsense2_FOUND)
# 1: Older implementation that publishes images
add_executable(depth_publisher src/depth_publisher.cpp)
ament_target_dependencies(depth_publisher
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  realsense2
)
target_link_libraries(depth_publisher realsense2 ${OpenCV_LIBS})
install(TARGETS
  depth_publisher
  DESTINATION lib/${PROJECT_NAME}
)
# 2A: New implementation that publishes depth information as raw int 16 multiarray
add_executable(depth_publisher_e src/depth_publisher_e.cpp)
ament_target_dependencies(depth_publisher_e rclcpp std_msgs realsense2)
target_link_libraries(depth_publisher_e realsense2)
install(TARGETS depth_publisher_e DESTINATION lib/${PROJECT_NAME})

# 2B: This is the node that processes the raw data
add_executable(depth_to_img src/depthtoimg.cpp)
ament_target_dependencies(depth_to_img rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
install(TARGETS depth_to_img DESTINATION lib/${PROJECT_NAME})

endif()

# This is for the loadcell
if(LIBSERIAL_FOUND)
add_executable(loadcell_publisher src/load_publisher.cpp)
ament_target_dependencies(loadcell_publisher rclcpp std_msgs)
target_link_libraries(loadcell_publisher serial)
install(TARGETS
  loadcell_publisher
  DESTINATION lib/${PROJECT_NAME}
)
endif()

# for teleop robot client
if(GPIOD_FOUND)
add_executable(teleop_client src/teleop_client.cpp)
target_link_libraries(teleop_client gpiodcxx serialport)
ament_target_dependencies(teleop_client rclcpp geometry_msgs std_msgs)
install(TARGETS
  teleop_client
  DESTINATION lib/${PROJECT_NAME}
)
endif()

# for teleop host
add_executable(teleop_host src/teleop_host.cpp)
target_link_libraries(teleop_host SDL2)
ament_target_dependencies(teleop_host rclcpp geometry_msgs)

install(TARGETS teleop_host DESTINATION lib/${PROJECT_NAME})

# Launch file for Depth+(loadcell+blade_angle)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
