cmake_minimum_required(VERSION 3.8)
project(camera_cus_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc aruco)
# find_package(image_transport REQUIRED)
# find_package(OpenCV REQUIRED)

add_executable(camera_node_cpp src/camera_node.cpp)
target_include_directories(camera_node_cpp PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(camera_node_cpp
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
  OpenCV
)

install(TARGETS
  camera_node_cpp
  DESTINATION lib/${PROJECT_NAME}
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
