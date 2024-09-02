cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(vector_transformation_node src/vector_transformation_node.cpp)
add_executable(coordinate_publisher_node src/coordinate_publisher_node.cpp)
add_executable(camera_node src/camera_node.cpp)
add_executable(object_detection_node src/object_detection_node.cpp)

ament_target_dependencies(
  coordinate_publisher_node
  rclcpp
  std_msgs
)

ament_target_dependencies(
  vector_transformation_node
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(
  camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(
  object_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)

target_include_directories(vector_transformation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vector_transformation_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install all executables
install(TARGETS
  vector_transformation_node
  coordinate_publisher_node
  camera_node
  object_detection_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ressources/
  DESTINATION share/${PROJECT_NAME}/ressources)
  
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
