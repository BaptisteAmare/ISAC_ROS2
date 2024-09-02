cmake_minimum_required(VERSION 3.8)
project(navigation_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Déclarez les exécutables avant d'ajouter les interfaces ROSIDL
add_executable(gps_tracker_node src/gps_tracker_node.cpp)
add_executable(waypoint_manager_node src/waypoint_manager_node.cpp)
add_executable(navigation_calculator_node src/navigation_calculator_node.cpp)
add_executable(monitoring_display_node src/monitoring_display_node.cpp)

# Ajoutez les dépendances pour chaque cible
ament_target_dependencies(gps_tracker_node rclcpp std_msgs geometry_msgs)
ament_target_dependencies(waypoint_manager_node rclcpp std_srvs std_msgs geometry_msgs)
ament_target_dependencies(navigation_calculator_node rclcpp std_msgs geometry_msgs)
ament_target_dependencies(monitoring_display_node rclcpp std_msgs geometry_msgs)

# Générez les interfaces ROSIDL et liez-les avec les cibles appropriées
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddWaypoint.srv"
  "srv/RemoveWaypoint.srv"  
  DEPENDENCIES geometry_msgs std_msgs
)

# Lier les interfaces ROSIDL avec la cible waypoint_manager_node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(waypoint_manager_node ${cpp_typesupport_target})

# Installation des exécutables
install(TARGETS
  gps_tracker_node
  waypoint_manager_node
  navigation_calculator_node
  monitoring_display_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()