cmake_minimum_required(VERSION 3.16)
project(map_sim)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(map_sim_node
  src/main.cpp
  src/map_sim.cpp
)
target_compile_features(map_sim_node PRIVATE cxx_std_17)
target_include_directories(map_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_sim_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS map_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

