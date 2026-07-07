cmake_minimum_required(VERSION 3.16)
project(map_build)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(map_build_node
  src/main.cpp
  src/map_build.cpp
)
target_compile_features(map_build_node PRIVATE cxx_std_17)
target_include_directories(map_build_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_build_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS map_build_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch maps assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

