cmake_minimum_required(VERSION 3.16)
project(camera_calibration)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(calibrate src/calibrate.cc)
target_compile_features(calibrate PRIVATE cxx_std_17)
target_include_directories(calibrate PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calibrate ${OpenCV_LIBS})

add_executable(pose src/pose.cc)
target_compile_features(pose PRIVATE cxx_std_17)
target_include_directories(pose PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(
  pose
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
target_link_libraries(pose ${OpenCV_LIBS})

install(
  TARGETS calibrate pose
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

