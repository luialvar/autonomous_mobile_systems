#include "map_build/map_build.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <vector>

using namespace std;

map_build::map_build() : Node("map_build") {
  RCLCPP_INFO(this->get_logger(), "Map builder started");
  output_map_path_ = declare_parameter<string>("output_map_path", "map.ppm");

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 1,
      std::bind(&map_build::scan_subscriber_callback, this,
                std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1,
      std::bind(&map_build::odom_subscriber_callback, this,
                std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map_from_scan", rclcpp::QoS(1).reliable().transient_local());

  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      map[i][j] = 0.5f;
    }
  }
  new_pose = false;
  initial_pose_ = false;
  scan_count = 0;
}

void map_build::scan_subscriber_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
  if (!new_pose)
    return;
  else
    new_pose = false;

  float angle_min = _msg->angle_min;
  float angle_increment = _msg->angle_increment;
  float range_min = _msg->range_min;
  float range_max = _msg->range_max;

  float alpha = atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y +
                   orientation.z * orientation.z));
  float angle = alpha + angle_min;
  geometry_msgs::msg::Point current_pose = pose;

  for (size_t i = 0; i < _msg->ranges.size(); i++) {
    float distance = _msg->ranges.at(i);

    bool invalid = !std::isfinite(distance) ||
                   distance >= range_max ||
                   distance <= range_min;
    if (invalid) {
      distance = range_max;
    }

    this->update_map(current_pose, angle, distance, invalid);

    angle += angle_increment;
  }
  this->map_to_ppm(output_map_path_.c_str());
  this->publish_map();
  scan_count++;
}

void map_build::odom_subscriber_callback(
    const nav_msgs::msg::Odometry::SharedPtr _msg) {
  pose = _msg->pose.pose.position;

  if (initial_pose_ == false) {
    initial_pose = pose;
    initial_pose_ = true;
  }

  new_pose = true;
  pose.x = pose.x - initial_pose.x;
  pose.y = pose.y - initial_pose.y;
  orientation = _msg->pose.pose.orientation;
}

void map_build::update_map(geometry_msgs::msg::Point robot_pose, float angle,
                           float range, bool invalid) {
  geometry_msgs::msg::Point end;
  geometry_msgs::msg::Point pose;
  pose.x = robot_pose.x / CELL_SIZE + MAP_SIZE / 2;
  pose.y = robot_pose.y / CELL_SIZE + MAP_SIZE / 2;
  end.x = (robot_pose.x + range * cos(angle)) / CELL_SIZE + MAP_SIZE / 2;
  end.y = (robot_pose.y + range * sin(angle)) / CELL_SIZE + MAP_SIZE / 2;

  if (!in_bounds((int)pose.x, (int)pose.y))
    return;

  if (!invalid && in_bounds((int)end.x, (int)end.y))
    this->update_probability(end, P_HIT);

  this->bresenham(pose, end);
}

void map_build::update_probability(geometry_msgs::msg::Point point,
                                   float intensity) {
  int x = (int)point.x;
  int y = (int)point.y;
  if (!in_bounds(x, y))
    return;

  float a_priori = std::clamp(map[x][y], 0.01f, 0.99f);
  float P_m = P_UNKNOWN;
  float P_zx = std::clamp(intensity, 0.01f, 0.99f);
  float a_posteriori = 1 - (1 / (1 + (P_zx / (1 - P_zx)) * ((1 - P_m) / P_m) *
                                         (a_priori / (1 - a_priori))));
  map[x][y] = std::clamp(a_posteriori, 0.0f, 1.0f);
}

void map_build::bresenham(geometry_msgs::msg::Point start,
                          geometry_msgs::msg::Point end) {
  int x0 = (int)start.x;
  int y0 = (int)start.y;

  int x1 = (int)end.x;
  int y1 = (int)end.y;

  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);

  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;

  int err = dx - dy;

  while (true) {
    if (!in_bounds(x0, y0))
      break;

    if (x0 == x1 && y0 == y1)
      break;

    geometry_msgs::msg::Point point;
    point.x = x0;
    point.y = y0;
    this->update_probability(point, P_EMPTY);

    int e2 = 2 * err;

    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }

    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

bool map_build::in_bounds(int x, int y) const {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

void map_build::map_to_ppm(const char *file_name) {
  FILE *f = fopen(file_name, "w");
  if (!f) {
    perror("fopen");
    return;
  }

  fprintf(f, "P2\n#Map for AMS E8.1!\n%d %d\n255\n", MAP_SIZE, MAP_SIZE);

  for (int y = 0; y < MAP_SIZE; y++) {
    for (int x = 0; x < MAP_SIZE; x++) {
      float value = (1.0f - map[x][y]) * 255.0f;

      if (value < 0.0f)
        value = 0.0f;
      if (value > 255.0f)
        value = 255.0f;

      fprintf(f, "%d ", (int)value);
    }
    fprintf(f, "\n");
  }

  fclose(f);
}

void map_build::publish_map() {
  nav_msgs::msg::OccupancyGrid grid;

  grid.header.stamp = this->get_clock()->now();
  grid.header.frame_id = "map";

  grid.info.map_load_time = this->get_clock()->now();
  grid.info.resolution = CELL_SIZE;
  grid.info.width = MAP_SIZE;
  grid.info.height = MAP_SIZE;
  geometry_msgs::msg::Pose origin;
  origin.position.x = -MAP_SIZE * CELL_SIZE / 2;
  origin.position.y = -MAP_SIZE * CELL_SIZE / 2;
  origin.position.z = 0;
  origin.orientation.w = 1;
  origin.orientation.x = 0;
  origin.orientation.y = 0;
  origin.orientation.z = 0;
  grid.info.origin = origin;

  std::vector<int8_t> data(MAP_SIZE * MAP_SIZE, -1);
  for (int y = 0; y < MAP_SIZE; y++) {
    for (int x = 0; x < MAP_SIZE; x++) {
      float probability = map[x][y];
      int8_t value = -1;
      if (probability >= THRESHOLD_OCCUPIED) {
        value = 100;
      } else if (probability <= THRESHOLD_EMPTY) {
        value = 0;
      }

      data[x + y * MAP_SIZE] = value;
    }
  }
  grid.data = data;
  map_pub_->publish(grid);
}
