#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// Map stuff
#define CELL_SIZE    0.10f
#define MAP_SIZE     (int)(100/ CELL_SIZE)

// probability stuff
#define P_UNKNOWN    0.5f
#define P_HIT        0.7f
#define P_EMPTY      0.3f

#define THRESHOLD_EMPTY 0.45f
#define THRESHOLD_OCCUPIED 0.55f

class map_build: public rclcpp::Node
{
public:
  map_build();
private:
  void scan_subscriber_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
  void odom_subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr _msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  geometry_msgs::msg::Point pose;
  geometry_msgs::msg::Quaternion orientation;

  float map[MAP_SIZE][MAP_SIZE];
  bool new_pose;
  bool initial_pose_;
  geometry_msgs::msg::Point initial_pose;
  std::string output_map_path_;


  void update_map(
		  geometry_msgs::msg::Point robot_pose,
		  float angle,
		  float range,
		  bool valid
		  );

  void update_probability(
			  geometry_msgs::msg::Point point,
			  float intensity
			  );
  void bresenham(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end);
  bool in_bounds(int x, int y) const;

  int scan_count;

  void map_to_ppm(const char *file_name);
  void publish_map();
};
