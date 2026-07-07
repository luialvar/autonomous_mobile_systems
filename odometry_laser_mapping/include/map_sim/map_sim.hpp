# pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

typedef struct pose2D {
  double x, y;
} pose2D;

typedef struct pose {
  pose2D pos;
  double theta;
} pose;

class map_sim : public rclcpp::Node
{
public:
  map_sim();
private:
  void timer_callback();
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  int cell_value(int x, int y);

  pose2D bresenham(pose2D start, pose2D end);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.0;
  double o_x_ = 0.0;
  double o_y_ = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  pose laser_pose;
  double angle_increment_deg_ = 0.05;
  double range_max_m_ = 100.0;
};
