#include "map_sim/map_sim.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;
using namespace std;

namespace {
int coordinates_to_index(int x, int y, int width) {
  int index = y * width + x;
  return index;
}
}

map_sim::map_sim() : Node("map_sim")
{
  RCLCPP_INFO(this->get_logger(), "Map sim started");

  laser_pose.pos.x = declare_parameter<double>("laser_pose_x_cells", 992.0);
  laser_pose.pos.y = declare_parameter<double>("laser_pose_y_cells", 992.0);
  laser_pose.theta = declare_parameter<double>("laser_pose_theta", 0.0);
  angle_increment_deg_ = declare_parameter<double>("angle_increment_deg", 0.05);
  range_max_m_ = declare_parameter<double>("range_max_m", 100.0);
  angle_increment_deg_ = std::max(angle_increment_deg_, 0.001);
  range_max_m_ = std::max(range_max_m_, 0.01);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
								     "map",
								     rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
								     std::bind(&map_sim::map_callback, this, std::placeholders::_1)
								     );
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  timer_ = this->create_wall_timer(100ms, bind(&map_sim::timer_callback, this));

}

void map_sim::timer_callback()
{
  if (width_ != 0 && map_) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();

    t.header.frame_id = "map";
    t.child_frame_id = "laser";

    t.transform.translation.x = o_x_ + laser_pose.pos.x * resolution_;
    t.transform.translation.y = o_y_ + laser_pose.pos.y * resolution_;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = sin(laser_pose.theta / 2.0);
    t.transform.rotation.w = cos(laser_pose.theta / 2.0);

    tf_broadcaster_->sendTransform(t);


    sensor_msgs::msg::LaserScan scan;

    const double angle_increment_rad = angle_increment_deg_ * M_PI / 180.0;
    const int beam_count = std::max(1, static_cast<int>(360.0 / angle_increment_deg_));
    scan.header.stamp = this->get_clock()->now();
    scan.header.frame_id = "laser";

    scan.angle_min = -M_PI;
    scan.angle_increment = angle_increment_rad;
    scan.angle_max = scan.angle_min + (beam_count - 1) * scan.angle_increment;

    scan.range_min = 0.01;
    scan.range_max = range_max_m_;

    scan.ranges.resize(beam_count);
    scan.intensities.resize(beam_count);
    scan.time_increment = 0.0;
    scan.scan_time = 0.1;

    pose2D start;
    start.x = laser_pose.pos.x;
    start.y = laser_pose.pos.y;

    for (int idx = 0; idx < beam_count; ++idx) {
      pose2D end;

      double theta_rad = laser_pose.theta + scan.angle_min + idx * scan.angle_increment;
      double max_range_cells = scan.range_max / resolution_;

      end.x = start.x + max_range_cells * cos(theta_rad);
      end.y = start.y + max_range_cells * sin(theta_rad);

      pose2D result = this->bresenham(start, end);

      double dx = (double)result.x - start.x;
      double dy = (double)result.y - start.y;

      double dist = sqrt(dx * dx + dy * dy) * resolution_;
      dist = std::min(dist, (double)scan.range_max);

      scan.ranges[idx] = dist;
      float t = (float)idx / (float)beam_count;
      scan.intensities[idx] = t;
    }
    scan_pub_->publish(scan);
  }

}

pose2D map_sim::bresenham(pose2D start, pose2D end)
{
    int x0 = (int)start.x;
    int y0 = (int)start.y;

    int x1 = (int)end.x;
    int y1 = (int)end.y;

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true)
    {
        int value = cell_value(x0, y0);

        if (value > 50 || value == -1)
        {
            pose2D result;
            result.x = x0;
            result.y = y0;
            return result;
        }

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }

    return end;
}

void map_sim::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = msg;

  width_ = msg->info.width;
  height_ = msg->info.height;
  resolution_ = msg->info.resolution;

  o_x_ = msg->info.origin.position.x;
  o_y_ = msg->info.origin.position.y;

  RCLCPP_INFO(get_logger(), "Got map!");
}

int map_sim::cell_value(int x, int y)
{
    if (x < 0 || x >= width_ ||
        y < 0 || y >= height_)
    {
        return 100;
    }

    int index = coordinates_to_index(x, y, width_);
    return map_->data[index];
}
