#ifndef MAPS_HOUGH_ROBOT_HPP
#define MAPS_HOUGH_ROBOT_HPP

// C++ library header files
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <chrono>

// C library header files
#include <cmath>
#include <cstdio>
#include <cstdlib>

// ros2 libraries
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "cv_bridge/cv_bridge.h"
#include "tf2/utils.h"

// Local header files
#include "maps_hough/common.hpp"
#include "maps_hough/grid_map.hpp"
#include "maps_hough/hough.hpp"
#include "maps_hough/io.hpp"
#include "maps_hough/manipulatemap.hpp"


class Robot : public rclcpp::Node
{
private:
  uint id_;
  std::string name_;
  double sensing_radius_;
  long int avoid_count, rand_count;
  sensor_msgs::msg::LaserScan scan_info_();

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_abs_pose_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  // Constructor
  Robot(uint robot_id_, std::string robot_name_, uint no_of_robots=1,
        double radial_noise=0.01,
        double sensing_radius_=0.40);

  // Robot Attribute Encapsulation
  void set_turn_speed(double new_speed);

  // Robot Operations
  void move();
  void stop();

  // Callbacks
  void timer_callback();
  void laser_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void base_pose_callback(nav_msgs::msg::Odometry::SharedPtr msg);

  // Robot Pose
  double x;  // meters
  double y;  // meters
  double w;  // radians

  // Robot Msgs
  geometry_msgs::msg::Twist cmd_vel_msg();

  // Grid map
  mapmerge::grid_map g_map;

};

#endif  // MAPS_HOUGH_ROBOT_HPP