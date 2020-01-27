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
  uint robot_id_;
  std::string robot_name_;
  


  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  // Constructor
  Robot(uint robot_id_, std::string robot_name_, uint no_of_robots=1,
        double radial_noise=0.01,
        double sensing_radius_=0.40);
  void move();
  void timer_callback();

};

#endif  // MAPS_HOUGH_ROBOT_HPP