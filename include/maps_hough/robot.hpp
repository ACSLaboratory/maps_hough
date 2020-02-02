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
#include "levy_walk_msgs/srv/neighbors.hpp"


struct Bounds
{
  /*
   * A structure to store the bounds of the quantities
   */
  double max;
  double min;
};


// A structure to hold range sensor data
struct LaserSensor
{
  /*
   * A structure to store various entities of a laser range scanner
   */
  std::vector<double> bearings;  // various bearing of the laser scanner
  double range_noise_const;      // variance of the range sensor along the radial direction
  Bounds range;                  // the max and min range of the sensor
  Bounds angle;                  // the max and min angle of the sensor
  std::vector<double> ranges;    // the range along each bearing

};


class Robot : public rclcpp::Node
{
private:
  // Robot Attributes
  uint id_;
  std::string name_;
  double sensing_radius_;
  long int avoid_count_;
  long int rand_count_;
  LaserSensor laser_sensor_info_;

  // Neighbor Attributes
  std::vector<uint> nbh_ids_;               // vector to store the neighbor robot ids
  std::vector<double> last_communication_;  // store robot's time of encounter with others (sec)
  const double comm_delay_ = 10;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_map_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_abs_pose_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 3> sub_map_;

  // Services
  rclcpp::Client<levy_walk_msgs::srv::Neighbors>::SharedPtr srv_nbh_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  // Constructor
  Robot(uint robot_id_, std::string robot_name_, std::string srv_name_nbh,
        uint no_of_robots=1,
        double radial_noise=0.01,
        double sensing_radius_=0.40);

  // Robot Attribute Methods
  void set_turn_speed(double new_speed);
  void update_neighbors();

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

  // Node Attribues
  bool verbose = false;

  // Timer Attributes
  double static_time;
  long unsigned int pre_time;
  int count;

};

#endif  // MAPS_HOUGH_ROBOT_HPP