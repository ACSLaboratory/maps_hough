#include "maps_hough/robot.hpp"


Robot::Robot(uint robot_id_, 
             std::string robot_name_,
             uint no_of_robots,
             double radial_noise,
             double sensing_radius_):
    Node("robot_node"),
    id_{robot_id_},
    name_{robot_name_},
    abs_pose{0.0, 0.0, 0.0},
    sensing_radius_{sensing_radius_},
    last_communication(no_of_robots),
    avoidCount{0},
    randCount{0}

/**
 * Constructor for the robot class
 * @param robot_id_ : the id number of the robot
 * @param robot_name_ : the common name for the robot
 * @param planner_ : the planner object pointer
 * @param radial_noise : the variance of the radial noise of the range sensor
 * @param sensing_radius_ : the sensing radius of the robot
 */
{
  std:: string robot_name_aug = robot_name_ + std::to_string(robot_id_);
  laser_sensor.range_noise_const = radial_noise;

  // Set data path
  if (in_test_mode_)
  {
    RCLCPP_WARN(this->get_logger(), "TEST MODE: True");
    data_path = "/home/acs-lab/ROS2/ragesh_ws/data/tb3_" + std::to_string(robot_id_) + "/";
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "TEST MODE: False");
    data_path = "/home/turtlebot-0" + std::to_string(robot_id_) + "/ros2/ragesh_ws/data/tb3_" + 
      std::to_string(robot_id_) + "/";
  }
  in_test_mode_check_ = in_test_mode_;

  // Setting up the subscriber to laser ranger sensors
  sub_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", std::bind(&Robot::laser_scanner_callback, this, std::placeholders::_1), rmw_qos_profile_sensor_data);

  // Setting up the subscriber to error free gps pose
  sub_abs_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "base_pose_ground_truth", 10, std::bind(&Robot::base_pose_ground_truth_callback, this, std::placeholders::_1));

  // Setting up the publisher to sent maps stored in the robot
  /**
  pub_map_ = it_.advertise("map", 1);

  // Setting up the subscriber to get the map of the robots
  for(uint i=0; i<no_of_robots; i++)
  {
    if(i != robot_id_)
    {
      sub_map_[i] = it_.subscribe(
          "/tb3_" + std::to_string(i) + "/map", 1,
          std::bind(&Robot::update_map_callback, this, std::placeholders::_1, i));
    }
  }
  **/

  // Lambda function for updating maps. This is an inception model, a lambda within a lambda.
  auto update_map_callback = [this](const int &i)
  {
    return [this, i](sensor_msgs::msg::Image::SharedPtr msg) -> void
    {
      RCLCPP_INFO(this->get_logger(), "TEST UMC%d Size of nbh_ids: %d", i, nbh_ids.size());
      if (!nbh_ids.empty() && std::find(nbh_ids.begin(), nbh_ids.end(), i) != nbh_ids.end())
      {
        RCLCPP_INFO(get_logger(), "Updating Map");
        
        // check if ample time has past since the map merger
        if (last_communication[i] + comm_delay < this->now().seconds())
        {
          RCLCPP_INFO(get_logger(), "Merging the maps of robots %d and %d", robot_id, i);
          merger(occ_grid_map->og_, cv_bridge::toCvShare(msg, "mono8")->image);
          last_communication[i] = this->now().seconds();
        }
      }
    };
  };

  for(uint i=0; i<no_of_robots; i++)
  {
    if(i != robot_id_)
    {
      sub_map_[i] = this->create_subscription<sensor_msgs::msg::Image>(
          "/tb3_" + std::to_string(i) + "/map", 1, update_map_callback(i));
    }
  }

  pub_map_ = this->create_publisher<sensor_msgs::msg::Image>("map", 10);

  // Setting up the publisher to command velocity
  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Setting up the client for neighbor service
  get_nbh_client_ = this->create_client<levy_walk_msgs::srv::Neighbors>(nbh_service_name);

  // Set up timer
  timer_ = this->create_wall_timer(500ms, std::bind(&Robot::timer_callback, this));
  static_time = this->now().seconds();
  pre_time = 0;
  count = 0;

}