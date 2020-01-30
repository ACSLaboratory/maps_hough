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

  // Setting up the subscriber to laser ranger sensors
  sub_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", std::bind(&Robot::laser_scanner_callback, this, std::placeholders::_1), rmw_qos_profile_sensor_data);

  // Setting up the subscriber to error free gps pose
  sub_abs_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "base_pose_ground_truth", 10, std::bind(&Robot::base_pose_ground_truth_callback, this, std::placeholders::_1));

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

void Robot::laser_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
/**
 * This is the callback function to update the laser range measurement data
 * @param msg : the laser range msg as the data type sensor_msgs/LaserScan
 * 
 * Zahi heavily modified this code to account for the laser scan range on the
 * Turtlebot3 Burger.
 */
{
  if (laser_sensor.ranges.size() == 0)  // initialize the object
  {
    laser_sensor.angle.max = msg->angle_max;
    laser_sensor.angle.min = msg->angle_min;
    laser_sensor.range.max = 0.4;  // msg->range_max;
    laser_sensor.range.min = msg->range_min;
    double temp_range = 0.0;
    
    for (double b = 100 * msg->angle_increment; b >= msg->angle_min; b -= msg->angle_increment)
    {
      laser_sensor.bearings.emplace_back(b);
    }
    
    for (double b = msg->angle_max; b >= 260 * msg->angle_increment; b -= msg->angle_increment)
    {
      laser_sensor.bearings.emplace_back(b);
    }

    int no_of_beams = ((msg->angle_max-msg->angle_min)/msg->angle_increment+1) / 2;
    
    // int j = 0;
    for (auto i=90; i>=0; i--)
    {
      laser_sensor.ranges.emplace_back(static_cast<double>(msg->ranges[i]));
      // laser_sensor.ranges[j] = static_cast<double>(msg->ranges[i]);
      // j++;
    }

    for (auto i=359; i>=270; i--)
    {
      laser_sensor.ranges.emplace_back(static_cast<double>(msg->ranges[i]));
      // laser_sensor.ranges[j] = static_cast<double>(msg->ranges[i]);
      // j++;
    }

  }
  else
  {
    int j = 90;
    for (auto i=0; i<=90; i++)
    {
      // ROS_INFO("laser sensor %d: %.3f", i, msg->ranges[i]);
      if (msg->ranges[j] > 0.4)
      {
        laser_sensor.ranges[i] = 0.4;
      // This `else if` statement is a way to filter 0 readings by the lidar
      }
      else if (msg->ranges[j] <= 0.02)
      {
        // laser_sensor.ranges[i] = 0.36;
        for (int k=1; k<=3; k++)
        {
          if (msg->ranges[j+k] > 0.02)
          {
            laser_sensor.ranges[i] = (msg->ranges[j+k] > 0.4) ? 0.4 : msg->ranges[j+k];
            break;
          }
          else if (k == 3)
          {
            laser_sensor.ranges[i] = 0.36;
          }
        }
      }
      else
      {
        laser_sensor.ranges[i] = msg->ranges[j];
      }
      j--;
    }

    j = 359;
    for (auto i=91; i<=180; i++)
    {
      // ROS_INFO("laser sensor %d: %.3f", i, msg->ranges[i]);
      if (msg->ranges[j] > 0.4)
      {
        laser_sensor.ranges[i] = 0.4;
      // This `else if` statement is a way to filter 0 readings by the lidar
      }
      else if (msg->ranges[j] <= 0.02)
      {
        // laser_sensor.ranges[i] = 0.36;
        for (int k=1; k<=3; k++)
        {
          if (msg->ranges[j-k] > 0.02)
          {
            laser_sensor.ranges[i] = (msg->ranges[j-k] > 0.4) ? 0.4 : msg->ranges[j-k];
            break;
          }
          else if (k == 3)
          {
            laser_sensor.ranges[i] = 0.36;
          }
        }
      }
      else
      {
        laser_sensor.ranges[i] = msg->ranges[j];
      }
      j--;
    }
  }
}