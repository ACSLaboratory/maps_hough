#include "maps_hough/robot.hpp"


Robot::Robot(uint robot_id_,
             std::string robot_name_,
             uint no_of_robots,
             double radial_noise,
             double sensing_radius_):
    Node("robot_node"),
    id_{robot_id_},
    name_{robot_name_},
    x{0.0},
    y{0.0},
    w{0.0},
    sensing_radius_{sensing_radius_},
    last_communication(no_of_robots),
    avoid_count{0},
    rand_count{0}

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
    "scan", std::bind(&Robot::laser_scan_callback, this, std::placeholders::_1), rmw_qos_profile_sensor_data);

  // Setting up the subscriber to error free gps pose
  sub_abs_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&Robot::base_pose_callback, this, std::placeholders::_1));

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
          RCLCPP_INFO(get_logger(), "Merging the maps of robots %d and %d", id_, i);
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
    
    for (auto i=90; i>=0; i--)
    {
      laser_sensor.ranges.emplace_back(static_cast<double>(msg->ranges[i]));
    }

    for (auto i=359; i>=270; i--)
    {
      laser_sensor.ranges.emplace_back(static_cast<double>(msg->ranges[i]));
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


void Robot::base_pose_callback(nav_msgs::msg::Odometry::SharedPtr msg)
/**
 * This is the callback function to update the pose of the robot using error free gps
 * @param msg : the absolute truth of the pose of the robot as nav_msgs/Odometry
 */
{
  this->x = msg->pose.pose.position.x;
  this->y = msg->pose.pose.position.y;
  this->w = tf2::getYaw(msg->pose.pose.orientation);
}


void Robot::stop()
/**
 * Publishes the velocity command to STOP the robots.
 */
{
  this->cmd_vel_msg.linear.x = 0.0;
  this->cmd_vel_msg.angular.z = 0.0;
  pub_cmd_vel_->publish(this->cmd_vel_msg);
}


void Robot::move()
/**
 * the method to move the robot
 */
{
  // The function moves the robot according to the planner object

  // Tolerance to check the various angle and time conditions
  const double rad_tol = 2 * M_PI / 180;
  const double time_tol = 0.1;
  static NS_my_planner::MOTION_MODES currentMode = NS_my_planner::MOTION_MODES::START;
  
  // variables for obstacle avoidance
  static const double stopDist = 0.20;  // stopping distance of the robot (original 0.3)
  static const int avoidDuration = 50;  // duration to perform obstacle avoidance (original 10)
  static const double avoidSpeed = 0.01;  // original 0.05
  static const double avoidTurn = 0.175;  // original 0.5
  static const double minFrontDistance = 0.35;  // original 1.0
  bool obstruction = false;  // flag for detecting obstruction
  bool stop = false;  // flag to stop the robot

  // Get the laser data
  auto& laser_scan = laser_sensor.ranges;
  auto sample_count = laser_scan.size();
  if (verbose)
    RCLCPP_INFO(get_logger(), "Sample Count Laser: %lu", sample_count);


  // find the closest distance to the left and right and also check if
  // there's anything in front
  double minleft = 100.0;  // 1e6
  double minright = 100.0;  // 1e6
  double weight = 0.5;

  for (uint32_t i = 0; i < sample_count; i++)
  {
  // if (verbose && 0)
  //   printf("%.3f ", laser_scan[i]);

    if ((i > (sample_count / 3)) && (i < (sample_count - (sample_count / 3))) && laser_scan[i] < minFrontDistance)
    {
      // if (verbose)
      //   ROS_INFO("Obstruction!");
      //   puts("  obstruction!");
      obstruction = true;
    }
    // ROS_INFO("%.3f", laser_scan[i]);
    // if (laser_scan[i] < minFrontDistance) {
      // if (verbose)
      //  ROS_INFO("Obstruction!");
      // obstruction = true;
      // stop = true;
    // }

    if (laser_scan[i] < stopDist)
    {
      // if (verbose)
      //   ROS_INFO("Stopping!");
      //   puts("  stopping!");
      stop = true;
    }

    if (i < sample_count / 2)
    {
      minleft = std::min(minleft, laser_scan[i]);
    }
    else
    {
      minright = std::min(minright, laser_scan[i]);
    }
  }

  if (verbose)
  {
    RCLCPP_INFO(get_logger(), "min left: %.3f", minleft);
    RCLCPP_INFO(get_logger(), "min right: %.3f", minright);
  }

  if (obstruction || stop || (avoid_count > 0))
  {
    if (verbose)
      RCLCPP_INFO(get_logger(), "Avoid: %ld", avoid_count);

    // setting the robot to stop
    this->cmd_vel_msg.linear.x = stop ? 0.0 : avoidSpeed;
    pub_cmd_vel_->publish(this->cmd_vel_msg);

    // once we start avoiding, select a turn direction and stick
    // with it for a few iterations
    if (avoid_count < 1)
    {
      if (verbose)
        RCLCPP_INFO(get_logger(), "Avoid START");
      
      avoid_count = random() % (avoidDuration - 10);

      // right -; left +;
      if (minleft < minright)
      {
        set_turn_speed(-avoidTurn);
        if (verbose)
          RCLCPP_INFO(get_logger(), "Turning right: %.2f", -avoidTurn);
      }
      else
      {
        set_turn_speed(+avoidTurn);
        if (verbose)
          RCLCPP_INFO(get_logger(), "Turning left: %.2f", +avoidTurn);
      }
      publish();
    }

    minleft = 100.0;
    minright = 100.0;
    avoid_count--;

  }
  else
  {
    // if there is no obstruction follow the planned path
    if (verbose)
      RCLCPP_INFO(get_logger(), "Cruise; No Obstruction.");

    
  }
}


void Robot::set_turn_speed(double new_speed)
/**
 * Set the rotation speed of the robot in z direction
 * @param new_speed : rotation speed of the  robot
 */
{
  this->cmd_vel_msg.angular.z = new_speed;
}


void Robot::timer_callback()
{
  move();
  build_map();

  // to do some action every 20 seconds
  long unsigned int time_now = static_cast<long unsigned int>(this->now().seconds() - static_time);
  
  RCLCPP_INFO(this->get_logger(), "Time: %d", time_now);
  if (time_now % 30 == 0 && pre_time != time_now)
  {
    RCLCPP_INFO(this->get_logger(), "Writing Map");
    write_map_image();
    pre_time = time_now;
    count++;
  }
  update_neighbors();  // TODO: Gets hung up here
  publish_map();
  
  // Entropy and Coverage Maps
  add_map_entropy(static_time);  // Add static time stamp
  add_map_coverage(static_time);  // Add static time stamp

  if (count >= 20)  // Change back to 20 for experiment
  {
    stop_robot();
    write_map_entropy(data_path);
    write_map_coverage(data_path);
    rclcpp::shutdown();
  }

  RCLCPP_INFO(this->get_logger(), "-------------- END OF LOOP --------------");
}
