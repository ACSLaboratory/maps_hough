#include "maps_hough/robot.hpp"


using namespace std::chrono_literals;


Robot::Robot(uint robot_id_,
             std::string robot_name_,
             std::string srv_name_nbh,
             uint no_of_robots,
             double radial_noise_,
             double sensing_radius_):
    Node("robot_node"),
    id_{robot_id_},
    name_{robot_name_},
    x{0.0},
    y{0.0},
    w{0.0},
    sensing_radius_{sensing_radius_},
    last_communication_(no_of_robots),
    avoid_count_{0},
    rand_count_{0}

/**
 * Constructor for the robot class
 * @param robot_id_ : the id number of the robot
 * @param robot_name_ : the common name for the robot
 * @param radial_noise_ : the variance of the radial noise of the range sensor
 * @param sensing_radius_ : the sensing radius of the robot
 */
{
  std:: string robot_name_aug = robot_name_ + std::to_string(robot_id_);
  laser_sensor_info_.range_noise_const = radial_noise_;

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
      RCLCPP_INFO(this->get_logger(), "TEST UMC%d Size of nbh_ids: %d", i, nbh_ids_.size());
      if (!nbh_ids_.empty() && std::find(nbh_ids_.begin(), nbh_ids_.end(), i) != nbh_ids_.end())
      {
        RCLCPP_INFO(get_logger(), "Updating Map");
        
        // check if ample time has past since the map merger
        if (last_communication_[i] + comm_delay_ < this->now().seconds())
        {
          RCLCPP_INFO(get_logger(), "Merging the maps of robots %d and %d", id_, i);
          merger(occ_grid_map->og_, cv_bridge::toCvShare(msg, "mono8")->image);
          last_communication_[i] = this->now().seconds();
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
  srv_nbh_client_ = this->create_client<levy_walk_msgs::srv::Neighbors>(srv_name_nbh);

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
  if (laser_sensor_info_.ranges.size() == 0)  // initialize the object
  {
    laser_sensor_info_.angle.max = msg->angle_max;
    laser_sensor_info_.angle.min = msg->angle_min;
    laser_sensor_info_.range.max = 0.4;  // msg->range_max;
    laser_sensor_info_.range.min = msg->range_min;
    double temp_range = 0.0;
    
    for (double b = 100 * msg->angle_increment; b >= msg->angle_min; b -= msg->angle_increment)
    {
      laser_sensor_info_.bearings.emplace_back(b);
    }
    
    for (double b = msg->angle_max; b >= 260 * msg->angle_increment; b -= msg->angle_increment)
    {
      laser_sensor_info_.bearings.emplace_back(b);
    }

    int no_of_beams = ((msg->angle_max-msg->angle_min)/msg->angle_increment+1) / 2;
    
    for (auto i=90; i>=0; i--)
    {
      laser_sensor_info_.ranges.emplace_back(static_cast<double>(msg->ranges[i]));
    }

    for (auto i=359; i>=270; i--)
    {
      laser_sensor_info_.ranges.emplace_back(static_cast<double>(msg->ranges[i]));
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
        laser_sensor_info_.ranges[i] = 0.4;
      // This `else if` statement is a way to filter 0 readings by the lidar
      }
      else if (msg->ranges[j] <= 0.02)
      {
        // laser_sensor.ranges[i] = 0.36;
        for (int k=1; k<=3; k++)
        {
          if (msg->ranges[j+k] > 0.02)
          {
            laser_sensor_info_.ranges[i] = (msg->ranges[j+k] > 0.4) ? 0.4 : msg->ranges[j+k];
            break;
          }
          else if (k == 3)
          {
            laser_sensor_info_.ranges[i] = 0.36;
          }
        }
      }
      else
      {
        laser_sensor_info_.ranges[i] = msg->ranges[j];
      }
      j--;
    }

    j = 359;
    for (auto i=91; i<=180; i++)
    {
      // ROS_INFO("laser sensor %d: %.3f", i, msg->ranges[i]);
      if (msg->ranges[j] > 0.4)
      {
        laser_sensor_info_.ranges[i] = 0.4;
      // This `else if` statement is a way to filter 0 readings by the lidar
      }
      else if (msg->ranges[j] <= 0.02)
      {
        // laser_sensor.ranges[i] = 0.36;
        for (int k=1; k<=3; k++)
        {
          if (msg->ranges[j-k] > 0.02)
          {
            laser_sensor_info_.ranges[i] = (msg->ranges[j-k] > 0.4) ? 0.4 : msg->ranges[j-k];
            break;
          }
          else if (k == 3)
          {
            laser_sensor_info_.ranges[i] = 0.36;
          }
        }
      }
      else
      {
        laser_sensor_info_.ranges[i] = msg->ranges[j];
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
  // Variables for obstacle avoidance
  static const double stop_dist = 0.20;           // stopping distance of the robot (original 0.3)
  static const int avoid_duration = 50;           // duration to perform obstacle avoidance (original 10)
  static const double avoid_speed = 0.01;         // original 0.05
  static const double avoid_turn = 0.175;         // original 0.5
  static const double min_front_distance = 0.35;  // original 1.0
  bool obstruction = false;                       // flag for detecting obstruction
  bool stop = false;                              // flag to stop the robot

  // Get the laser data
  auto& laser_scan = laser_sensor_info_.ranges;
  auto sample_count = laser_sensor_info_.ranges.size();

  if (verbose)
    RCLCPP_INFO(get_logger(), "Sample Count Laser: %lu", sample_count);


  // find the closest distance to the left and right and also check if
  // there's anything in front
  double min_left = 100.0;  // 1e6
  double min_right = 100.0;  // 1e6
  double weight = 0.5;

  for (uint32_t i = 0; i < sample_count; i++)
  {
    if ((i > (sample_count / 3)) && (i < (sample_count - (sample_count / 3))) && laser_scan[i] < min_front_distance)
      obstruction = true;

    if (laser_scan[i] < stop_dist)
      stop = true;

    if (i < sample_count / 2)
      min_left = std::min(min_left, laser_scan[i]);
    else
      min_right = std::min(min_right, laser_scan[i]);
  }

  if (verbose)
  {
    RCLCPP_INFO(get_logger(), "min left: %.3f", min_left);
    RCLCPP_INFO(get_logger(), "min right: %.3f", min_right);
  }

  if (obstruction || stop || (avoid_count_ > 0))
  {
    if (verbose)
      RCLCPP_INFO(get_logger(), "Avoid: %ld", avoid_count_);

    // setting the robot to stop
    this->cmd_vel_msg.linear.x = stop ? 0.0 : avoid_speed;
    pub_cmd_vel_->publish(this->cmd_vel_msg);

    // once we start avoiding, select a turn direction and stick
    // with it for a few iterations
    if (avoid_count_ < 1)
    {
      if (verbose)
        RCLCPP_INFO(get_logger(), "Avoid START");
      
      avoid_count_ = random() % (avoid_duration - 10);

      // right -; left +;
      if (min_left < min_right)
      {
        set_turn_speed(-avoid_turn);
        if (verbose)
          RCLCPP_INFO(get_logger(), "Turning right: %.2f", -avoid_turn);
      }
      else
      {
        set_turn_speed(+avoid_turn);
        if (verbose)
          RCLCPP_INFO(get_logger(), "Turning left: %.2f", +avoid_turn);
      }
      pub_cmd_vel_->publish(this->cmd_vel_msg);
    }

    min_left = 100.0;
    min_right = 100.0;
    avoid_count_--;

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


void Robot::update_neighbors()
/**
 * the method to update the neighbours of a robot using a ros2 service
 */
{
  while (!srv_nbh_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  
  auto request = std::make_shared<levy_walk_msgs::srv::Neighbors::Request>();
  request->robot_id = id_;
  request->radius = sensing_radius_;

  using ServiceResponseFuture = rclcpp::Client<levy_walk_msgs::srv::Neighbors>::SharedFuture;
  
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "Result recieved from server!");
    nbh_ids_.assign(result->neighbors.begin(), result->neighbors.end());
  };
  

  auto future_result = srv_nbh_client_->async_send_request(request, response_received_callback);
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
