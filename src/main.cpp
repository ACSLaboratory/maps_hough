#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "maps_hough/command_line_parser.hpp"


int main(int argc, char** argv)
{
  // Initialize
  rclcpp::init(argc, argv);

  // Parse the inputs
  CommandLineParser cml_parser(argc, argv);

  // Parse input arguments for robot id.
  std::string robot_number;
  if (cml_parser["-id"])
  {
    robot_number = cml_parser("-id");
  }

  std::string no_of_robots;
  if (cml_parser["-n"])
  {
    no_of_robots = cml_parser("-n");
  }

  bool in_test_mode = true;
  if (cml_parser["-t"] && cml_parser("-t") == "false")
  {
    in_test_mode = false;
  }
  
  // The parameters for map object
  const double min_x = -1.38;        // in meters (original -8) // -1.3  // -1.5
  const double min_y = -1.07;        // in meters (original -8) // -0.8  // -1.5
  const double cell_size_x = 0.02;  // in meters (original 0.02)
  const double cell_size_y = 0.02;  // in meters (original 0.02)
  const int n_cell_x = 138;         // no of cells along x (original 800) // 130  // 150
  const int n_cell_y = 107;         // no of cells along y (original 800)  // 80  // 150

  // Create a map object
  map::occupancyGrid2D<double, int> occ_grid{
    min_x, min_y, cell_size_x, cell_size_y, n_cell_x, n_cell_y
  };

  // Robot speed parameters
  static const double cruisesSpeed = 0.05;  // 0.05 original 0.4
  static const double turnSpeed = 0.1;      // 0.1 original 0.2

  // Velocity object
  Velocity velocity;
  velocity.linear.x = cruisesSpeed;
  velocity.linear.y = 0;
  velocity.linear.z = 0;
  velocity.angular.x = 0;
  velocity.angular.y = 0;
  velocity.angular.z = turnSpeed;

  Pose startP{
    0.0, 0.0, 0.0
  };

  // Radial range sensor noise
  double radial_noise = 0.01;

  // Forward sensor model parameter
  NS_my_planner::F_S_M_parameters fsm;
  fsm.sigma = radial_noise;

  // Create a planner object
  NS_my_planner::base_planner planner{
    50, 0, startP, velocity
  };
  NS_my_planner::MI_levyWalk_planner MI_planner{
    0, startP, velocity, fsm, NS_my_planner::KLDMI, 5
  };

  // Create a robot object
  auto robot = std::make_shared<Robot>(
    static_cast<uint>(std::stoi(robot_number)), std::string{"turtlebot_0"}, &planner,
    static_cast<uint>(std::stoi(no_of_robots)), radial_noise, in_test_mode);

  // Assigning the map object to the robot
  robot->occ_grid_map = &occ_grid;

  rclcpp::spin(robot);
  rclcpp::shutdown();
  return 0;
}