#include "ctrl_lib/controller.hpp" // Adjust this include path to match your setup
#include <armadillo>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <streambuf>

#include "ament_index_cpp/get_package_share_directory.hpp" // Include this header
#include "geometry_msgs/msg/point.hpp"
#include "yaml-cpp/yaml.h" // include the yaml library
#include <filesystem>      // Include the filesystem library

using namespace std;

// The class init, we add a scene input and a call to a new function
class MazeSolver : public Controller {
public:
  // readPointsFromYAML() Constructor for the MazeSolver class
  MazeSolver()
      : Controller("cmd_vel", "odometry/filtered", "naive_maze_solver_node") {
    // Declare and get the reverse_solve parameter
    this->declare_parameter<bool>("reverse_solve", false);
    this->get_parameter("reverse_solve", reverse_solve_);

    // Read points from YAML file and store them in way_pts_
    readPointsFromYAML();
  }

  void solveMaze() {
    if (way_pts_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No waypoints available to solve the maze.");
      return;
    }

    arma::mat temp_pts = reverse_solve_ ? arma::flipud(way_pts_) : way_pts_;

    temp_pts.each_row([this](const arma::rowvec &goal) {
      RCLCPP_INFO(this->get_logger(), "Turning Towards [%f, %f]", goal(0),
                  goal(1));
      this->makeAbsTurn(goal.t());
      RCLCPP_INFO(this->get_logger(), "Moving Towards [%f, %f]", goal(0),
                  goal(1));
      this->makeAbsMove(goal.t());
    });
  }

private:
  void readPointsFromYAML() {
    // Get the package's share directory and append the YAML file path
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory(
            "pid_maze_solver"); // Replace with your package name

    std::string waypoint_file_name;

    switch (scene_num) {
    case 0: // Simulation
      waypoint_file_name = "sim_points.yaml";
      break;

    case 1: // CyberWorld
      waypoint_file_name = "cyberworld_points.yaml";
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d", scene_num);
    }

    RCLCPP_INFO(this->get_logger(), "############# Waypoint file loaded: %s",
                waypoint_file_name.c_str());

    std::string yaml_file =
        package_share_directory + "/waypoints/" + waypoint_file_name;

    try {
      YAML::Node config = YAML::LoadFile(yaml_file);
      way_pts_ = readPtMatrixFromYAML(config);
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read YAML file: %s",
                   e.what());
      throw; // Propagate exception or handle it as per your application's needs
    }
  }

  // Assuming YAML and other necessary libraries are included
  arma::mat readPtMatrixFromYAML(const YAML::Node &config) {
    if (!config["points"]) {
      RCLCPP_ERROR(this->get_logger(), "Missing 'points' key in YAML file.");
      throw std::runtime_error("Missing 'points' key in YAML file.");
    }

    arma::mat pointsMatrix;
    int numValidPoints = 0; // Track the number of valid points

    for (const auto &point : config["points"]) {
      if (point.IsSequence() && point.size() == 2) {
        numValidPoints++;
      } else {
        RCLCPP_WARN(this->get_logger(), "Skipping invalid point format.");
      }
    }

    pointsMatrix.set_size(numValidPoints,
                          2); // Allocate matrix to fit valid points only
    int idx = 0;
    for (const auto &point : config["points"]) {
      if (point.IsSequence() && point.size() == 2) {
        float x = point[0].as<float>();
        float y = point[1].as<float>();
        pointsMatrix(idx, 0) = x;
        pointsMatrix(idx, 1) = y;
        idx++;
      }
    }

    return pointsMatrix;
  }

  arma::mat way_pts_;
  bool reverse_solve_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // init node
  auto node = std::make_shared<MazeSolver>();

  // forward solve maze
  node->solveMaze();

  rclcpp::shutdown();
  return 0;
}
