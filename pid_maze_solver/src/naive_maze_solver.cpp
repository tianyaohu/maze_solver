#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ctrl_lib/controller.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "yaml-cpp/yaml.h"
#include <armadillo>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <streambuf>
#include <thread>

using namespace std;

class MazeSolver : public Controller {
public:
  // readPointsFromYAML() Constructor for the MazeSolver class
  MazeSolver() : Controller("cmd_vel", "amcl_pose", "naive_maze_solver_node") {
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

    // check if topic_pose is amcl, do some rotation for better pose estimation
    // and force new message from amcl (amcl only updates passively if there are
    // movements) is the pose source.
    if (getTopicPose().find("amcl") != std::string::npos) {
      updateAmclPoseWithRotation();
    }

    // Future TODO: instead of assuming always start from the very begining of
    // waypoint matrix, find the closest pt to the current pose and start from
    // that point to either end of the maze.

    // check if reverse solve

    cout << "before getting way points" << endl;

    arma::mat way_pts = getWayPoints();
    cout << "after getting way points" << endl;

    way_pts.each_row([this](const arma::rowvec &goal) {
      RCLCPP_INFO(this->get_logger(), "Turning Towards [%f, %f]", goal(0),
                  goal(1));
      this->makeAbsTurn(goal.t());
      RCLCPP_INFO(this->get_logger(), "Moving Towards [%f, %f]", goal(0),
                  goal(1));
      this->makeAbsMove(goal.t());
    });
  }

private:
  arma::mat getWayPoints() {
    // Compute the current position into an Armadillo row vector
    arma::rowvec current_pos = getCurXY().t();

    cout << "before distance" << endl;

    cout << "test " << way_pts_.each_row() - current_pos << endl;

    // Compute distances from current position to all waypoints
    arma::vec distances = arma::sqrt(
        arma::sum(arma::square(way_pts_.each_row() - current_pos), 1));

    cout << "after distance" << endl;

    distances.print("this is distance");

    // Find the index of the closest waypoint
    arma::uword closestIndex;
    distances.min(closestIndex); // This overload of min() returns the index of
                                 // the min value

    // visualize in terminal starting with nth index
    RCLCPP_INFO(this->get_logger(),
                "Starting from the waypoint at index: %llu (closest to current "
                "position)",
                static_cast<unsigned long long>(closestIndex));

    cout << "1" << endl;

    // Select waypoints starting from the closest, adjusting for reverse solving
    arma::mat selected_pts;
    if (reverse_solve_) {
      cout << "2" << endl;

      // Include waypoints from the closest to the start, in reverse order
      if (closestIndex > 0) {
        selected_pts = arma::flipud(way_pts_.rows(0, closestIndex));
      }
      cout << "3" << endl;

    } else {
      cout << "4" << endl;

      // For normal solving, include waypoints from the closest to the end
      selected_pts = way_pts_.rows(closestIndex, way_pts_.n_rows - 1);
      cout << "5" << endl;
    }

    return selected_pts;
  }

  void updateAmclPoseWithRotation() {
    RCLCPP_INFO(this->get_logger(), "Inducing movement for AMCL update...");

    // Rotate left a small amount (example: 10 degrees, converted to radians)
    this->makeDeltaTurn(M_PI / 8);
    this->makeDeltaTurn(-M_PI / 4);

    RCLCPP_INFO(this->get_logger(), "Movement induced for AMCL update.");
  }

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