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
  MazeSolver()
      : Controller("cmd_vel", "odom/filtered", "naive_maze_solver_node") {
    this->declare_parameter<bool>("reverse_solve", false);
    this->get_parameter("reverse_solve", reverse_solve_);
    readPointsFromYAML();
  }
  void solveMaze() {
    if (way_pts_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No waypoints available to solve the maze.");
      return;
    }

    std::cout << "Does topic pose include 'amcl'? "
              << (getTopicPose().find("amcl") != std::string::npos)
              << std::endl;

    if (getTopicPose().find("amcl") != std::string::npos) {
      updateAmclPoseWithRotation();
    }

    arma::mat way_pts = getWayPoints();
    if (way_pts.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No valid waypoints determined for navigation.");
      return;
    }

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
    arma::rowvec current_pos = getCurXY().t();

    std::cout << "current_pose is " << current_pos << std::endl;

    arma::mat distance_diffs = way_pts_.each_row() - current_pos;
    arma::vec distances =
        arma::sqrt(arma::sum(arma::square(distance_diffs), 1));
    distances.print("Distances:");

    arma::uword closestIndex;
    distances.min(closestIndex);

    RCLCPP_INFO(this->get_logger(),
                "Starting from the waypoint at index: %llu (closest to current "
                "position)",
                static_cast<unsigned long long>(closestIndex));

    if (reverse_solve_) {
      return arma::mat(arma::flipud(way_pts_.rows(0, closestIndex)));
    } else {
      return way_pts_.rows(closestIndex, way_pts_.n_rows - 1);
    }
  }

  void updateAmclPoseWithRotation() {
    RCLCPP_INFO(this->get_logger(), "Inducing movement for AMCL update...");

    // Rotate left a small amount (example: 10 degrees, converted to radians)
    this->makeDeltaTurn(M_PI / 8);
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

  arma::mat readPtMatrixFromYAML(const YAML::Node &config) {
    if (!config["points"]) {
      RCLCPP_ERROR(this->get_logger(), "Missing 'points' key in YAML file.");
      throw std::runtime_error("Missing 'points' key in YAML file.");
    }

    arma::mat pointsMatrix;
    int numValidPoints = 0;
    for (const auto &point : config["points"]) {
      if (point.IsSequence() && point.size() == 2) {
        numValidPoints++;
      }
    }

    pointsMatrix.set_size(numValidPoints, 2);
    int idx = 0;
    for (const auto &point : config["points"]) {
      if (point.IsSequence() && point.size() == 2) {
        pointsMatrix(idx, 0) = point[0].as<float>();
        pointsMatrix(idx, 1) = point[1].as<float>();
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