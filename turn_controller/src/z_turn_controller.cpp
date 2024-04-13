#include "ctrl_lib/controller.hpp" // Adjust this include path to match your setup
#include "rclcpp/logging.hpp"
#include <armadillo>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace std;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // init node
  std::shared_ptr<Controller> node = std::make_shared<Controller>(
      "cmd_vel", "odometry/filtered", "turn_controller_node");

  arma::vec D = {-M_PI / 2, M_PI / 2, M_PI, -M_PI};

  // init way point matrix
  arma::mat W;

  switch (node->scene_num) {
  case 0: // Simulation

    // init turn target way points
    W = {{0.52, -1.34}, {1.37, -0.34}, {0.63, 0.55}};
    break;

  case 1: // the Construct CyberLab

    // init turn target way points
    W = {{0.492054, -0.817005}, {-0.15507, -1.74964}};

    // simple 90 180 turn  test
    RCLCPP_INFO(node->get_logger(), "Forcing AMCL Update 1/2 ... ");
    node->makeDeltaTurn(M_PI / 2);

    RCLCPP_INFO(node->get_logger(), "Forcing AMCL Update 2/2 ... ");
    node->makeDeltaTurn(-M_PI / 2);

    RCLCPP_INFO(node->get_logger(), "Finished AMCL initialization ... ");
    break;

  default:
    RCLCPP_ERROR(node->get_logger(), "Invalid scene number: %d",
                 node->scene_num);
  }
  cout << node->scene_num << " is scene num" << endl;

  // ensure there is odom to work with
  rclcpp::sleep_for(200ms);
  rclcpp::spin_some(node);

  // simple 90 180 turn  test
  //   for (arma::uword i = 0; i < D.n_elem; ++i) {
  //     std::cout << "Element at index " << i << ": " << D(i) << std::endl;
  //     node->makeDeltaTurn(D(i));
  //   }

  // Loop through all delta angles
  W.each_row([&](arma::rowvec goal) {
    cout << "Turning Towards " << goal << endl;

    node->makeAbsTurn(goal.t());
  });

  rclcpp::shutdown();
  return 0;
}
