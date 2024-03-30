#include "ctrl_lib/controller.hpp" // Adjust this include path to match your setup
#include <armadillo>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace std;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // init node
  auto node = std::make_shared<Controller>("cmd_vel", "odometry/filtered",
                                           "turn_controller_node");

  //   arma::vec D = {-M_PI / 2, M_PI / 2, M_PI, -M_PI};

  // define way_points here
  arma::mat W = {{0.52, -1.34}, {1.37, -0.34}, {0.63, 0.55}};

  // ensure there is odom to work with
  rclcpp::sleep_for(200ms);
  //
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
