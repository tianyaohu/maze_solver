#include "ctrl_lib/controller.hpp" // Adjust this include path to match your setup
#include <armadillo>
#include <rclcpp/rclcpp.hpp>

using namespace std;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // init node
  auto node = std::make_shared<Controller>("cmd_vel", "odometry/filtered",
                                           "turn_controller_node");

  // Define delta angles (in radians) as turning objectives
  arma::vec W = {M_PI / 2, -M_PI / 2, M_PI,
                 -M_PI}; // Example: 90 degrees right, 90 degrees left, 180
                         // degrees right, 180 degrees left

  // ensure there is odom to work with
  rclcpp::sleep_for(200ms);
  //
  rclcpp::spin_some(node);

  // Loop through all delta angles
  for (double delta : W) {
    cout << "Turning by delta radians: " << delta << endl;

    // Turn by the specified delta angle
    node->makeDeltaTurn(delta);
  }

  rclcpp::shutdown();
  return 0;
}
