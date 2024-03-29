#include "ctrl_lib/controller.hpp" // Adjust this include path to match your setup
#include <armadillo>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <streambuf>

using namespace std;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // init node
  auto node = std::make_shared<Controller>("cmd_vel", "odometry/filtered",
                                           "naive_maze_solver_node");

  // list of simulation way points
  arma::mat SIM_F = {{0.0, 0.0},    {0.51, -0.11}, {0.52, -1.34}, {1.03, -1.34},
                     {1.05, -0.87}, {1.38, -0.87}, {1.37, -0.34}, {2.0, -0.28},
                     {1.9, 0.55},   {1.51, 0.57},  {1.49, 0.17},  {1.01, 0.19},
                     {0.63, 0.55},  {0.19, 0.55}};

  arma::mat Reverse = arma::flipud(SIM_F);

  // ensure there is odom to work with
  rclcpp::sleep_for(200ms);
  rclcpp::spin_some(node);

  SIM_F.each_row([&](arma::rowvec goal) {
    cout << "Moving Towards " << goal << endl;

    // Turn by the specified delta angle
    // **no need for any sleep inbetween, sleep implemented within ctrl_lib
    node->makeAbsTurn(goal.t());
    node->makeAbsMove(goal.t());
  });

  rclcpp::shutdown();
  return 0;
}
