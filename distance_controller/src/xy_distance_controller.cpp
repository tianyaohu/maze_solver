#include "ctrl_lib/controller.hpp" // Adjust this include path to match your setup
#include <armadillo>
#include <rclcpp/rclcpp.hpp>

using namespace std;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // init node
  auto node = std::make_shared<Controller>("cmd_vel", "odometry/filtered",
                                           "distance_controller_node");

  arma::mat W = {
      {1, 0}, // Move to 1,0 then back to origin
      {2, 0}, // Move to 2,0 then back to origin
      {3, 0}, // Move to 3,0 then back to origin
      {4, 0}  // Move to 4,0 then back to origin
  };
  //   arma::mat W = {{0.5, 0}, {1, 0}, {1.5, 0}};
  //   arma::mat W = {{0, 0.5}, {0, 1}, {0, 1.5}, {0, 3}, {-1, -1}};

  // ensure there is odom to work with
  rclcpp::sleep_for(200ms);
  rclcpp::spin_some(node);

  W.each_row([&](arma::rowvec goal) {
    cout << "Moving Towards " << goal << endl;

    // Turn by the specified delta angle
    node->makeAbsMove(goal.t());

    rclcpp::sleep_for(1s);
    arma::vec negativeGoal = -1 * goal.t();

    // node->makeAbsMove(negativeGoal);
    node->makeAbsMove(arma::vec({0, 0}));
  });

  rclcpp::shutdown();
  return 0;
}
