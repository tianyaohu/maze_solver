#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <armadillo>
#include <cmath>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "pid_lib/pid.hpp"

using namespace std;
using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node {
public:
  //   DistanceController(double wb, double wd, double tw)
  DistanceController() : Node("distance_controller") {

    int scene_num;

    // Declare the parameter with a default value, in case it's not set
    this->declare_parameter<int>("scene_num", 0); // 0 is simulation

    // print to see if parameter Setting workedf
    this->get_parameter("scene_num", scene_num);
    RCLCPP_INFO(this->get_logger(), "scene_num is: %d", scene_num);

    double ku, tu;
    double kp, ki, kd;
    // switch between scene based on num
    switch (scene_num) {
    case 0:               // Simulation
                          // ku here tested is with controller set at 30hz
      ku = 15, tu = 8.50; // tu is in seconds
                          // set kp, ki, kd
      kp = 0.125 * ku;
      ki = 0.001 * kp / (0.5 * tu);
      kd = 0.075 * ku * tu;
      break;
    // CyberWorld the construct
    case 1:
      // THE FOLLOWING ARE LEGACY OF Z_CONTROLLER
      ku = 5, tu = 3; // tu is in seconds

      kp = 0.1 * ku; // 0.15 is perfect turn in the real robot
      ki = 0.0;      // 05 * kp / (0.5 * tu);
      kd = 0.0;      // 18 * ku * tu; // starting from 0.125
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d", scene_num);
      return;
    }

    //  Init z_pid
    xy_pid = std::make_unique<PID<arma::vec>>(kp, ki, kd, 2);

    // init publishers and subscribers
    wheel_speed_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&DistanceController::odom_callback, this,
                  std::placeholders::_1));
  }

  void move_to_xy_goal(arma::vec goal) {
    // threshold between goal and cur_pos
    const double THRESH = 0.05;
    arma::vec error, cur_pos;

    // Initialize error matrix with 2 rows (for x and y errors) and bufferSize
    // columns
    const size_t bufferSize = 10;
    arma::mat errorBuffer(2, bufferSize, arma::fill::zeros); // Start with zeros

    // init loop rate and goal reach flag
    rclcpp::Rate rate(30); // this affects control behavior
    bool goalReached = false;

    // reset pid to clear prev_time
    xy_pid->reset();

    size_t errorIndex = 0; // To track the insertion point for new errors

    do {
      // process odoms
      rclcpp::spin_some(shared_from_this());

      // get current pos and error
      cur_pos = {cur_x, cur_y};
      error = goal - cur_pos;

      // Insert new error into the matrix at the current index, rolling if
      // necessary
      errorBuffer.col(errorIndex) = error;
      errorIndex = (errorIndex + 1) % bufferSize;

      // GOAL REACH CONDITION:
      // (1) all error values in the buffer are below the threshold
      bool all_within_thresh =
          arma::all(arma::vectorise(arma::abs(errorBuffer.row(0))) < THRESH);
      // (2) check the max difference within buffer
      arma::vec minVec =
          arma::min(errorBuffer, 1); // 1 for column-wise operation
      arma::vec maxVec =
          arma::max(errorBuffer, 1); // 1 for column-wise operation
      // Calculate the squared error between the minVec and maxVec
      arma::vec squaredError = arma::square(maxVec - minVec);
      bool sq_error_within_thresh =
          arma::all(arma::vectorise(arma::abs(squaredError)) < THRESH);

      //   cout << "minVec is " << minVec << endl;
      //   cout << "maxVec is " << maxVec << endl;
      //   cout << "squaredError is " << squaredError << endl;
      //   cout << "sq_error_within_thresh is " << sq_error_within_thresh <<
      //   endl; cout << "all_within_thresh is " << all_within_thresh << endl;

      //   goalReached = arma::all(arma::vectorise(arma::abs(errorBuffer)) <
      //   THRESH);
      goalReached = all_within_thresh && sq_error_within_thresh;

      // calc and publish speeds
      arma::vec speeds = xy_pid->computeOutput(error);
      this->publishSpeeds(speeds);

      rate.sleep();

    } while (rclcpp::ok() && !goalReached);

    // stop after reaching goal
    stop();

    rclcpp::sleep_for(2s);
    // process odoms
    rclcpp::spin_some(shared_from_this());

    // get current pos and error
    cur_pos = {cur_x, cur_y};

    // final pos after movement
    cout << "final arrival pos is " << cur_pos << endl;
    cout << "error for the final pos is " << error << endl;
  }

private:
  void stop() {
    arma::vec stop(3, arma::fill::zeros);
    publishSpeeds(stop);
  }

  // speeds : {theta, x, y}
  void publishSpeeds(arma::vec speeds) {
    // init message
    geometry_msgs::msg::Twist msg;
    // msg.angular.z = speeds(0);
    msg.linear.x = speeds(0);
    msg.linear.y = 0.2 * speeds(1);

    speeds.print("speed is ");

    // pub to cmd_vel
    wheel_speed_pub_->publish(msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double roll, pitch; // roll pitch is not used here
    // Convert quaternion to Euler angles
    tf2::Quaternion quat;
    // Get qua from msg
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, cur_theta);

    // get x,y
    cur_x = msg->pose.pose.position.x;
    cur_y = msg->pose.pose.position.y;
  }

  // ROS2 sub & pub
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_speed_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // twist2wheel matrix
  arma::mat K;

  double cur_x, cur_y, cur_theta;

  std::unique_ptr<PID<arma::vec>> xy_pid;

  // robot physical info
  double wheel_base;
  double wheel_diameter;
  double track_width;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<DistanceController> node =
      std::make_shared<DistanceController>();

  //   arma::mat W = {{1, 0}, {2, 0}}; //{3, 0}, {4, 0}};
  arma::mat W = {{-0.5, 0}, {0, 0}, {0.5, 0}, {1, 0}, {1.5, 0}};

  // loop to move through all the way points
  W.each_row([&](arma::rowvec &way_pt) {
    cout << "Moving Towards " << way_pt << endl;
    node->move_to_xy_goal(way_pt.t());
    // after finishing moving to waypoint, move back to origin
    node->move_to_xy_goal(arma::vec({-0.5, 0}));
  });

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}