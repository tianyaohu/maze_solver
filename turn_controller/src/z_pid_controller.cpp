#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <armadillo>
#include <cmath>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "pid_lib/pid.hpp"

using namespace std;
using namespace std::chrono_literals;

class AngleController : public rclcpp::Node {
public:
  //   AngleController(double wb, double wd, double tw)
  AngleController(double kp, double ki, double kd)
      : Node("eight_trajectory") //{
        ,
        z_pid(kp, ki, kd) {

    // init publishers and subscribers
    wheel_speed_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&AngleController::odom_callback, this,
                  std::placeholders::_1));
  }

  void turn_to_z_goal(double goal) {
    // threshold between goal and error
    const double THRESH = 0.05;
    double error, speed;

    // Initialize error matrix with 1 rows (for theta errors)
    const size_t bufferSize = 10;
    arma::mat errorBuffer(1, bufferSize, arma::fill::zeros); // Start with zeros
    size_t errorIndex = 0; // To track the insertion point for new errors

    // init loop rate and goal reach flag
    rclcpp::Rate rate(30); // this affects control behavior
    bool goalReached = false;

    // reset pid to clear prev_time
    z_pid.reset();

    do {
      // process odoms
      rclcpp::spin_some(shared_from_this());

      // get current pos and error
      error = normalize(normalize(goal) - cur_theta);

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

      // calc and publish speed
      speed = z_pid.computeOutput(error);
      this->publishTurnSpeed(speed);

      rate.sleep();

    } while (rclcpp::ok() && !goalReached);

    // stop after reaching goal
    stop();

    rclcpp::sleep_for(2s);
    // process odoms
    rclcpp::spin_some(shared_from_this());

    // final pos after movement
    cout << "final arrival angle is " << cur_theta << endl;
    cout << "error for the final pos is " << goal - cur_theta << endl;
  }

private:
  static inline double normalize(double angle) {
    const double TWO_PI = 2.0 * M_PI;
    angle = fmod(angle + M_PI, TWO_PI);
    if (angle < 0)
      angle += TWO_PI;
    return angle - M_PI;
  }

  void stop() {
    double stop(0);
    publishTurnSpeed(stop);
  }

  // speed : theta
  void publishTurnSpeed(double speed) {
    // init message
    geometry_msgs::msg::Twist msg;
    msg.angular.z = speed;
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

  PID<double> z_pid;

  // robot physical info
  double wheel_base;
  double wheel_diameter;
  double track_width;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // ku here tested is with controller set at 30hz
  double ku = 15, tu = 8.50; // tu is in seconds
  // set kp, ki, kd
  double kp = 0.145 * ku;
  double ki = 0;
  //   double ki = 0.001 * kp / (0.5 * tu);
  double kd = 0.1 * ku * tu; // starting from 0.125

  std::shared_ptr<AngleController> node =
      std::make_shared<AngleController>(kp, ki, kd);

  int n = 3;         // vector length
  double start = 0;  // first element
  double end = M_PI; // last element

  arma::vec W = arma::linspace<arma::vec>(start, end, n);

  W.print("Show All Goals");
  // loop to move through all the way points

  for (double &goal_angle : W) {
    cout << "Turn Towards " << goal_angle << endl;
    node->turn_to_z_goal(goal_angle);
    // after finishing moving to waypoint, move back to origin
    node->turn_to_z_goal(0);
  }

  rclcpp::shutdown();
  return 0;
}