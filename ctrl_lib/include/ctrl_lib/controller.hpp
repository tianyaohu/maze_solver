#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_lib/pid.hpp"
#include <armadillo>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
public:
  Controller(const string topic_vel_, const string topic_odom_,
             const string node_name);
  bool moveForXY(double x, double y);
  //   bool turnForDegrees(double degree);

  void makeDeltaTurn(const double &delta);
  void makeAbsTurn(const arma::vec &xy_goal);
  void makeDeltaMove(const arma::vec &delta);
  void makeAbsMove(const arma::vec &xy_goal);

private:
  string topic_vel_;
  string topic_odom_;

  std::unique_ptr<PID<double>> dist_pid;
  std::unique_ptr<PID<double>> turn_pid;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  // state info
  double cur_x, cur_y, cur_theta;

  int getSceneNum();
  void setTurnPID(int scene_num);
  void setDistPID(int scene_num);

  static inline double normalize(double angle);

  double calcTurnError(double abs_goal);
  arma::vec calcMoveError(arma::vec abs_goal);

  arma::vec calcTurnSpeed(double error);
  arma::vec calcMoveSpeed(arma::vec error);

  template <typename T, typename ErrorFunc, typename CommandFunc>
  void moveToMinimizeError(T abs_goal, ErrorFunc calc_error,
                           CommandFunc command_func);

  //   template <typename T>
  //   void moveToMinimizeError(T abs_goal, std::function<T(T)> calc_error,
  //                            std::function<arma::vec(T)> calc_speed);
  inline arma::vec getCurXY();

  // callbacks
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishSpeed(arma::vec speeds);
  void stop() { publishSpeed(arma::zeros<arma::vec>(3)); };
};

#endif // CONTROLLER_HPP