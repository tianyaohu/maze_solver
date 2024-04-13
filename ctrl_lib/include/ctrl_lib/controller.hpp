#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_lib/pid.hpp"
#include <armadillo>
#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
public:
  Controller(const string &topic_vel, const string &topic_pose,
             const string &node_name);

  void makeDeltaTurn(const double &delta);
  void makeAbsTurn(const arma::vec &xy_goal);
  void makeDeltaMove(const arma::vec &delta);
  void makeAbsMove(const arma::vec &xy_goal);

  // getter methods
  std::string getTopicPose() const { return topic_pose_; };

  // Get cur pose
  inline arma::vec getCurXY();

  int scene_num;

private:
  string topic_vel_;
  string topic_pose_;

  std::unique_ptr<PID<double>> dist_pid;
  std::unique_ptr<PID<double>> turn_pid;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::SubscriptionBase::SharedPtr sub_pose_;

  // state info
  double cur_x, cur_y, cur_theta;

  // init subscriber for localization (odom || amcl_pose)
  void setTopicPose();
  void setPoseSubscription();

  void setSceneNum();
  void setSceneNum(int scene_num);
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

  // For pose_callbacks to update position and orientation
  void updatePositionAndOrientation(const geometry_msgs::msg::Pose &pose);

  // callbacks
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void amclCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // speed methods
  void publishSpeed(arma::vec speeds);
  void stop() { publishSpeed(arma::zeros<arma::vec>(3)); };
};

#endif // CONTROLLER_HPP