#ifndef PID_HPP
#define PID_HPP

#include <armadillo>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

template <typename T> class PID {
public:
  PID(const double kp, const double ki, const double kd);
  PID(const double kp, const double ki, const double kd,
      const int vec_size); // Constructor for vector

  T computeOutput(const T error);

  void reset();

private:
  double kp_, ki_, kd_;
  rclcpp::Time prev_time_;
  rclcpp::Clock clock_;
  T integral_;
  T prev_error_;
};

#endif // PID_HPP