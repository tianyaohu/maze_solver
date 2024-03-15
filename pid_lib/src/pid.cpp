#include "pid_lib/pid.hpp"

template <typename T>
PID<T>::PID(const double kp, const double ki, const double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_time_(clock_.now()), integral_(T()),
      prev_error_(T()) {}

// Specialization for PID<double>
template <>
PID<double>::PID(const double kp, const double ki, const double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_time_(rclcpp::Clock().now()),
      integral_(0.0), prev_error_(0.0) {}

// Specialization for PID<arma::vec>
template <>
PID<arma::vec>::PID(const double kp, const double ki, const double kd,
                    const int vec_size)
    : kp_(kp), ki_(ki), kd_(kd), prev_time_(rclcpp::Clock().now()),
      integral_(arma::zeros(vec_size)), prev_error_(arma::zeros(vec_size)) {}

template <typename T> T PID<T>::computeOutput(const T error) {
  rclcpp::Time current_time = clock_.now();
  rclcpp::Duration duration = current_time - prev_time_;
  double dt = duration.seconds();

  // calc error
  integral_ += error * dt;
  T derivative = (error - prev_error_) / dt;
  prev_error_ = error;
  prev_time_ = current_time;

  //   std::cout << "error is" << error << std::endl;
  //   std::cout << "kp_ is" << kp_ << std::endl;
  //   std::cout << "kp_ * error is" << kp_ * error << std::endl;

  return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

// General reset method implementation
template <typename T> void PID<T>::reset() {
  integral_ = T(); // Assumes T can be zero-initialized
  prev_error_ = T();
  prev_time_ = clock_.now();
}

// Specialization for arma::vec
template <> void PID<arma::vec>::reset() {
  integral_.zeros();
  prev_error_.zeros();
  prev_time_ = clock_.now();
}

// Specialization for double
template <> void PID<double>::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  prev_time_ = clock_.now();
}

template class PID<arma::vec>;
