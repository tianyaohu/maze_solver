#include "ctrl_lib/controller.hpp"

// Constructor implementation
Controller::Controller(const string topic_vel_, const string topic_odom_,
                       const string node_name = "movement_controller")
    : Node(node_name), topic_vel_(topic_vel_), topic_odom_(topic_odom_) {

  // Init pub and sub
  pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_vel_, 10);

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_odom_, 10,
      std::bind(&Controller::odomCallback, this, std::placeholders::_1));

  // Get scene number
  setSceneNum();

  // Set PIDs
  setTurnPID(scene_num);
  setDistPID(scene_num);
}

void Controller::setSceneNum() {
  // Declare the parameter with a default value, in case it's not set
  this->declare_parameter<int>("scene_num", 0); // 0 is simulation

  // print to see if parameter Setting workedf
  this->get_parameter("scene_num", scene_num);

  // visualize the scene number in terminal
  RCLCPP_INFO(this->get_logger(), "scene_num is: %d", scene_num);
}

void Controller::setTurnPID(int scene_num) {
  // Implement logic to set turn PID based on scene
  turn_pid = std::make_unique<PID<double>>(1.0, 0.0, 0.0); // Placeholder

  double ku, tu;
  double kp, ki, kd;
  // switch between scene based on num
  switch (scene_num) {
  case 0: // Simulation
    // init pid for Simulation
    ku = 15, tu = 8.5;

    kp = 0.12 * ku;
    ki = 0.00 * kp / (0.5 * tu);
    kd = 0.15 * ku * tu;
    break;
  // CyberWorld real robot lab @ the Construct
  case 1:
    ku = 5, tu = 3.50; // tu is in seconds

    kp = 0.27 * ku; // 0.15 is perfect turn in the real robot
    ki = 0.005 * kp / (0.5 * tu);
    kd = 0.018 * ku * tu; // starting from 0.125
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d", scene_num);
    return;
  }

  //  Init turn_pid
  turn_pid = std::make_unique<PID<double>>(kp, ki, kd);
}

void Controller::setDistPID(int scene_num) {
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
    kd = 0.1 * ku * tu;
    break;
  // CyberWorld the construct
  case 1:
    ku = 3, tu = 3; // tu is in seconds

    kp = 0.69 * ku;
    ki = 0.003 * kp / (0.5 * tu);
    kd = 0.155 * ku * tu;
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Invalid scene number: %d", scene_num);
    return;
  }

  //  Init z_pid
  dist_pid = std::make_unique<PID<double>>(kp, ki, kd);
}

// ########## GET STATE INFO ##############
inline arma::vec Controller::getCurXY() { return arma::vec({cur_x, cur_y}); }

// ################ Calc Turn Errors ##################
inline double Controller::normalize(double angle) {
  const double TWO_PI = 2.0 * M_PI;
  angle = fmod(angle + M_PI, TWO_PI);
  if (angle < 0)
    angle += TWO_PI;
  return angle - M_PI;
}

double Controller::calcTurnError(double abs_goal) {
  return normalize(abs_goal - cur_theta);
}

arma::vec Controller::calcTurnSpeed(double error) {
  return arma::vec({0, 0, turn_pid->computeOutput(error)});
}

// ################ Calc Turn Errors ##################

arma::vec Controller::calcMoveError(arma::vec abs_goal) {
  return abs_goal - getCurXY();
}

arma::vec Controller::calcMoveSpeed(arma::vec error) {
  double theta_error = normalize(atan2(error(1), error(0)) - cur_theta);

  // Decide go forward or backward based on theta_error, aka if robot's head is
  // more facing the goal or tail
  bool go_backward = fabs(theta_error) > M_PI / 2;

  // Adjust theta_error for backward movement
  if (go_backward) {
    // If we are going backward, adjust the direction of movement
    if (theta_error > 0) {
      theta_error = theta_error - M_PI;
    } else {
      theta_error = theta_error + M_PI;
    }
  }

  // calc speed
  double forward_speed = dist_pid->computeOutput(arma::norm(error));
  double turn_speed = turn_pid->computeOutput(theta_error);

  // If going backward, reverse the forward speed
  if (go_backward) {
    forward_speed = -forward_speed;
  }

  // To combat the challenge of moving through a narrow corridor, here we are
  // limiting forward speed based on theta error
  //  (1) as theta error approach 0, y approaches 1, aka go straight ahead
  // as abs(theta_error) gets greater, y approaches 0, aka first turn, before
  // moving forward
  auto y = [](double x) -> double {
    double aggro = 5.0; // increase aggro does more aggressive lin speed control
    // here y is 1-abs(2/(1+e^aggro*-x)-1)
    return 1 - std::abs(2.0 / (1.0 + std::exp(aggro * -x)) - 1.0);
  };

  return arma::vec({y(theta_error) * forward_speed, 0, turn_speed});
}

// ################### Optimization functions ###############
template <typename T, typename ErrorFunc, typename CommandFunc>
void Controller::moveToMinimizeError(T abs_goal, ErrorFunc calc_error,
                                     CommandFunc calc_speed) {

  //############### START OF TEMP ###################
  rclcpp::sleep_for(200ms);
  // process odoms
  rclcpp::spin_some(shared_from_this());
  //############### END OF TEMP ###################

  // init error
  T error;

  // init thresh hold
  const double THRESH = 0.05;

  // Initialize error matrix with 2xbuffersize
  const size_t bufferSize = 10;
  arma::mat errorBuffer;
  if constexpr (std::is_same<T, double>::value) {
    // If T is double, length of error buffer should be 1
    errorBuffer.set_size(1, bufferSize);
    errorBuffer.fill(arma::fill::zeros);
  } else if constexpr (std::is_same<T, arma::vec>::value) {
    // If T is arma::vec, length of error buffer should be length of abs_goal
    // times buffer_size
    size_t abs_goal_length =
        abs_goal.n_elem; // Getting the length of the abs_goal vector
    errorBuffer.set_size(abs_goal_length, bufferSize);
    errorBuffer.fill(arma::fill::zeros);
  } else {
    // Handle unexpected types if necessary
    static_assert(std::is_same<T, double>::value ||
                      std::is_same<T, arma::vec>::value,
                  "Unsupported type for T. T must be double or arma::vec.");
  }

  // init loop rate and abs_goal reach flag
  rclcpp::Rate rate(30); // this affects control behavior
  bool goalReached = false;

  size_t errorIndex = 0; // To track the insertion point for new errors
  do {
    // process odoms
    rclcpp::spin_some(shared_from_this());

    // calc error
    error = calc_error(abs_goal);

    // Insert new error into the matrix at the current index, rolling if
    // necessary
    errorBuffer.col(errorIndex) = error;
    errorIndex = (errorIndex + 1) % bufferSize;

    // GOAL REACH CONDITION:
    // (1) all error values in the buffer are below the threshold
    bool all_within_thresh =
        //   arma::all(arma::vectorise(arma::abs(errorBuffer.row(0))) <
        //   THRESH);
        arma::all(arma::vectorise(arma::abs(errorBuffer)) < THRESH);

    // (2) check the max difference within buffer
    arma::vec minVec = arma::min(errorBuffer, 1); // 1 for column-wise operation
    arma::vec maxVec = arma::max(errorBuffer, 1); // 1 for column-wise operation
    // Calculate the squared error between the minVec and maxVec
    arma::vec squaredError = arma::square(maxVec - minVec);
    bool sq_error_within_thresh =
        arma::all(arma::vectorise(arma::abs(squaredError)) < THRESH);

    goalReached = all_within_thresh && sq_error_within_thresh;

    // calc and publish speeds
    arma::vec speeds = calc_speed(error);
    this->publishSpeed(speeds);

    rate.sleep();
  } while (rclcpp::ok() && !goalReached);

  // stop after reaching abs_goal
  stop();

  rclcpp::sleep_for(2s);
  // process odoms
  rclcpp::spin_some(shared_from_this());

  /// ############ TO BE DELETED ##############
  // get current pos and error
  arma::vec cur_pos = {cur_x, cur_y};
  error = calc_error(abs_goal);

  // final pos after movement
  cur_pos.print("final arrival pos is ");
  cout << "error for the final pos is " << error << endl;
  /// ############ END OF TO BE DELETED ##############
}

// User Functions For movements
void Controller::makeDeltaTurn(const double &delta) {
  double abs_goal = normalize(delta + cur_theta);
  cout << "abs_goal is " << abs_goal << endl;
  moveToMinimizeError(
      abs_goal,
      std::bind(&Controller::calcTurnError, this, std::placeholders::_1),
      std::bind(&Controller::calcTurnSpeed, this, std::placeholders::_1));
}

void Controller::makeAbsTurn(const arma::vec &xy_goal) {
  double delta = atan2(xy_goal(1) - cur_y, xy_goal(0) - cur_x);

  xy_goal.print("turning towards ");
  cout << "delta is " << delta << endl;
  moveToMinimizeError(
      delta, std::bind(&Controller::calcTurnError, this, std::placeholders::_1),
      std::bind(&Controller::calcTurnSpeed, this, std::placeholders::_1));
}

void Controller::makeAbsMove(const arma::vec &xy_goal) {
  cout << "xy_goal is " << xy_goal << endl;
  // Use std::bind to bind this pointer for the member function calcMoveError
  moveToMinimizeError(
      xy_goal,
      std::bind(&Controller::calcMoveError, this, std::placeholders::_1),
      std::bind(&Controller::calcMoveSpeed, this, std::placeholders::_1));
}

void Controller::makeDeltaMove(const arma::vec &delta) {
  arma::vec xy_goal = getCurXY() + delta;
  cout << "xy_goal is " << xy_goal << endl;
  makeAbsMove(xy_goal);
}

// ################# CALLBACKS AND PUBS ###################
void Controller::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Update current state from odometry message
  cur_x = msg->pose.pose.position.x;
  cur_y = msg->pose.pose.position.y;

  double roll, pitch; // roll pitch is not used here
  // Convert quaternion to Euler angles
  tf2::Quaternion quat;
  // Get qua from msg
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, cur_theta); // ignoring roll & pitch
}

void Controller::publishSpeed(arma::vec speeds) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = speeds(0);
  msg.linear.y = speeds(1);
  msg.angular.z = speeds(2);
  pub_vel_->publish(msg);
}