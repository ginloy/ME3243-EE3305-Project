#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ee3305 {

template <typename T> class Point {
public:
  T x;
  T y;

  Point(const T &x, const T &y): x(x), y(y) {}

  T dist(const Point<T> &other) const {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
  }
};

class Controller : public rclcpp::Node {
private:
  // States and ROS2 Parameters, Actions, Topics, Timers and Services
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;

  // ----------- Timers -------------------
  rclcpp::TimerBase::SharedPtr
      timer_main; // contains the timer that runs the main looping function at
                  // regular intervals.

  // ----------- Parameters ---------------
  double frequency;
  double lookahead_distance;
  double stop_thres;
  double lookahead_lin_vel;
  double max_lin_vel;
  double max_lin_acc;
  double max_ang_vel;
  double max_ang_acc;
  double curve_threshold;

  // ----------- States / Others -------------
  std::vector<Point<double>> path;
  double rbt_x;
  double rbt_y;
  double rbt_h;
  double prev_time;
  double prev_lin_vel;
  double prev_ang_vel;

public:
  // Constructors, Destructors, etc..

  explicit Controller();

private:
  // Initializations for ROS2 PATTs and states
  // Callback definitions required for any ROS2 PATTs

  void initStates();
  void initParams();
  void initTopics();
  void initServices();
  void initTimers();

  void cbPath(nav_msgs::msg::Path::SharedPtr msg);

  // transform assumes zero transform between `map` frame and `odom` frame.
  void cbOdom(nav_msgs::msg::Odometry::SharedPtr msg);

  void cbTimerMain();

  void prunePath();

public:
  // Other public definitions
};
} // namespace ee3305

#endif
