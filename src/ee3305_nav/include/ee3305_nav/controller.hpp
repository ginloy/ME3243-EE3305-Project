#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <ee3305_nav/ee3305_nav.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ee3305 {
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

  // ----------- States / Others -------------
  std::vector<double> path_flat;
  double rbt_x;
  double rbt_y;
  double rbt_h;
  double prev_time;

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

public:
  // Other public definitions
};
} // namespace ee3305

#endif
