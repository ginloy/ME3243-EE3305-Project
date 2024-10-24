#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

namespace ee3305 {
class Controller : public rclcpp::Node {
private:
  // States and ROS2 Parameters, Actions, Topics, Timers and Services

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

public:
  // Other public definitions
};
} // namespace ee3305

#endif
