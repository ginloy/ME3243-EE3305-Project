#include "ee3305_nav/controller.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

using namespace ee3305;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ee3305::Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

Controller::Controller() : rclcpp::Node("controller") {
  initStates();
  initParams();
  initTopics();
  initServices();
  initTimers();
}

void Controller::initStates() {
  rbt_x = NAN;
  rbt_y = NAN;
  rbt_h = NAN;
}

void Controller::initParams() {
  initParam(this, "frequency", frequency);
  initParam(this, "lookahead_distance", lookahead_distance);
  initParam(this, "stop_thres", stop_thres);
  initParam(this, "lookahead_lin_vel", lookahead_lin_vel);
}

void Controller::initServices() {
  timer_main = this->create_wall_timer(
      1s / frequency, std::bind(&Controller::cbTimerMain, this));
}

void Controller::initTopics() {
  sub_path = this->create_subscription<nav_msgs::msg::Path>(
      "path", rclcpp::SensorDataQoS(),
      std::bind(&Controller::cbPath, this, std::placeholders::_1));

  sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&Controller::cbOdom, this, std::placeholders::_1));

  pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS());
}

void Controller::initTimers() {
  // TODO
}

void Controller::cbPath(nav_msgs::msg::Path::SharedPtr msg) {
  plan_flat.clear();
  for (const geometry_msgs::msg::PoseStamped &pose : msg->poses) {
    plan_flat.push_back(pose.pose.position.x);
    plan_flat.push_back(pose.pose.position.y);
  }
}

void Controller::cbOdom(nav_msgs::msg::Odometry::SharedPtr msg) {
  rbt_x = msg->pose.pose.position.x;
  rbt_y = msg->pose.pose.position.y;
  const auto &q = msg->pose.pose.orientation;
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y + q.y + q.z * q.z);
  rbt_h = atan2(siny_cosp, cosy_cosp);
}

void Controller::cbTimerMain() {
  double ang_vel = 0.0; // TODO
  double lin_vel = 0.0; // TODO

  // Publish message
  geometry_msgs::msg::Twist msg;
  msg.angular.x = 0; // redundancy
  msg.angular.y = 0; // redundancy
  msg.angular.z = ang_vel;
  msg.linear.x = lin_vel;
  msg.linear.y = 0; // redundancy
  msg.linear.z = 0; // redundancy
  pub_cmd_vel->publish(msg);
}
