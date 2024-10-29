#include "ee3305_nav/controller.hpp"
#include "ee3305_nav/ee3305_nav.hpp"
#include <chrono>
#include <cmath>
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
  prev_time = this->now().seconds();
  prev_lin_vel = 0;
  prev_ang_vel = 0;
}

void Controller::initParams() {
  initParam(this, "frequency", frequency);
  initParam(this, "lookahead_distance", lookahead_distance);
  initParam(this, "stop_thres", stop_thres);
  initParam(this, "lookahead_lin_vel", lookahead_lin_vel);
  initParam(this, "max_lin_vel", max_lin_vel);
  initParam(this, "max_lin_acc", max_lin_acc);
  initParam(this, "max_ang_vel", max_ang_vel);
  initParam(this, "max_ang_acc", max_ang_acc);
}

void Controller::initServices() {
  // TODO
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
  timer_main = this->create_wall_timer(
      1s / frequency, std::bind(&Controller::cbTimerMain, this));
}

void Controller::cbPath(nav_msgs::msg::Path::SharedPtr msg) {
  path.clear();
  for (const geometry_msgs::msg::PoseStamped &pose : msg->poses) {
    path.emplace_back(pose.pose.position.x, pose.pose.position.y);
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
  if (path.empty() || std::isnan(rbt_x)) {
    return;
  }
// Calculate distance to the final point in the path
  Point<double> end_point = path.back();
  double dist_to_end = sqrt(pow(end_point.x - rbt_x, 2) + pow(end_point.y - rbt_y, 2));

  // Check if within stopping threshold
  if (dist_to_end < stop_thres) {
    // Gradually decelerate to stop
    double decel_factor = dist_to_end / stop_thres; // Scale down velocity based on distance to endpoint
    double lin_vel = prev_lin_vel * decel_factor;
    double ang_vel = prev_ang_vel * decel_factor;

    // Set previous velocities to zero to completely stop at the endpoint
    if (dist_to_end < 0.1) { // Consider a small threshold for complete stop
      lin_vel = 0;
      ang_vel = 0;
    }

    // Publish the decelerated velocities
    geometry_msgs::msg::Twist msg;
    msg.angular.z = ang_vel;
    msg.linear.x = lin_vel;
    pub_cmd_vel->publish(msg);
    return; // Exit early, so it doesn’t proceed with further calculations
  }
  // auto sgn = [](double v) -> double {

  // };

  // Get closest point that exceeds lookahead
  Point<double> closest = [&](){
    double smallestDist = INFINITY;
    int idx = -1;
    for (size_t i = 0; i < path.size(); ++i) {
      auto p = path[i];
      const auto dist = p.dist(Point(rbt_x, rbt_y));
      if (dist < smallestDist && dist > lookahead_distance) {
        smallestDist = dist;
        idx = i;
      }
    }
    return idx >= 0 ? path[idx] : Point<double>(INFINITY, INFINITY);
  }();
  if (closest.x == INFINITY) {
    return;
  }

  // Get elapsed time and update prev time
  double current_time = this->now().seconds();
  double elapsed_time = current_time - prev_time;
  prev_time = current_time;

  // Calculate the lookahead point’s coordinates in the robot frame.
  Point<double> lookPoint = [&](){
    double dx = closest.x - rbt_x;
    double dy = closest.y - rbt_y;

    double lookX = dx * cos(rbt_h) + dy * sin(rbt_h); 
    double lookY = dy * cos(rbt_h) - dx * sin(rbt_h); 
    
    return Point(lookX, lookY);
  }();

  // TODO Calculate curvature
  double curvature = 2 * lookPoint.y / (std::pow(lookPoint.x, 2) + std::pow(lookPoint.y, 2));

  // TODO Constrain linear acceleration
  double acc = (lookahead_lin_vel - prev_lin_vel) / elapsed_time;
  double lin_acc = acc > max_lin_acc ? max_lin_acc : acc;  

  // TODO Constrain linear velocity and update prev_lin_vel
  double vel = prev_lin_vel + lin_acc;
  double lin_vel = vel > max_lin_vel ? max_lin_vel : vel;
  prev_lin_vel = lin_vel;

  // TODO Calculate the desired angular velocity from the constrained linear velocity
  double ang_vel = curvature * lin_vel;

  // TODO Constrain angular acceleration
  double ang_acc = (ang_vel - prev_ang_vel) / elapsed_time;
  ang_acc = abs(ang_acc) > abs(max_ang_acc) ? (ang_acc < 0 ? -max_ang_acc : max_ang_acc) : ang_acc;

  // TODO Constrain angular velocity and update prev_ang_vel
  ang_vel = prev_ang_vel + ang_acc;
  ang_vel = abs(ang_vel) > abs(max_ang_vel) ? (ang_vel < 0 ? -max_ang_vel : max_ang_vel) : ang_vel;
  prev_ang_vel = ang_vel;

  std::cout << "Linear Velocity: " << lin_vel << std::endl;
  std::cout << "Angular Velocity: " << ang_vel << std::endl;
  

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
