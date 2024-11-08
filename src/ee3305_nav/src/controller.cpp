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
  initParam(this, "curve_threshold", curve_threshold);
}

void Controller::initServices() {
  // Unused
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

void Controller::prunePath() {
  auto it = path.begin();
  auto currentPt = Point(rbt_x, rbt_y);
  auto closest = it;
  auto smallestDist = INFINITY;
  for (; it < path.end(); ++it) {
    auto dist = it->dist(currentPt);
    if (dist < smallestDist) {
      smallestDist = dist;
      closest = it;
    }
  }

  path = std::vector(closest, path.end());
}

void Controller::cbTimerMain() {
  if (path.empty() || std::isnan(rbt_x)) {
    return;
  }

  prunePath();

  Point<double> currentPt = Point(rbt_x, rbt_y);

  // Get closest point that exceeds lookahead
  Point<double> closest = [&]() {
    auto currentPt = Point(rbt_x, rbt_y);
    for (const auto &p : path) {
      if (p.dist(currentPt) > lookahead_distance) {
        return p;
      }
    }
    return *path.rbegin();
  }();

  double lookaheadError = closest.dist(currentPt) / lookahead_distance;
  double lin_vel = lookahead_lin_vel * (lookaheadError < 1.0 ? lookaheadError : 1.0);
  lin_vel = currentPt.dist(*path.rbegin()) < stop_thres ? 0.0 : lin_vel;

  // Calculate the lookahead point’s coordinates in the robot frame.
  Point<double> lookPoint = [&]() {
    double dx = closest.x - rbt_x;
    double dy = closest.y - rbt_y;

    double lookX = dx * cos(rbt_h) + dy * sin(rbt_h);
    double lookY = dy * cos(rbt_h) - dx * sin(rbt_h);

    return Point(lookX, lookY);
  }();


  // Calculate curvature
  double curvature =
      2 * lookPoint.y / (std::pow(lookPoint.x, 2) + std::pow(lookPoint.y, 2));

  std::cout << "Curvature: " << curvature << std::endl;

  if (abs(curvature) > curve_threshold) {
    lin_vel *= curve_threshold / abs(curvature);
  }
  double ang_vel = curvature * lin_vel;
  if (abs(atan2(lookPoint.y, lookPoint.x)) > M_PI / 3) {
    lin_vel = 0.0;
  }


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
