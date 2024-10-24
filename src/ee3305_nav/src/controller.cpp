#include "ee3305_nav/controller.hpp"
#include <memory>
#include <rclcpp/node.hpp>

using namespace ee3305;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ee3305::Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

Controller::Controller(): rclcpp::Node("controller") {
    initStates();
    initParams();
    initTopics();
    initServices();
    initTimers();
}

void Controller::initStates() {
    // TODO
}

void Controller::initParams() {
    // TODO
}

void Controller::initServices() {
    // TODO
}

void Controller::initTopics() {
    // TODO
}

void Controller::initTimers() {
    // TODO
}