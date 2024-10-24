// include statements allow the script to use external libraries or user-defined
// header files similar to import statements in python
#include <chrono>  // for time-related functions such as timing and duration
#include <cmath>   // for math operators like sqrt
#include <iomanip> // for manipulating i/o format, e.g. setting precision for floating point numbers
#include <iostream> // for i/o operations
#include <memory> // allows the use of smart pointers for automatic memory management
#include <vector> // provides the std::vector container, which is a dynamic array class

#include "ee3305_nav/ee3305_nav.hpp"
#include "ee3305_nav/planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals; // required for using the chrono literals
                                      // such as "ms" or "s"

// declaring a namespace encapsulates all definitions within the scope of this
// project prevents naming conflicts with external libraries or other parts of
// the program
namespace ee3305 {
// =================================================================================================
/** Handles the global path planning */
class Planner : public rclcpp::Node {

  // public/private labels define access control for members
  // (variables/functions) of a class public members can be accessed from
  // anywhere outside the class private members can only be accessed within the
  // class itself or within other functions/classes declared as friends
private:
  // ----------- Publishers / Subscribers --------------
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      sub_global_costmap;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;

  // // ----------- Services -------------------
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srv_get_plan;

  // ----------- Parameters ---------------

  // ----------- States / Others -------------
  std::vector<int8_t> map;
  double resolution;
  double origin_x;
  double origin_y;
  int rows;
  int cols;

public:
  /** Constructor. Run only once when this node is first created in `main()`. */
  explicit Planner() : rclcpp::Node("planner") {
    initStates();
    initParams();
    initTopics();
    initServices();
  }

private:
  void initStates() // void means this function does not return anything
  {
    resolution = 0;
    origin_x = 0;
    origin_y = 0;
    cols = 0;
  }

  /** Initializes and read parameters, if any. */
  void initParams() {}

  /** Initializes topics and messages, if any. */
  void initTopics() {
    auto qos = rclcpp::SensorDataQoS();
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.keep_last(1);

    sub_global_costmap =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "global_costmap", qos,
            std::bind(&Planner::cbGlobalCostmap, this, std::placeholders::_1));

    qos = rclcpp::SensorDataQoS();
    pub_path = this->create_publisher<nav_msgs::msg::Path>("path", qos);
  }

  void cbGlobalCostmap(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map = msg->data;
    resolution = msg->info.resolution;
    origin_x = msg->info.origin.position.x;
    origin_y = msg->info.origin.position.y;
    rows = msg->info.height;
    cols = msg->info.width;
  }

  void initServices() {
    srv_get_plan = this->create_service<nav_msgs::srv::GetPlan>(
        "get_plan",
        std::bind(&Planner::cbSrvGetPlan, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default);
  }

  /** Service server that returns a path in the map coordinates */
  void
  cbSrvGetPlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
               std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    RCLCPP_INFO_STREAM(this->get_logger(), "GetPlan service request received.");
    while (map.empty()) { // wait until a map is published
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "global_costmap topic is not published yet. Waiting for a global "
          "costmap to be published into the topic.");
      rclcpp::sleep_for(100ms);
      rclcpp::spin_some(this->get_node_base_interface());
    }

    double start_x = request->start.pose.position.x;
    double start_y = request->start.pose.position.y;
    double goal_x = request->goal.pose.position.x;
    double goal_y = request->goal.pose.position.y;

    int start_i = floor((start_x - origin_x) / resolution);
    int start_j = floor((start_y - origin_y) / resolution);
    int goal_i = floor((goal_x - origin_x) / resolution);
    int goal_j = floor((goal_y - origin_y) / resolution);

    std::vector<int> path_flat = run(start_i, start_j, goal_i, goal_j);

    // convert to map coordinates and write to response.
    // the response is from start to goal.
    // `path_flat` is from goal to start.
    nav_msgs::msg::Path path;
    // std::cout << "path: ";
    for (int p = path_flat.size() - 2; p >= 0; p -= 2) {
      double i = path_flat[p];
      double j = path_flat[p + 1];

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "map";
      pose_stamped.header.stamp = this->now();
      pose_stamped.pose.position.x =
          (i * resolution) + origin_x + resolution / 2;
      pose_stamped.pose.position.y =
          (j * resolution) + origin_y + resolution / 2;
      // std::cout << pose_stamped.pose.position.x << "," <<
      // pose_stamped.pose.position.y << ";  ";

      pose_stamped.pose.position.z = 0;
      pose_stamped.pose.orientation.x = 0;
      pose_stamped.pose.orientation.y = 0;
      pose_stamped.pose.orientation.z = 0;
      pose_stamped.pose.orientation.w = 1;

      path.poses.push_back(pose_stamped);
    }
    // std::cout << std::endl;
    path.header.frame_id = "map";

    // publish to topic
    pub_path->publish(path);

    // write to response
    response->plan = path;
  }

  /** The main path finding algorithm */
  std::vector<int> run(int start_i, int start_j, int goal_i, int goal_j) {
    // Returns index of flattened vector
    auto getIndex = [&](int i, int j) -> int { return j * cols + i; };

    // checks whether the given coordinates lie in the map
    auto inMap = [&](int i, int j) -> bool {
      return i >= 0 && i < cols && j >= 0 && j < rows;
    };

    // returns the euclidean distance between 2 cells
    auto eucDist = [&](int i1, int j1, int i2, int j2) -> double {
      return std::sqrt(std::pow(i1 - i2, 2) + std::pow(j1 - j2, 2));
    };

    // returns the cost penalty of the cell (ranges from 1 to 99)
    auto getPenalty = [&](int i, int j) -> double {
      return map[getIndex(i, j)];
    };

    std::vector<PlannerNode> nodes; // initialise all nodes as a flattened
                                    // vector of PlannerNode structs
    for (int j = 0; j < rows; ++j) {
      for (int i = 0; i < cols; ++i) {
        nodes.emplace_back(i, j);
      }
    }

    // * means the function returns a pointer to a PlannerNode object
    auto getNode = [&](int i, int j) -> PlannerNode * {
      return &nodes[getIndex(i, j)];
    };

    OpenList open_list; // initialise open_list

    std::vector<int> path = {}; // initialise empty path

    // initialise start node
    PlannerNode *node = getNode(start_i, start_j);
    node->g = 0;
    node->f = eucDist(start_i, start_j, goal_i, goal_j);
    open_list.queue(node);

    while (!open_list.empty()) {            // while open_list is not empty
      PlannerNode *node = open_list.poll(); // return first node in open_list
                                            // and remove it from open_list
      if (node->visited) {
        continue;
      }

      node->visited = true; // mark it as expanded

      if (node->i == goal_i &&
          node->j == goal_j) { // if current node is the goal
        PlannerNode *currentNode = node;

        // trace back all parent nodes from the goal to obtain the path
        while (currentNode->parent != nullptr) {
          if (currentNode->parent->i == start_i &&
              currentNode->parent->j == start_j) { // if parent is the start
            break;
          }
          // append node coordinates to the path
          path.push_back(currentNode->parent->i);
          path.push_back(currentNode->parent->j);

          currentNode =
              currentNode->parent; // reassign parent as current node to
                                   // search the parent nodes recursively
        }
      } else { // if current node is not the goal

        static const int di[] = {0, 0, 1, 1, -1, -1, 1, -1};
        static const int dj[] = {1, -1, 1, -1, 1, -1, 0, 0};
        for (int d = 0; d < 8; ++d) {
          // get neighbour coordinates
          int nb_i = (node->i) + di[d];
          int nb_j = (node->j) + dj[d];

          // if neighbour is not in the map
          if (!inMap(nb_i, nb_j)) {
            continue;
          }

          PlannerNode *nb_node = getNode(nb_i, nb_j);
          double nb_penalty = getPenalty(nb_i, nb_j);
          double new_nb_g_cost =
              node->g + nb_penalty * eucDist(nb_i, nb_j, node->i, node->j);

          // if this g cost is cheaper than the previously stored value
          if (new_nb_g_cost < nb_node->g) {

            // neighbour's parent becomes the current node
            nb_node->parent = node;

            // neighbour's g cost becomes the new g cost
            nb_node->g = new_nb_g_cost;
            nb_node->f = new_nb_g_cost + eucDist(nb_i, nb_j, goal_i, goal_j);
            open_list.queue(nb_node);
          }
        }
      }
    }
    return path;
  }
};
} // namespace ee3305

/** The main function to compile. */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ee3305::Planner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
