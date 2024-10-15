#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ee3305_nav/ee3305_nav.hpp"
#include "ee3305_nav/planner.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    // =================================================================================================
    /** Handles the global path planning */
    class Planner : public rclcpp::Node
    {

    private:
        // ----------- Publishers / Subscribers --------------
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_costmap;
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
        explicit Planner()
            : rclcpp::Node("planner")
        {
            initStates();
            initParams();
            initTopics();
            initServices();
        }

    private:
        void initStates()
        {
            resolution = 0;
            origin_x = 0;
            origin_y = 0;
            cols = 0;
        }

        /** Initializes and read parameters, if any. */
        void initParams()
        {
        }

        /** Initializes topics and messages, if any. */
        void initTopics()
        {
            auto qos = rclcpp::SensorDataQoS();
            qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            qos.keep_last(1);

            sub_global_costmap = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "global_costmap", qos,
                std::bind(&Planner::cbGlobalCostmap, this, std::placeholders::_1)); 

            qos = rclcpp::SensorDataQoS();
            pub_path = this->create_publisher<nav_msgs::msg::Path>(
                "path", qos);
        }

        void cbGlobalCostmap(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            map = msg->data;
            resolution = msg->info.resolution;
            origin_x = msg->info.origin.position.x;
            origin_y = msg->info.origin.position.y;
            rows = msg->info.height;
            cols = msg->info.width;
        }

        void initServices()
        {
            srv_get_plan = this->create_service<nav_msgs::srv::GetPlan>(
                "get_plan",
                std::bind(&Planner::cbSrvGetPlan, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default);
        }

        /** Service server that returns a path in the map coordinates */
        void cbSrvGetPlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                          std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "GetPlan service request received.");
            while (map.empty())
            { // wait until a map is published
                RCLCPP_WARN_STREAM(this->get_logger(), "global_costmap topic is not published yet. Waiting for a global costmap to be published into the topic.");
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
            for (int p = path_flat.size() - 2; p >= 0; p -= 2)
            {
                double i = path_flat[p];
                double j = path_flat[p + 1];

                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = "map";
                pose_stamped.header.stamp = this->now();
                pose_stamped.pose.position.x = (i * resolution) + origin_x + resolution / 2;
                pose_stamped.pose.position.y = (j * resolution) + origin_y + resolution / 2;
                // std::cout << pose_stamped.pose.position.x << "," << pose_stamped.pose.position.y << ";  ";

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
        std::vector<int> run(
            int start_i, int start_j, int goal_i, int goal_j)
        {
            std::vector<int> path = {goal_i, goal_j, start_i, start_j};
            return path;
        }
    };
}

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ee3305::Planner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}