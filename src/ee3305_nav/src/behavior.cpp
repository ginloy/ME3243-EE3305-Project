#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "ee3305_nav/ee3305_nav.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    /** Publishes the raw occupancy grid data, and the global costmap that has inflation cost values. */
    class Behavior : public rclcpp::Node
    {

    private:
        // ----------- Publishers / Subscribers --------------
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

        // ----------- Services ---------------
        rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr cli_get_plan;

        // ----------- Timers -------------------
        rclcpp::TimerBase::SharedPtr timer_main; // contains the timer that runs the main looping function at regular intervals.

        // ----------- Parameters ---------------
        double frequency;
        double plan_frequency;
        
        // ----------- States / Others -------------
        rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future_get_plan;
        std::vector<double> plan_flat;
        double goal_x;
        double goal_y;
        double rbt_x;
        double rbt_y;
        double prev_plan_time;
        bool need_plan; //redundant since the service is called every plan_frequency Hz.

    public:
        /** Constructor. Run only once when this node is first created in `main()`. */
        explicit Behavior()
            : Node("behavior")
        {
            initStates();
            initParams();
            initTopics();
            initServices();

            initTimers();
        }

    private:
        void initStates()
        {
            goal_x = NAN;
            goal_y = NAN;
            rbt_x = NAN;
            rbt_y = NAN;
            prev_plan_time = NAN;
        }

        /** Initializes and read parameters, if any. */
        void initParams()
        {
            initParam(this, "frequency", frequency);
            initParam(this, "plan_frequency", plan_frequency);
        }

        
        /** Initializes topics and messages, if any. */
        void initTopics()
        {
            auto qos = rclcpp::SensorDataQoS();
            qos.keep_last(1);

            sub_clicked_point = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "clicked_point", qos,
                std::bind(&Behavior::cbClickedPoint, this, std::placeholders::_1));

            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", qos,
                std::bind(&Behavior::cbOdom, this, std::placeholders::_1));
        }

        void cbClickedPoint(geometry_msgs::msg::PointStamped::SharedPtr msg)
        {
            goal_x = msg->point.x;
            goal_y = msg->point.y;
            need_plan = true;
            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Received goal at clicked point (" << goal_x << "," << goal_y << ")");
        }

        void cbOdom(nav_msgs::msg::Odometry::SharedPtr msg)
        { // transform assumes zero transform between `map` frame and `odom` frame.
            rbt_x = msg->pose.pose.position.x;
            rbt_y = msg->pose.pose.position.y;
        }

        void initServices()
        {
            cli_get_plan = this->create_client<nav_msgs::srv::GetPlan>(
                "get_plan", rmw_qos_profile_services_default);
        }

        /**
         * Sends a service request to the planner node to generate a path,
         * if a new goal is received from the clicked_point topic, or at every `plan_frequency` Hz.
         * Exits before a response is received, so it does not block; the request is handled by cbGetPlanReceived.
         */
        void getPlan()
        {
            if (std::isnan(rbt_x) || std::isnan(goal_x))
            {
                RCLCPP_DEBUG_STREAM(
                    this->get_logger(),
                    "robot or goal position not available: rbt(" 
                        << rbt_x << "," << rbt_y << "); goal("
                        << goal_x << "," << goal_y << ")");
                return; // rbt position or goal position not available.
            }

            double time_now = this->now().seconds();
            double time_elapsed = time_now - prev_plan_time;

            if (need_plan || time_elapsed > 1 / plan_frequency)
            {
                prev_plan_time = time_now;

                auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
                request->goal.pose.position.x = goal_x;
                request->goal.pose.position.y = goal_y;
                request->start.pose.position.x = rbt_x;
                request->start.pose.position.y = rbt_y;

                RCLCPP_INFO_STREAM(
                    this->get_logger(),
                    "Requesting plan for goal ( "
                        << goal_x << ", " << goal_y << " )"
                        << "from start[robot]( " << rbt_x << ", " << rbt_y << " )");

                // wait for service to load.
                while (!cli_get_plan->wait_for_service(0.5s))
                {
                    if (!rclcpp::ok())
                        return;
                    RCLCPP_INFO_STREAM(this->get_logger(), "waiting for get_plan service");
                }

                // send request. future_get_plan as a class member prevents the request from being deleted while it is processed asynchronously.
                future_get_plan = cli_get_plan->async_send_request(
                                                  request,
                                                  std::bind(&Behavior::cbGetPlanReceived, this, std::placeholders::_1))
                                      .future; // don't block; wait until the 
                need_plan = false;
            }
        }

        /**
         * Handles the response from a prior service request for a path.
         */
        void cbGetPlanReceived(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
        { // example code for implementing async callbacks.
        }

        /** Initializes the timers with their callbacks.*/
        void initTimers()
        {
            prev_plan_time = this->now().seconds();
            timer_main = this->create_wall_timer(
                1s / frequency,
                std::bind(&Behavior::cbTimerMain, this));
        }

        /** The function that is run at regular intervals */
        void cbTimerMain()
        {
            getPlan();
        }
    };
}

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee3305::Behavior>());
    rclcpp::shutdown();
    return 0;
}