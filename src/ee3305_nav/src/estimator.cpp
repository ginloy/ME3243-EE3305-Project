#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "ee3305_nav/ee3305_nav.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    /** Publishes the raw occupancy grid data, and the global costmap that has inflation cost values. */
    class Estimator : public rclcpp::Node
    {

    private:
        // ---------- Tfs ------------
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_map_to_odom;

    public:
        /** Constructor. Run only once when this node is first created in `main()`. */
        explicit Estimator()
            : Node("estimator")
        {
            initStaticTf();
        }

    private:
        
        /**
         * Initializes the static transform broadcaster from `map` frame to `odom` frame.
         * The transformation has zero position and rotation.
         * In real applications, having no transformation between the map and odom
         * frame implies that there is no drift (errors from ground truth) in the odometry calculations.
         */
        void initStaticTf()
        {
            // Populate message
            geometry_msgs::msg::TransformStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            msg.child_frame_id = "odom";
            // no translation
            msg.transform.translation.x = 0;
            msg.transform.translation.y = 0;
            msg.transform.translation.z = 0;
            // no rotation.
            msg.transform.rotation.x = 0;
            msg.transform.rotation.y = 0;
            msg.transform.rotation.z = 0;
            msg.transform.rotation.w = 1;

            // Instantiate the broadcaster and send the tf message.
            tf_broadcaster_map_to_odom = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            tf_broadcaster_map_to_odom->sendTransform(msg);
        }
    };
}

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ee3305::Estimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}