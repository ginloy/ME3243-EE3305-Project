#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ee3305_nav/ee3305_nav.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    /** Publishes the raw occupancy grid data, and the global costmap that has inflation cost values. */
    class MapServer : public rclcpp::Node
    {

    private:
        // ----------- Publishers / Subscribers --------------
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map;           
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_global_costmap;

        // // ----------- Timers -------------------
        rclcpp::TimerBase::SharedPtr timer_main; // contains the timer that runs the main looping function at regular intervals.

        // ----------- Parameters ---------------
        std::string map_yaml_path; // to yaml file containing map, must be in same directory as .pgm file
        double cost_exponent;
        double circumscribed_radius;
        double inflation_radius;
        char max_cost;

        // ----------- States / Others -------------
        struct InflationMaskElement
        {
            char cost;   // relative cost given the distance. (0 to 100)
            int ri;      // relative i coordinate (corresponds to x)
            int rj;      // relative j coordinate (corresponds to y)
            double dist; // physical distance
        };

    public:
        /** Constructor. Run only once when this node is first created in `main()`. */
        explicit MapServer(const std::string &map_yaml_path)
            : Node("map_server"), map_yaml_path(map_yaml_path)
        {
            initParams();
            initTopics();

            initTimers();
        }

    private:

        /** Initializes and read parameters, if any. */
        void initParams()
        {
            initParam(this, "cost_exponent", cost_exponent);
            initParam(this, "circumscribed_radius", circumscribed_radius);
            initParam(this, "inflation_radius", inflation_radius);
            initParam(this, "max_cost", max_cost);
        }

        /** Initializes topics and messages, if any. */
        void initTopics()
        {
            auto qos = rclcpp::SystemDefaultsQoS();
            qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            pub_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
            pub_global_costmap = this->create_publisher<nav_msgs::msg::OccupancyGrid>("global_costmap", qos);
        }

        /** Initializes the timers with their callbacks.*/
        void initTimers()
        {
            timer_main = this->create_wall_timer(
                1s, 
                std::bind(&MapServer::cbTimerMain, this));
        }

        /** The function that is run at regular intervals */
        void cbTimerMain()
        {
            std::filesystem::path path_yaml(map_yaml_path);
            if (std::filesystem::exists(path_yaml))
            {
                std::filesystem::path path_pgm = path_yaml;
                path_pgm.replace_extension("pgm");

                if (std::filesystem::exists(path_pgm))
                {
                    nav_msgs::msg::OccupancyGrid map = generateMap(path_yaml, path_pgm);
                    pub_map->publish(map);

                    nav_msgs::msg::OccupancyGrid global_costmap = generateMapServer(
                        map,
                        circumscribed_radius, inflation_radius,
                        max_cost, cost_exponent);
                    pub_global_costmap->publish(global_costmap);
                }
                else
                    throw std::range_error("PGM map file does not exist: '" + std::string(path_yaml.c_str()) + "'");
            }
            else
                throw std::range_error("YAML map file does not exist: '" + std::string(path_yaml.c_str()) + "'");
            timer_main = nullptr; // run this function only once.
            // rclcpp::shutdown();
        }

        /** 
         * Generates a `nav_msgs::msg::OccupancyGrid` map from PGM and YAML File.
         * Has no inflation zones.
         * To be published to `map` topic. 
         */
        nav_msgs::msg::OccupancyGrid generateMap(const std::filesystem::path path_yaml, const std::filesystem::path path_pgm)
        {
            std::string tmp;
            double resolution, x, y;

            std::ifstream yaml_file(path_yaml);
            while (std::getline(yaml_file, tmp))
            {
                std::istringstream line(tmp);
                std::string key;
                line >> key;
                if (key == "origin:")
                {
                    char c;
                    while (line >> c && c != '[')
                    {
                    }
                    line >> x >> tmp >> y;
                }
                else if (key == "resolution:")
                {
                    line >> resolution;
                }
            }

            std::ifstream pgm_file(path_pgm);

            int rows, cols;
            pgm_file >> tmp >> cols >> rows >> tmp;
            int length = rows * cols;

            unsigned char *buffer = new unsigned char[length];
            pgm_file.read((char *)buffer, length);

            // prepare message
            nav_msgs::msg::OccupancyGrid map;
            map.data.resize(length);
            map.header.frame_id = "map";
            map.info.resolution = resolution;
            map.info.width = cols;
            map.info.height = rows;
            map.info.origin.position.x = x;
            map.info.origin.position.y = y;
            map.info.origin.position.z = 0;
            map.info.origin.orientation.x = 0;
            map.info.origin.orientation.y = 0;
            map.info.origin.orientation.z = 0;
            map.info.origin.orientation.w = 1;

            for (int j = 0; j < rows; ++j)
                for (int i = 0; i < cols; ++i)
                    map.data[(rows - 1 - j) * cols + i] = buffer[j * cols + i] > 200 ? 0 : 100;

            return map;
        }

        /**
         * Generates a `nav_msgs::msg::OccupancyGrid` message describing the global_costmap, which has inflation zones and multiple cost values.
         * To be published into the `global_costmap` topic.
         */
        nav_msgs::msg::OccupancyGrid generateMapServer(const nav_msgs::msg::OccupancyGrid &map,
                                                           double circumscribed_radius, double inflation_radius,
                                                           char max_cost, double cost_exponent)
        {

            nav_msgs::msg::OccupancyGrid global_costmap;
            global_costmap.data.resize(map.data.size(), 0); // initialize all to zero ost
            global_costmap.header.frame_id = map.header.frame_id;
            global_costmap.info.resolution = map.info.resolution;
            global_costmap.info.width = map.info.width;
            global_costmap.info.height = map.info.height;
            global_costmap.info.origin.position.x = map.info.origin.position.x;
            global_costmap.info.origin.position.y = map.info.origin.position.y;
            global_costmap.info.origin.position.z = map.info.origin.position.z;
            global_costmap.info.origin.orientation.x = map.info.origin.orientation.x;
            global_costmap.info.origin.orientation.y = map.info.origin.orientation.y;
            global_costmap.info.origin.orientation.z = map.info.origin.orientation.z;
            global_costmap.info.origin.orientation.w = map.info.origin.orientation.w;

            // Fill costmap (exponential inflation costs)
            std::vector<InflationMaskElement> mask = generateInflationMask(
                map.info.resolution,
                circumscribed_radius, inflation_radius,
                max_cost, cost_exponent);
            const long cols = global_costmap.info.width;
            const long rows = global_costmap.info.height;

            for (long i = 0; i < rows; ++i)
            {
                for (long j = 0; j < cols; ++j)
                {
                    // from the closest neighbor cells, fill the cost if any obstacle cell lies within the mask.
                    for (const InflationMaskElement &rel : mask)
                    {
                        const long mi = i + rel.ri;
                        if (mi < 0 || mi > rows)
                            continue; // skip if out of map
                        const long mj = j + rel.rj;
                        if (mj < 0 || mj > cols)
                            continue; // skip if out of map

                        const long mkey = mi * cols + mj;
                        if (map.data[mkey] > 50)
                        {
                            const long key = i * cols + j;
                            global_costmap.data[key] = rel.cost;
                            break; // closest obstacle found
                        }
                    }
                }
            }

            return global_costmap;
        }

        /** 
         * Generates a list of relative coordinates and their exponential inflation cost 
         */
        std::vector<InflationMaskElement> generateInflationMask(
            double resolution, double circumscribed_radius, double inflation_radius,
            char max_cost, double cost_exponent)
        {
            if (inflation_radius < circumscribed_radius)
            {
                RCLCPP_WARN_STREAM(this->get_logger(),
                                   "Circumscribed radius ("
                                       << circumscribed_radius
                                       << ") is larger than inflation radius ("
                                       << inflation_radius
                                       << "). Setting the value of the inflation radius to circumscribed radius.");
                inflation_radius = circumscribed_radius;
            }
            if (resolution <= 0)
                throw std::range_error("resolution must be a positive number");
            if (max_cost <= 0)
                throw std::range_error("max_cost must be a positive number");

            // generate mask
            std::vector<InflationMaskElement> mask;
            int c = ceil(inflation_radius / resolution);
            for (int i = -c; i <= c; ++i)
            {
                for (int j = -c; j <= c; ++j)
                {
                    const double dist = sqrt(i * i + j * j) * resolution; // physical distance
                    if (dist > inflation_radius)
                        continue;

                    InflationMaskElement inf;
                    inf.dist = dist;
                    inf.ri = i;
                    inf.rj = j;
                    if (dist < circumscribed_radius)
                        inf.cost = max_cost;
                    else
                        inf.cost = static_cast<char>(round(
                            (2 + 1 / (0.5 * pow(dist - circumscribed_radius, cost_exponent) - 1)) * max_cost));
                    mask.push_back(inf);
                }
            }
            // sort the mask. front is cheapest, back is furthest
            std::sort(mask.begin(), mask.end(),
                      [](InflationMaskElement a, InflationMaskElement b)
                      { return a.dist < b.dist; });

            return mask;
        }
    };
}

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string map_yaml_path = argv[1];
    auto node = std::make_shared<ee3305::MapServer>(map_yaml_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}