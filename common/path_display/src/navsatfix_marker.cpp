#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "coordinate_transform/earth.hpp"

using std::placeholders::_1;
using Point = geometry_msgs::msg::Point;
using Marker = visualization_msgs::msg::Marker;
using NavSatFix = sensor_msgs::msg::NavSatFix;

class NavSatFixMarker : public rclcpp::Node
{
    public:
        NavSatFixMarker()
        : Node("navsatfix_marker")
        {   
            init_marker();

            // Subscriber
            auto navsatfix_callback = bind(&NavSatFixMarker::navsatfix_callback, this, _1);
            navsatfix_sub = this->create_subscription<NavSatFix>("/novatel/oem7/navsatfix", 10, navsatfix_callback);

            // Publisher
            marker_pub = this->create_publisher<Marker>("/navsatfix_marker", 10);
        }

    private:
        // Subscriber
        rclcpp::Subscription<NavSatFix>::SharedPtr navsatfix_sub;

        // Publisher
        rclcpp::Publisher<Marker>::SharedPtr marker_pub;

        // Marker
        Marker marker;
        int id = 0;

        // Callback function
        void init_marker()
        {
            marker.header.frame_id = "map";
            marker.type = Marker::POINTS;
            marker.action = Marker::ADD;
    
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
    
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
        }

        void navsatfix_callback(const NavSatFix::SharedPtr navsatfix_msg)
        {   
            earth::geo2loc(navsatfix_msg->latitude, navsatfix_msg->longitude, navsatfix_msg->altitude);

            marker.header.stamp = navsatfix_msg->header.stamp;
            marker.id = id++;
            marker.points.clear();

            Point point;
            point.x = earth::getloc_x() - 169000.0;
            point.y = earth::getloc_y() - 2540000.0;
            point.z = earth::getloc_z();
            marker.points.push_back(point);

            // RCLCPP_INFO(this->get_logger(), "%f, %f", point.x, point.y);

            marker_pub->publish(marker);
            
            if (id == 60) id = 0;
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavSatFixMarker>());
    rclcpp::shutdown();

    return 0;
}