#include <rclcpp/rclcpp.hpp>

#include <novatel_oem7_msgs/msg/bestgnsspos.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

using std::placeholders::_1;
using BESTGNSSPOS = novatel_oem7_msgs::msg::BESTGNSSPOS;
using NavSatFix = sensor_msgs::msg::NavSatFix;

class BESTGNSSPOSConverter : public rclcpp::Node
{   
    public:
        BESTGNSSPOSConverter()
        : Node("bestgnsspos_converter")
        {   
            // Subscriber
            auto bestgnsspos_callback = bind(&BESTGNSSPOSConverter::bestgnsspos_callback, this, _1);
            bestgnsspos_sub = this->create_subscription<BESTGNSSPOS>("/novatel/oem7/bestgnsspos", 10, bestgnsspos_callback);

            // Publisher
            navsatfix_pub = this->create_publisher<NavSatFix>("/novatel/oem7/navsatfix", 10);
        }
    
    private:
        // Subscriber
        rclcpp::Subscription<BESTGNSSPOS>::SharedPtr bestgnsspos_sub;

        // Publisher
        rclcpp::Publisher<NavSatFix>::SharedPtr navsatfix_pub;

        // Callback function
        void bestgnsspos_callback(const BESTGNSSPOS::SharedPtr bestgnsspos_msg)
        {
            NavSatFix navsatfix_msg;

            navsatfix_msg.header.stamp.sec = bestgnsspos_msg->nov_header.gps_week_milliseconds / 1000;
            navsatfix_msg.header.stamp.nanosec = (bestgnsspos_msg->nov_header.gps_week_milliseconds % 1000) * 1e6;
            navsatfix_msg.header.frame_id = "novatel_gps_link";

            navsatfix_msg.latitude = bestgnsspos_msg->lat;
            navsatfix_msg.longitude = bestgnsspos_msg->lon;
            navsatfix_msg.altitude = bestgnsspos_msg->hgt + bestgnsspos_msg->undulation;

            navsatfix_msg.position_covariance[0] = std::pow(bestgnsspos_msg->lat_stdev, 2);
            navsatfix_msg.position_covariance[4] = std::pow(bestgnsspos_msg->lon_stdev, 2);
            navsatfix_msg.position_covariance[8] = std::pow(bestgnsspos_msg->hgt_stdev, 2);

            navsatfix_pub->publish(navsatfix_msg);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BESTGNSSPOSConverter>());
    rclcpp::shutdown();

    return 0;
}