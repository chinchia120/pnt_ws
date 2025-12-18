#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

class PointCloud2Poser : public rclcpp::Node
{
    public:
        PointCloud2Poser()
        : Node("pointcloud2_poser")
        {
            // Subscriber
            auto pointcloud2_callback = bind(&PointCloud2Poser::pointcloud2_callback, this, _1);
            pointcloud2_sub = this->create_subscription<PointCloud2>("/velodyne_points", 10, pointcloud2_callback);

            // Publisher
            pointcloud2_pub = this->create_publisher<PointCloud2>("/velodyne_poser/pointcloud2", 10);
        }

    private:
        // Subscriber
        rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud2_sub;

        // Publisher
        rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud2_pub;

        // Callback function
        void pointcloud2_callback(const PointCloud2::SharedPtr pointcloud2_msg)
        {   
            PointCloud2 pointcloud2 = *pointcloud2_msg;

            pointcloud2.header.stamp = this->now();
            pointcloud2.header.frame_id = "vlp16";
            pointcloud2_pub->publish(pointcloud2);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloud2Poser>());
    rclcpp::shutdown();

    return 0;
}