#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;
using Image = sensor_msgs::msg::Image;

class ImagePoser : public rclcpp::Node
{
    public:
        ImagePoser()
        : Node("image_poser")
        {
            // Subscriber
            auto image_callback = bind(&ImagePoser::image_callback, this, _1);
            image_sub = this->create_subscription<Image>("/my_camera/pylon_ros2_camera_node/image_raw", 10, image_callback);

            // Publisher
            image_pub = this->create_publisher<Image>("/basler_poser/image", 10);
        }

    private:
        // Subscriber
        rclcpp::Subscription<Image>::SharedPtr image_sub;

        // Publisher
        rclcpp::Publisher<Image>::SharedPtr image_pub;

        // Callback function
        void image_callback(const Image::SharedPtr image_msg)
        {   
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");

            cv_bridge::CvImage bridge;
            bridge.header.stamp = image_msg->header.stamp;
            bridge.header.frame_id = "basler_link";
            bridge.encoding = "bgr8";
            bridge.image = cv_ptr->image;
            image_pub->publish(*bridge.toImageMsg());
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePoser>());
    rclcpp::shutdown();

    return 0;
}