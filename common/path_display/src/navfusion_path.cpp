#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "coordinate_transform/earth.hpp"

using std::placeholders::_1;
using Path = nav_msgs::msg::Path;
using Odometry = nav_msgs::msg::Odometry;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class NavFusionPath : public rclcpp::Node
{
    public:
        NavFusionPath()
        : Node("navfusion_path")
        {
            // Subscriber
            auto odometry_callback = bind(&NavFusionPath::odometry_callback, this, _1);
            odometry_sub = this->create_subscription<Odometry>("/navfusion_odometry", 10, odometry_callback);

            // Publisher
            path_pub = this->create_publisher<Path>("/navfusion_path", 10);
        }

    private:
        // Subscriber
        rclcpp::Subscription<Odometry>::SharedPtr odometry_sub;

        // Publisher
        rclcpp::Publisher<Path>::SharedPtr path_pub;

        // Path
        std::vector<PoseStamped> navfusion_path;
        std::unique_ptr<tf2_ros::TransformBroadcaster> navfusion_tf_bro;

        // Callback function
        void odometry_callback(const Odometry::SharedPtr odometry_msg)
        {   
            // Save PoseStamped
            earth::geo2loc(odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z);
            
            PoseStamped posestamped;
            posestamped.header = odometry_msg->header;

            posestamped.pose.position.x = earth::getloc_x() - 169000.0;
            posestamped.pose.position.y = earth::getloc_y() - 2540000.0;
            posestamped.pose.position.z = earth::getloc_z();

            tf2::Quaternion q;
            q.setRPY(odometry_msg->pose.pose.orientation.x/180.0*M_PI, -odometry_msg->pose.pose.orientation.y/180.0*M_PI, -odometry_msg->pose.pose.orientation.z/180.0*M_PI + M_PI/2.0);
            posestamped.pose.orientation.x = q.x();
            posestamped.pose.orientation.y = q.y();
            posestamped.pose.orientation.z = q.z();
            posestamped.pose.orientation.w = q.w();

            navfusion_path.push_back(posestamped);

            // Publish Path
            Path path;
            path.header = odometry_msg->header;
            path.header.frame_id = "map";
            path.poses.clear();

            for (long unsigned int i = 0; i < navfusion_path.size(); i++)
            {   
                PoseStamped point;

                point.header = path.header;
                point.pose = navfusion_path[i].pose;

                path.poses.push_back(point);
            }
            path_pub->publish(path);  

            // Publish TF
            navfusion_tf_bro = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            TransformStamped transformstamped;

            transformstamped.header = odometry_msg->header;
            transformstamped.header.frame_id = "map";
            transformstamped.child_frame_id = "base_link";

            transformstamped.transform.translation.x = earth::getloc_x() - 169000.0;
            transformstamped.transform.translation.y = earth::getloc_y() - 2540000.0;
            transformstamped.transform.translation.z = earth::getloc_z();

            transformstamped.transform.rotation = posestamped.pose.orientation;

            navfusion_tf_bro->sendTransform(transformstamped);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavFusionPath>());
    rclcpp::shutdown();

    return 0;
}