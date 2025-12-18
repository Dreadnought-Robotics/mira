
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ImuToTfNode : public rclcpp::Node
{
public:
    ImuToTfNode()
    : Node("imu_to_tf_broadcaster")
    {
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(this);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/zed/imu/data_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&ImuToTfNode::imuCallback, this, std::placeholders::_1)
        );
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = msg->header.frame_id.empty()
                                    ? "zed_imu"
                                    : msg->header.frame_id;

        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation = msg->orientation;

        tf_broadcaster_->sendTransform(tf_msg);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToTfNode>());
    rclcpp::shutdown();
    return 0;
}
