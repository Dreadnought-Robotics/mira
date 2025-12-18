#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

#include <sensorcapture.hpp>

using namespace sl_oc::sensors;

class ZedOpenCaptureImuNode : public rclcpp::Node
{
public:
    ZedOpenCaptureImuNode()
    : Node("zed_open_capture_imu"),
      sensor_(sl_oc::VERBOSITY::INFO)
    {
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "/zed/imu/data_raw",
            rclcpp::SensorDataQoS());


        // ----> Enumerate devices
        std::vector<int> devices = sensor_.getDeviceList();
        if (devices.empty()) {
            RCLCPP_FATAL(get_logger(), "No ZED cameras with sensors found");
            rclcpp::shutdown();
            return;
        }

        // ----> Initialize sensors
        if (!sensor_.initializeSensors(devices[0])) {
            RCLCPP_FATAL(get_logger(), "Failed to initialize ZED sensors");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Connected to ZED camera SN: %d",
            sensor_.getSerialNumber());

        timer_ = create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&ZedOpenCaptureImuNode::publishImu, this)
        );
    }

private:
    /* ===================== Madgwick Filter ===================== */

    void madgwickUpdate(
        double gx, double gy, double gz,
        double ax, double ay, double az,
        double dt)
    {
        // Normalize accelerometer
        double norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm < 1e-6) return;
        ax /= norm; ay /= norm; az /= norm;

        double _2q0 = 2.0*q0_;
        double _2q1 = 2.0*q1_;
        double _2q2 = 2.0*q2_;
        double _2q3 = 2.0*q3_;

        double f1 = _2q1*q3_ - _2q0*q2_ - ax;
        double f2 = _2q0*q1_ + _2q2*q3_ - ay;
        double f3 = 1.0 - _2q1*q1_ - _2q2*q2_ - az;

        double s0 = -_2q2*f1 + _2q1*f2;
        double s1 =  _2q3*f1 + _2q0*f2 - 2.0*q1_*f3;
        double s2 = -_2q0*f1 + _2q3*f2 - 2.0*q2_*f3;
        double s3 =  _2q1*f1 + _2q2*f2;

        norm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (norm < 1e-6) return;
        s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;

        q0_ += (-0.5*(q1_*gx + q2_*gy + q3_*gz) - beta_*s0) * dt;
        q1_ += ( 0.5*(q0_*gx + q2_*gz - q3_*gy) - beta_*s1) * dt;
        q2_ += ( 0.5*(q0_*gy - q1_*gz + q3_*gx) - beta_*s2) * dt;
        q3_ += ( 0.5*(q0_*gz + q1_*gy - q2_*gx) - beta_*s3) * dt;

        norm = std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
        q0_ /= norm; q1_ /= norm; q2_ /= norm; q3_ /= norm;
    }

    /* ===================== Publish ===================== */

    void publishImu()
    {
        const data::Imu imu = sensor_.getLastIMUData(5000);
        if (imu.valid == data::Imu::NOT_PRESENT)
            return;

        auto now_time = now();
        if (last_time_.nanoseconds() == 0) {
            last_time_ = now_time;
            return;
        }

        double dt = (now_time - last_time_).seconds();
        last_time_ = now_time;

        constexpr double DEG2RAD = M_PI / 180.0;

        double gx = imu.gX * DEG2RAD;
        double gy = imu.gY * DEG2RAD;
        double gz = imu.gZ * DEG2RAD;

        madgwickUpdate(gx, gy, gz, imu.aX, imu.aY, imu.aZ, dt);

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = now_time;
        msg.header.frame_id = "zed_imu";

        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;

        msg.linear_acceleration.x = imu.aX;
        msg.linear_acceleration.y = imu.aY;
        msg.linear_acceleration.z = imu.aZ;

        // Orientation from Madgwick
        msg.orientation.w = q0_;
        msg.orientation.x = q1_;
        msg.orientation.y = q2_;
        msg.orientation.z = q3_;

        // ----> Covariances

        constexpr double GYRO_VAR = 0.01 * 0.01;
        msg.angular_velocity_covariance[0] = GYRO_VAR;
        msg.angular_velocity_covariance[4] = GYRO_VAR;
        msg.angular_velocity_covariance[8] = GYRO_VAR;

        constexpr double ACC_VAR = 0.2 * 0.2;
        msg.linear_acceleration_covariance[0] = ACC_VAR;
        msg.linear_acceleration_covariance[4] = ACC_VAR;
        msg.linear_acceleration_covariance[8] = ACC_VAR;

        constexpr double ORI_VAR = 0.05 * 0.05;
        msg.orientation_covariance[0] = ORI_VAR;
        msg.orientation_covariance[4] = ORI_VAR;
        msg.orientation_covariance[8] = ORI_VAR;

        imu_pub_->publish(msg);

        // ----> Yaw (for PID)
        double yaw = std::atan2(
            2.0 * (q0_ * q3_ + q1_ * q2_),
            1.0 - 2.0 * (q2_ * q2_ + q3_ * q3_)
        );

        // Use `yaw` directly in your PID node
        (void)yaw;
    }

private:
    SensorCapture sensor_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Madgwick state
    double q0_ = 1.0, q1_ = 0.0, q2_ = 0.0, q3_ = 0.0;
    double beta_ = 0.1;
    rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedOpenCaptureImuNode>());
    rclcpp::shutdown();
    return 0;
}
    