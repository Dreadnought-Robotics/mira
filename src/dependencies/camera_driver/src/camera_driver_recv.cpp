#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <thread>
#include <atomic>
#include <sstream>

class RTSPReceiver : public rclcpp::Node
{
public:
    RTSPReceiver()
        : Node("rtsp_receiver")
    {
        gst_init(nullptr, nullptr);

        // Parameters
        declare_parameter("rtsp_url", "rtsp://localhost:8554/camera");
        declare_parameter("camera_name", "camera");
        declare_parameter("frame_id", "camera_frame");
        declare_parameter("transport", "tcp");
        declare_parameter("latency", 0);

        rtsp_url_   = get_parameter("rtsp_url").as_string();
        camera_name_ = get_parameter("camera_name").as_string();
        frame_id_   = get_parameter("frame_id").as_string();
        std::string transport = get_parameter("transport").as_string();
        int latency = get_parameter("latency").as_int();

        RCLCPP_INFO(get_logger(), "RTSP URL: %s", rtsp_url_.c_str());
        RCLCPP_INFO(get_logger(), "Publishing: /%s/image_raw", camera_name_.c_str());

        // Camera info publisher
        camera_info_pub_ =
            create_publisher<sensor_msgs::msg::CameraInfo>(
                camera_name_ + "/camera_info", 10);

        // GStreamer pipeline
        std::stringstream ss;
        ss << "rtspsrc location=" << rtsp_url_
           << " protocols=" << transport
           << " latency=" << latency
           << " buffer-mode=0 ! "
           << "rtph264depay ! "
           << "h264parse ! "
           << "avdec_h264 max-threads=2 ! "
           << "videoconvert ! "
           << "video/x-raw,format=RGB ! "
           << "appsink name=appsink_raw sync=false";

        pipeline_str_ = ss.str();
        RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline_str_.c_str());

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str_.c_str(), &error);
        if (error) {
            std::string msg = error->message;
            g_error_free(error);
            throw std::runtime_error(msg);
        }

        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink_raw");
        if (!appsink_)
            throw std::runtime_error("Failed to get appsink");

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        stop_ = false;
        grab_thread_ = std::thread(&RTSPReceiver::grab_loop, this);
    }

    ~RTSPReceiver()
    {
        stop_ = true;
        if (grab_thread_.joinable())
            grab_thread_.join();

        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }

        if (appsink_)
            gst_object_unref(appsink_);
    }

    void initialize()
    {
        image_transport::ImageTransport it(shared_from_this());
        image_pub_ = it.advertise(camera_name_ + "/image_raw", 10);
    }

private:
    void grab_loop()
    {
        while (rclcpp::ok() && !stop_) {
            GstSample* sample =
                gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
            if (!sample)
                break;

            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstCaps* caps = gst_sample_get_caps(sample);

            if (width_ == 0) {
                GstStructure* s = gst_caps_get_structure(caps, 0);
                gst_structure_get_int(s, "width", &width_);
                gst_structure_get_int(s, "height", &height_);
                RCLCPP_INFO(get_logger(),
                            "Stream resolution: %dx%d", width_, height_);
            }

            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);

            auto msg = sensor_msgs::msg::Image();
            msg.header.stamp = now();
            msg.header.frame_id = frame_id_;
            msg.height = height_;
            msg.width = width_;
            msg.encoding = "rgb8";
            msg.step = width_ * 3;
            msg.data.assign(map.data, map.data + map.size);

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);

            image_pub_.publish(msg);

            auto cam_info = create_camera_info();
            cam_info.header = msg.header;
            camera_info_pub_->publish(cam_info);
        }
    }

    sensor_msgs::msg::CameraInfo create_camera_info()
    {
        sensor_msgs::msg::CameraInfo ci;
        ci.width = width_;
        ci.height = height_;
        ci.distortion_model = "plumb_bob";

        double fx = width_;
        double fy = width_;
        double cx = width_ / 2.0;
        double cy = height_ / 2.0;

        ci.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        ci.d = {0, 0, 0, 0, 0};
        ci.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        ci.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};

        return ci;
    }

    // ROS
    image_transport::Publisher image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    // GStreamer
    GstElement* pipeline_{nullptr};
    GstElement* appsink_{nullptr};

    std::thread grab_thread_;
    std::atomic<bool> stop_{false};

    // State
    std::string rtsp_url_;
    std::string camera_name_;
    std::string frame_id_;
    std::string pipeline_str_;
    int width_{0};
    int height_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RTSPReceiver>();
    node->initialize();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
