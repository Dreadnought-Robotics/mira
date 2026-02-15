// camera_driver_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <libudev.h>
#include <glob.h>
#include <regex>
#include <sys/stat.h>

class CameraStreamer : public rclcpp::Node
{
public:
    CameraStreamer() : Node("camera_driver_gstreamer_node")
    {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);

        // Declare parameters
        this->declare_parameter("device_path", "");
        this->declare_parameter("vendor_id", -1);
        this->declare_parameter("product_id", -1);
        this->declare_parameter("serial_no", "");
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("frame_format", "MJPEG");
        this->declare_parameter("framerate", 30);
        this->declare_parameter("camera_name", "camera");
        this->declare_parameter("frame_id", "camera_frame");

        // Get parameters
        int vendor = this->get_parameter("vendor_id").as_int();
        int product = this->get_parameter("product_id").as_int();
        std::string serial = this->get_parameter("serial_no").as_string();
        width_ = this->get_parameter("image_width").as_int();
        height_ = this->get_parameter("image_height").as_int();
        std::string fmt = this->get_parameter("frame_format").as_string();
        framerate_ = this->get_parameter("framerate").as_int();
        std::string dev = this->get_parameter("device_path").as_string();
        std::string camera_name = this->get_parameter("camera_name").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Find webcam device
        if (dev.empty()) {
            dev = find_webcam(vendor, product, serial);
            if (dev.empty()) {
                RCLCPP_WARN(this->get_logger(), "Webcam with specified vendor/product/serial not found");
                RCLCPP_WARN(this->get_logger(), "Attempting to use first available camera...");
                dev = find_first_camera();
                if (dev.empty()) {
                    throw std::runtime_error("No video devices found on system");
                }
            }
        }

        device_ = dev;
        RCLCPP_INFO(this->get_logger(), "Using webcam: %s", dev.c_str());

        // Query and adjust capabilities
        std::vector<std::string> supported_formats;
        std::vector<std::pair<int, int>> supported_resolutions;
        query_camera_capabilities(dev, supported_formats, supported_resolutions);

        // Adjust format if not supported
        if (!supported_formats.empty()) {
            bool format_found = false;
            for (const auto& sf : supported_formats) {
                if (strcasecmp(sf.c_str(), fmt.c_str()) == 0) {
                    format_found = true;
                    break;
                }
            }
            if (!format_found) {
                fmt = supported_formats[0];
                RCLCPP_WARN(this->get_logger(), "Requested format not supported");
                RCLCPP_WARN(this->get_logger(), "Using '%s' instead", fmt.c_str());
            }
        }

        // Adjust resolution if not supported
        if (!supported_resolutions.empty()) {
            bool res_found = false;
            for (const auto& res : supported_resolutions) {
                if (res.first == width_ && res.second == height_) {
                    res_found = true;
                    break;
                }
            }
            if (!res_found) {
                // Find closest resolution
                auto best_res = supported_resolutions[0];
                int target_area = width_ * height_;
                int min_diff = std::abs(best_res.first * best_res.second - target_area);
                
                for (const auto& res : supported_resolutions) {
                    int diff = std::abs(res.first * res.second - target_area);
                    if (diff < min_diff) {
                        min_diff = diff;
                        best_res = res;
                    }
                }
                
                RCLCPP_WARN(this->get_logger(), "Requested resolution %dx%d not supported", width_, height_);
                width_ = best_res.first;
                height_ = best_res.second;
                RCLCPP_WARN(this->get_logger(), "Using %dx%d instead", width_, height_);
            }
        }

        // Print configuration
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "Stream Configuration:");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "Device:             %s", dev.c_str());
        RCLCPP_INFO(this->get_logger(), "Image Width:        %d px", width_);
        RCLCPP_INFO(this->get_logger(), "Image Height:       %d px", height_);
        RCLCPP_INFO(this->get_logger(), "Frame Format:       %s", fmt.c_str());
        RCLCPP_INFO(this->get_logger(), "Framerate:          %d fps", framerate_);
        RCLCPP_INFO(this->get_logger(), "ROS2 Topic:         /%s/image_raw", camera_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera Info Topic:  /%s/camera_info", camera_name.c_str());
        RCLCPP_INFO(this->get_logger(), "============================================================");

        // Create publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            camera_name + "/image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            camera_name + "/camera_info", 10);

        // Build GStreamer pipeline - keep it simple
        std::string gst_format = get_gstreamer_format(fmt);
        std::stringstream ss;
        
        ss << "v4l2src device=" << dev << " ! "
           << gst_format << ",width=" << width_ 
           << ",height=" << height_ 
           << ",framerate=" << framerate_ << "/1 ! ";

        // Decode MJPEG if needed
        if (strcasecmp(fmt.c_str(), "MJPEG") == 0) {
            ss << "jpegdec ! ";
        }

        ss << "videoconvert ! video/x-raw,format=RGB ! "
           << "appsink name=appsink";

        std::string pipeline_str = ss.str();

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline: %s", pipeline_str.c_str());

        // Create pipeline
        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (error) {
            std::string err_msg = error->message;
            g_error_free(error);
            throw std::runtime_error("Failed to create pipeline: " + err_msg);
        }

        // Get appsink
        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
        if (!appsink_) {
            throw std::runtime_error("Failed to get appsink element");
        }

        // Configure appsink - sync=TRUE is important for smooth playback
        g_object_set(G_OBJECT(appsink_),
            "emit-signals", FALSE,
            "sync", TRUE,  // Let GStreamer handle timing
            NULL);

        // Start pipeline
        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            throw std::runtime_error("Unable to set pipeline to playing state");
        }

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline started successfully");

        // Start frame grabbing thread (blocking pull mode like gscam)
        stop_signal_ = false;
        grab_thread_ = std::thread([this]() {
            this->grab_frames();
        });

        frame_count_ = 0;
    }

    ~CameraStreamer()
    {
        cleanup();
    }

private:
    GstFlowReturn on_new_sample()
    {
        // Pull sample directly (blocking call, more efficient than signal)
        GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
        if (!sample) {
            return GST_FLOW_EOS;
        }

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);

        // Get frame dimensions from caps (cache to avoid repeated queries)
        static int cached_width = -1, cached_height = -1;
        if (cached_width == -1 || cached_height == -1) {
            GstStructure* structure = gst_caps_get_structure(caps, 0);
            gst_structure_get_int(structure, "width", &cached_width);
            gst_structure_get_int(structure, "height", &cached_height);
        }
        int width = cached_width;
        int height = cached_height;

        // Map buffer memory (read-only)
        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        // Get timestamp - use ROS time for now (GStreamer timestamps can drift)
        rclcpp::Time stamp = this->now();

        // Create ROS2 Image message - pre-allocate to avoid reallocation
        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
        img_msg->header.stamp = stamp;
        img_msg->header.frame_id = frame_id_;
        img_msg->height = height;
        img_msg->width = width;
        img_msg->encoding = "rgb8";
        img_msg->is_bigendian = false;
        img_msg->step = width * 3;
        
        // Efficient copy - direct memory copy, no intermediate buffer
        size_t expected_size = width * height * 3;
        if (map.size >= expected_size) {
            img_msg->data.assign(map.data, map.data + expected_size);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Buffer underflow: expected %zu bytes, got %zu", expected_size, map.size);
            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        // Unmap and release ASAP
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        // Publish image (move semantics - no copy)
        image_pub_->publish(std::move(img_msg));

        // Publish camera info (lightweight)
        auto camera_info_msg = create_camera_info();
        camera_info_msg.header.stamp = stamp;
        camera_info_msg.header.frame_id = frame_id_;
        camera_info_pub_->publish(camera_info_msg);

        // Minimal logging
        frame_count_++;

        return GST_FLOW_OK;
    }

    static void start_feed(GstElement* pipeline, guint size, gpointer data)
    {
        // Called when pipeline needs data
    }

    static void stop_feed(GstElement* pipeline, gpointer data)
    {
        // Called when pipeline has enough data
    }

    void grab_frames()
    {
        RCLCPP_INFO(this->get_logger(), "Frame grabbing thread started");
        
        while (!stop_signal_ && rclcpp::ok()) {
            // This blocks until a new frame is available
            GstFlowReturn ret = on_new_sample();
            
            if (ret != GST_FLOW_OK) {
                if (ret == GST_FLOW_EOS) {
                    RCLCPP_INFO(this->get_logger(), "End of stream");
                } else if (ret != GST_FLOW_FLUSHING) {
                    RCLCPP_WARN(this->get_logger(), "Flow error: %d", ret);
                }
                break;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Frame grabbing thread stopped");
    }

    sensor_msgs::msg::CameraInfo create_camera_info()
    {
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.width = width_;
        camera_info.height = height_;
        camera_info.distortion_model = "plumb_bob";

        // Simple pinhole camera model
        double fx = width_;
        double fy = width_;
        double cx = width_ / 2.0;
        double cy = height_ / 2.0;

        camera_info.k = {fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0};
        
        camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        
        camera_info.r = {1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0};
        
        camera_info.p = {fx, 0.0, cx, 0.0,
                        0.0, fy, cy, 0.0,
                        0.0, 0.0, 1.0, 0.0};

        return camera_info;
    }

    std::string find_webcam(int vendor, int product, const std::string& serial)
    {
        glob_t glob_result;
        glob("/dev/video*", GLOB_TILDE, nullptr, &glob_result);
        
        std::vector<std::string> devices;
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            devices.push_back(glob_result.gl_pathv[i]);
        }
        globfree(&glob_result);

        if (devices.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No video devices found!");
            return "";
        }

        std::sort(devices.begin(), devices.end());

        // If no specific criteria, return empty
        if (vendor == -1 && product == -1 && serial.empty()) {
            return "";
        }

        struct udev* udev = udev_new();
        if (!udev) {
            return "";
        }

        for (const auto& dev : devices) {
            struct stat st;
            if (stat(dev.c_str(), &st) != 0) continue;

            struct udev_device* device = udev_device_new_from_devnum(udev, 'c', st.st_rdev);
            if (!device) continue;

            bool match = true;

            if (vendor != -1) {
                const char* vid = udev_device_get_sysattr_value(device, "idVendor");
                if (!vid || strtol(vid, nullptr, 16) != vendor) {
                    match = false;
                }
            }

            if (match && product != -1) {
                const char* pid = udev_device_get_sysattr_value(device, "idProduct");
                if (!pid || strtol(pid, nullptr, 16) != product) {
                    match = false;
                }
            }

            if (match && !serial.empty()) {
                const char* ser = udev_device_get_sysattr_value(device, "serial");
                if (!ser || serial != ser) {
                    match = false;
                }
            }

            udev_device_unref(device);

            if (match) {
                udev_unref(udev);
                return dev;
            }
        }

        udev_unref(udev);
        return "";
    }

    std::string find_first_camera()
    {
        glob_t glob_result;
        glob("/dev/video*", GLOB_TILDE, nullptr, &glob_result);
        
        std::vector<std::string> devices;
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            devices.push_back(glob_result.gl_pathv[i]);
        }
        globfree(&glob_result);

        if (devices.empty()) {
            return "";
        }

        std::sort(devices.begin(), devices.end());

        // Prefer even-numbered devices (usually capture devices)
        for (const auto& dev : devices) {
            std::regex num_regex(R"(/dev/video(\d+))");
            std::smatch match;
            if (std::regex_match(dev, match, num_regex)) {
                int num = std::stoi(match[1].str());
                if (num % 2 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Found video capture device: %s", dev.c_str());
                    return dev;
                }
            }
        }

        return devices[0];
    }

    void query_camera_capabilities(const std::string& device,
                                   std::vector<std::string>& formats,
                                   std::vector<std::pair<int, int>>& resolutions)
    {
        GstDeviceMonitor* monitor = gst_device_monitor_new();
        gst_device_monitor_add_filter(monitor, "Video/Source", nullptr);

        if (!gst_device_monitor_start(monitor)) {
            RCLCPP_WARN(this->get_logger(), "Failed to start GStreamer device monitor");
            gst_object_unref(monitor);
            return;
        }

        GList* devices = gst_device_monitor_get_devices(monitor);
        GstDevice* target_device = nullptr;

        for (GList* l = devices; l != nullptr; l = l->next) {
            GstDevice* dev = GST_DEVICE(l->data);
            GstStructure* props = gst_device_get_properties(dev);
            
            if (props) {
                const gchar* path = gst_structure_get_string(props, "api.v4l2.path");
                if (path && device == path) {
                    target_device = GST_DEVICE(gst_object_ref(dev));
                    gst_structure_free(props);
                    break;
                }
                gst_structure_free(props);
            }
        }

        g_list_free_full(devices, gst_object_unref);
        gst_device_monitor_stop(monitor);
        gst_object_unref(monitor);

        if (!target_device) {
            RCLCPP_WARN(this->get_logger(), "Device %s not found in GStreamer device monitor", device.c_str());
            return;
        }

        GstCaps* caps = gst_device_get_caps(target_device);
        if (!caps) {
            gst_object_unref(target_device);
            return;
        }

        for (guint i = 0; i < gst_caps_get_size(caps); i++) {
            GstStructure* structure = gst_caps_get_structure(caps, i);
            const gchar* media_type = gst_structure_get_name(structure);

            // Detect format
            if (g_strcmp0(media_type, "image/jpeg") == 0) {
                if (std::find(formats.begin(), formats.end(), "MJPEG") == formats.end()) {
                    formats.push_back("MJPEG");
                }
            } else if (g_strcmp0(media_type, "video/x-raw") == 0) {
                const gchar* format = gst_structure_get_string(structure, "format");
                if (format) {
                    std::string fmt_str;
                    if (g_strcmp0(format, "YUY2") == 0) fmt_str = "YUYV";
                    else if (g_strcmp0(format, "NV12") == 0) fmt_str = "NV12";
                    else if (g_strcmp0(format, "RGB") == 0) fmt_str = "RGB";
                    else if (g_strcmp0(format, "BGR") == 0) fmt_str = "BGR";
                    
                    if (!fmt_str.empty() && std::find(formats.begin(), formats.end(), fmt_str) == formats.end()) {
                        formats.push_back(fmt_str);
                    }
                }
            }

            // Extract resolutions
            int width, height;
            if (gst_structure_get_int(structure, "width", &width) &&
                gst_structure_get_int(structure, "height", &height)) {
                auto res = std::make_pair(width, height);
                if (std::find(resolutions.begin(), resolutions.end(), res) == resolutions.end()) {
                    resolutions.push_back(res);
                }
            }
        }

        if (!formats.empty()) {
            std::string formats_str;
            for (size_t i = 0; i < formats.size(); ++i) {
                if (i > 0) formats_str += ", ";
                formats_str += formats[i];
            }
            RCLCPP_INFO(this->get_logger(), "Camera supports formats: %s", formats_str.c_str());
        }

        if (!resolutions.empty()) {
            std::sort(resolutions.begin(), resolutions.end(), 
                [](const auto& a, const auto& b) { return a.first * a.second > b.first * b.second; });
            
            std::string res_str;
            size_t count = std::min(size_t(5), resolutions.size());
            for (size_t i = 0; i < count; ++i) {
                if (i > 0) res_str += ", ";
                res_str += std::to_string(resolutions[i].first) + "x" + std::to_string(resolutions[i].second);
            }
            if (resolutions.size() > 5) {
                res_str += " ... (" + std::to_string(resolutions.size()) + " total)";
            }
            RCLCPP_INFO(this->get_logger(), "Camera supports resolutions: %s", res_str.c_str());
        }

        gst_caps_unref(caps);
        gst_object_unref(target_device);
    }

    std::string get_gstreamer_format(const std::string& fmt)
    {
        if (strcasecmp(fmt.c_str(), "MJPEG") == 0) return "image/jpeg";
        if (strcasecmp(fmt.c_str(), "YUYV") == 0) return "video/x-raw,format=YUY2";
        if (strcasecmp(fmt.c_str(), "RGB") == 0) return "video/x-raw,format=RGB";
        if (strcasecmp(fmt.c_str(), "BGR") == 0) return "video/x-raw,format=BGR";
        if (strcasecmp(fmt.c_str(), "NV12") == 0) return "video/x-raw,format=NV12";
        return "image/jpeg";
    }

    void cleanup()
    {
        stop_signal_ = true;
        
        if (grab_thread_.joinable()) {
            grab_thread_.join();
        }
        
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
        if (appsink_) {
            gst_object_unref(appsink_);
            appsink_ = nullptr;
        }
    }

    // Member variables
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    GstElement* pipeline_ = nullptr;
    GstElement* appsink_ = nullptr;
    std::thread grab_thread_;
    std::atomic<bool> stop_signal_{false};

    std::string device_;
    std::string frame_id_;
    int width_;
    int height_;
    int framerate_;
    size_t frame_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<CameraStreamer>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_driver"), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}