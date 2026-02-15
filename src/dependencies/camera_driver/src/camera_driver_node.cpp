// camera_rtsp_streamer.cpp
// Updated to support hexadecimal vendor_id and product_id from launch files

#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <atomic>
#include <libudev.h>
#include <glob.h>
#include <regex>
#include <sys/stat.h>

class RTSPCameraStreamer : public rclcpp::Node
{
public:
    RTSPCameraStreamer() : Node("rtsp_camera_streamer")
    {
        // Initialize GStreamer
        gst_init(nullptr, nullptr);
        
        // Create GLib main loop (required for RTSP server)
        loop_ = g_main_loop_new(nullptr, FALSE);

        // Declare parameters - vendor_id and product_id as strings to support hex
        this->declare_parameter("device_path", "");
        this->declare_parameter("vendor_id", "");  // String to support "0x1234" format
        this->declare_parameter("product_id", "");  // String to support "0x1234" format
        this->declare_parameter("serial_no", "");
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("frame_format", "MJPEG");
        this->declare_parameter("framerate", 30);
        this->declare_parameter("port", 8554);
        this->declare_parameter("bitrate", 2000);  // kbps for H.264 encoding

        // Get parameters
        std::string vendor_str = this->get_parameter("vendor_id").as_string();
        std::string product_str = this->get_parameter("product_id").as_string();
        std::string serial = this->get_parameter("serial_no").as_string();
        int width = this->get_parameter("image_width").as_int();
        int height = this->get_parameter("image_height").as_int();
        std::string fmt = this->get_parameter("frame_format").as_string();
        int framerate = this->get_parameter("framerate").as_int();
        std::string dev = this->get_parameter("device_path").as_string();
        int port = this->get_parameter("port").as_int();
        std::string mount_point = "/image_rtsp";
        int bitrate = this->get_parameter("bitrate").as_int();

        // Parse vendor and product IDs (support both hex "0x1234" and decimal "1234")
        int vendor = parse_id(vendor_str, "vendor_id");
        int product = parse_id(product_str, "product_id");

        // Log the parsed values
        if (vendor != -1) {
            RCLCPP_INFO(this->get_logger(), "Vendor ID: 0x%04x (%d)", vendor, vendor);
        }
        if (product != -1) {
            RCLCPP_INFO(this->get_logger(), "Product ID: 0x%04x (%d)", product, product);
        }
        if (!serial.empty()) {
            RCLCPP_INFO(this->get_logger(), "Serial: %s", serial.c_str());
        }

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
                if (res.first == width && res.second == height) {
                    res_found = true;
                    break;
                }
            }
            if (!res_found) {
                auto best_res = supported_resolutions[0];
                int target_area = width * height;
                int min_diff = std::abs(best_res.first * best_res.second - target_area);
                
                for (const auto& res : supported_resolutions) {
                    int diff = std::abs(res.first * res.second - target_area);
                    if (diff < min_diff) {
                        min_diff = diff;
                        best_res = res;
                    }
                }
                
                RCLCPP_WARN(this->get_logger(), "Requested resolution %dx%d not supported", width, height);
                width = best_res.first;
                height = best_res.second;
                RCLCPP_WARN(this->get_logger(), "Using %dx%d instead", width, height);
            }
        }

        // Print configuration
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "RTSP Stream Configuration:");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "Device:             %s", dev.c_str());
        RCLCPP_INFO(this->get_logger(), "Image Width:        %d px", width);
        RCLCPP_INFO(this->get_logger(), "Image Height:       %d px", height);
        RCLCPP_INFO(this->get_logger(), "Frame Format:       %s", fmt.c_str());
        RCLCPP_INFO(this->get_logger(), "Framerate:          %d fps", framerate);
        RCLCPP_INFO(this->get_logger(), "H.264 Bitrate:      %d kbps", bitrate);
        RCLCPP_INFO(this->get_logger(), "RTSP Port:          %d", port);
        RCLCPP_INFO(this->get_logger(), "RTSP Mount Point:   %s", mount_point.c_str());
        RCLCPP_INFO(this->get_logger(), "RTSP URL:           rtsp://<host>:%d%s", port, mount_point.c_str());
        RCLCPP_INFO(this->get_logger(), "============================================================");

        // Build GStreamer pipeline for RTSP streaming with ultra-low latency
        std::string gst_format = get_gstreamer_format(fmt);
        std::stringstream ss;
        
        ss << "( "
           << "v4l2src device=" << dev << " ! "
           << gst_format << ",width=" << width 
           << ",height=" << height 
           << ",framerate=" << framerate << "/1 ! ";

        // Decode MJPEG if needed
        if (strcasecmp(fmt.c_str(), "MJPEG") == 0) {
            ss << "jpegdec ! ";
        }

        // Convert to I420 and encode to H.264 for RTSP with low-latency settings
        ss << "videoconvert ! "
           << "video/x-raw,format=I420 ! "
           << "x264enc bitrate=" << bitrate 
           << " speed-preset=ultrafast tune=zerolatency "
           << " key-int-max=" << framerate  // Keyframe every 1 second
           << " bframes=0 "  // No B-frames
           << " sliced-threads=true "
           << " rc-lookahead=0 "  // No lookahead
           << " sync-lookahead=0 "
           << " vbv-buf-capacity=0 "  // No buffering
           << " ! "
           << "rtph264pay name=pay0 pt=96 config-interval=1 "
           << ")";

        pipeline_str_ = ss.str();
        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline: %s", pipeline_str_.c_str());
        RCLCPP_INFO(this->get_logger(), "============================================================");

        // Create RTSP server
        server_ = gst_rtsp_server_new();
        gst_rtsp_server_set_service(server_, std::to_string(port).c_str());

        // Get the mount points
        GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server_);

        // Create a media factory
        GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
        
        // Set the launch pipeline
        gst_rtsp_media_factory_set_launch(factory, pipeline_str_.c_str());
        
        // Allow multiple clients
        gst_rtsp_media_factory_set_shared(factory, TRUE);

        // Attach the factory to the mount point
        gst_rtsp_mount_points_add_factory(mounts, mount_point.c_str(), factory);

        // Clean up
        g_object_unref(mounts);

        // Attach the server to the default main context
        server_id_ = gst_rtsp_server_attach(server_, nullptr);

        if (server_id_ == 0) {
            throw std::runtime_error("Failed to attach RTSP server");
        }

        RCLCPP_INFO(this->get_logger(), "RTSP server started successfully");
        RCLCPP_INFO(this->get_logger(), "Stream available at: rtsp://<host>:%d%s", port, mount_point.c_str());
    }

    ~RTSPCameraStreamer()
    {
        cleanup();
    }

private:
    // Parse ID string - supports hex "0x1234" and decimal "1234"
    int parse_id(const std::string& id_str, const std::string& param_name)
    {
        if (id_str.empty()) {
            return -1;
        }

        try {
            // Check if hex (starts with 0x or 0X)
            if (id_str.length() >= 2 && (id_str.substr(0, 2) == "0x" || id_str.substr(0, 2) == "0X")) {
                return std::stoi(id_str, nullptr, 16);
            } else {
                return std::stoi(id_str, nullptr, 10);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Invalid %s format: %s (error: %s)", 
                       param_name.c_str(), id_str.c_str(), e.what());
            return -1;
        }
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

            // Get parent USB device
            struct udev_device* usb_device = udev_device_get_parent_with_subsystem_devtype(
                device, "usb", "usb_device");

            if (usb_device) {
                if (vendor != -1) {
                    const char* vid = udev_device_get_sysattr_value(usb_device, "idVendor");
                    if (vid) {
                        int device_vendor = std::stoi(vid, nullptr, 16);
                        RCLCPP_DEBUG(this->get_logger(), "Device %s vendor: 0x%04x", dev.c_str(), device_vendor);
                        if (device_vendor != vendor) {
                            match = false;
                        }
                    } else {
                        match = false;
                    }
                }

                if (match && product != -1) {
                    const char* pid = udev_device_get_sysattr_value(usb_device, "idProduct");
                    if (pid) {
                        int device_product = std::stoi(pid, nullptr, 16);
                        RCLCPP_DEBUG(this->get_logger(), "Device %s product: 0x%04x", dev.c_str(), device_product);
                        if (device_product != product) {
                            match = false;
                        }
                    } else {
                        match = false;
                    }
                }

                if (match && !serial.empty()) {
                    const char* ser = udev_device_get_sysattr_value(usb_device, "serial");
                    if (ser) {
                        RCLCPP_DEBUG(this->get_logger(), "Device %s serial: %s", dev.c_str(), ser);
                        if (serial != ser) {
                            match = false;
                        }
                    } else {
                        match = false;
                    }
                }
            } else {
                // Not a USB device
                match = false;
            }

            udev_device_unref(device);

            if (match) {
                RCLCPP_INFO(this->get_logger(), "Found matching camera: %s", dev.c_str());
                udev_unref(udev);
                return dev;
            }
        }

        udev_unref(udev);
        RCLCPP_WARN(this->get_logger(), "No matching camera found with specified vendor/product/serial");
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
        if (loop_) {
            g_main_loop_quit(loop_);
            g_main_loop_unref(loop_);
            loop_ = nullptr;
        }
        
        if (server_id_ != 0) {
            g_source_remove(server_id_);
            server_id_ = 0;
        }
        
        if (server_) {
            g_object_unref(server_);
            server_ = nullptr;
        }
    }

    // Member variables
    GstRTSPServer* server_ = nullptr;
    guint server_id_ = 0;
    std::string pipeline_str_;
    GMainLoop* loop_ = nullptr;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RTSPCameraStreamer>();
        
        // Create a timer to pump GLib events while ROS2 is spinning
        auto timer = node->create_wall_timer(
            std::chrono::milliseconds(10),
            []() {
                // Pump GLib main loop events
                GMainContext* context = g_main_context_default();
                while (g_main_context_pending(context)) {
                    g_main_context_iteration(context, FALSE);
                }
            }
        );
        
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rtsp_camera_streamer"), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}