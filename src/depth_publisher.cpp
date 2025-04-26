// This is the publisher node to publish the depth data for data collection
// depth_roi_publisher.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs_advanced_mode.hpp> // Include this for advanced mode
#include <fstream>  // For reading JSON file
#include <thread>   // For sleep


class DepthROIPublisher : public rclcpp::Node {
public:
    DepthROIPublisher() : Node("depth_publisher") {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/depth/roi_image", 10);
        bridge_ = std::make_shared<cv_bridge::CvImage>();

        // === RealSense Device Setup ===
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "No RealSense device found!");
            return;
        }

        rs2::device dev = devices.front();
        if (dev.is<rs400::advanced_mode>()) {
            rs400::advanced_mode advanced_dev = dev.as<rs400::advanced_mode>();
            if (!advanced_dev.is_enabled()) {
                RCLCPP_INFO(this->get_logger(), "Enabling Advanced Mode...");
                advanced_dev.toggle_advanced_mode(true);
                std::this_thread::sleep_for(std::chrono::seconds(2));  // Shorter delay is usually enough
            }

            // JSON config skipped here
            RCLCPP_INFO(this->get_logger(), "Advanced Mode is enabled, but no JSON loaded.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Device does not support advanced mode.");
        }

        // === Depth Stream Setup ===
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
        pipe_.start(cfg);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&DepthROIPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        rs2::frameset frames = pipe_.wait_for_frames();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (!depth_frame) return;

        // Convert depth frame to OpenCV 16-bit grayscale
        cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                            CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // Step 1: Clip at a max depth (optional but improves contrast)
        int max_depth_mm = 400;
        cv::Mat clipped;
        cv::threshold(depth_image, clipped, max_depth_mm, max_depth_mm, cv::THRESH_TRUNC);

        // Step 2: Normalize to 8-bit for visualization
        cv::Mat normalized;
        cv::normalize(clipped, normalized, 0, 255, cv::NORM_MINMAX);
        normalized.convertTo(normalized, CV_8UC1);

        // Step 3: Apply histogram equalization
        cv::Mat equalized;
        cv::equalizeHist(normalized, equalized);

        // Publish as mono8 image
        bridge_->encoding = "mono8";
        bridge_->image = equalized;
        bridge_->header.stamp = this->now();
        bridge_->header.frame_id = "depth_map_frame";
        image_pub_->publish(*bridge_->toImageMsg());
    }

    rs2::pipeline pipe_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::shared_ptr<cv_bridge::CvImage> bridge_;

    int x1, y1, x2, y2;  // ROI bounds
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthROIPublisher>());
    rclcpp::shutdown();
    return 0;
}
