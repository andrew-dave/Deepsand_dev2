#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthProcessor : public rclcpp::Node {
public:
    DepthProcessor() : Node("depth_to_img") {
        depth_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/depth/raw", 10, std::bind(&DepthProcessor::depth_callback, this, std::placeholders::_1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/depth/processed", 10);
        bridge_ = std::make_shared<cv_bridge::CvImage>();
    }

private:
    void depth_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        // Extract dimensions
        uint32_t height = msg->layout.dim[0].size;
        uint32_t width = msg->layout.dim[1].size;

        // Convert to OpenCV Mat
        cv::Mat depth_image(height, width, CV_16UC1, const_cast<uint16_t*>(msg->data.data()));

        // Process: Clip, normalize, equalize
        int max_depth_mm = 400;
        cv::Mat clipped;
        cv::threshold(depth_image, clipped, max_depth_mm, max_depth_mm, cv::THRESH_TRUNC);

        cv::Mat normalized;
        cv::normalize(clipped, normalized, 0, 255, cv::NORM_MINMAX);
        normalized.convertTo(normalized, CV_8UC1);

        cv::Mat equalized;
        cv::equalizeHist(normalized, equalized);

        // Publish as mono8
        bridge_->encoding = "mono8";
        bridge_->image = equalized;
        bridge_->header.stamp = this->now();
        bridge_->header.frame_id = "depth_map_frame";
        image_pub_->publish(*bridge_->toImageMsg());
    }

    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::shared_ptr<cv_bridge::CvImage> bridge_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthProcessor>());
    rclcpp::shutdown();
    return 0;
}