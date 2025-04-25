#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class DepthProcessorNode : public rclcpp::Node {
public:
    DepthProcessorNode()
    : Node("depth_processor_node")
    {
        // Subscriber to raw depth image
        sub_ = image_transport::create_subscription(
            this,
            "/camera/depth/image_rect_raw",
            std::bind(&DepthProcessorNode::depthCallback, this, std::placeholders::_1),
            "raw"
        );

        // Publisher for processed depth image
        pub_ = image_transport::create_publisher(this, "/processed/depth/image");

        RCLCPP_INFO(this->get_logger(), "Depth processor node initialized.");
    }

private:
    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        try {
            // Convert to OpenCV image (16UC1 depth)
            auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth_raw = cv_ptr->image;

            // Normalize depth image for visualization
            cv::Mat depth_normalized;
            depth_raw.convertTo(depth_normalized, CV_8UC1, 255.0 / 10000.0);  // scale 0–10m to 0–255

            // Convert back to ROS image (for visualization as 8UC1)
            std_msgs::msg::Header header = msg->header;
            auto output_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, depth_normalized).toImageMsg();

            // Publish processed image
            pub_.publish(*output_msg);

        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthProcessorNode>());
    rclcpp::shutdown();
    return 0;
}

