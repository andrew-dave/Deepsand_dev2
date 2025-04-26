#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <librealsense2/rs.hpp>

class DepthPublisher : public rclcpp::Node {
public:
    DepthPublisher() : Node("depth_publisher") {
        depth_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/depth/raw", 10);

        // RealSense Device Setup
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "No RealSense device found!");
            return;
        }

        // Depth Stream Setup
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
        pipe_.start(cfg);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(66),  // ~15 FPS
            std::bind(&DepthPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        rs2::frameset frames = pipe_.wait_for_frames();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        if (!depth_frame) return;

        // Create UInt16MultiArray message
        std_msgs::msg::UInt16MultiArray depth_msg;
        depth_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        depth_msg.layout.dim[0].label = "height";
        depth_msg.layout.dim[0].size = depth_frame.get_height();
        depth_msg.layout.dim[0].stride = depth_frame.get_width() * depth_frame.get_height();
        depth_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        depth_msg.layout.dim[1].label = "width";
        depth_msg.layout.dim[1].size = depth_frame.get_width();
        depth_msg.layout.dim[1].stride = depth_frame.get_width();

        // Copy raw depth data (16-bit)
        const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
        depth_msg.data.assign(depth_data, depth_data + (depth_frame.get_width() * depth_frame.get_height()));

        depth_pub_->publish(depth_msg);
    }

    rs2::pipeline pipe_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr depth_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthPublisher>());
    rclcpp::shutdown();
    return 0;
}