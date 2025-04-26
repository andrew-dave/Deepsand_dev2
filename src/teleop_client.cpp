#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <gpiod.hpp>
#include <libserialport.h>
#include <string>
#include <array>
#include <chrono>

class MotorControl : public rclcpp::Node {
public:
    MotorControl() : Node("teleop_client") {
        // Subscriptions
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MotorControl::cmd_vel_callback, this, std::placeholders::_1));
        servo_sub_ = create_subscription<std_msgs::msg::Int16>(
            "/blade_angle", 10, std::bind(&MotorControl::servo_callback, this, std::placeholders::_1));
        
        // Publisher
        load_cell_pub_ = create_publisher<std_msgs::msg::Float32>("/load_cell", 10);
        
        // GPIO setup: Pins 29 (line 105), 31 (line 106), 32 (line 41), 33 (line 43)
        chip_ = gpiod::chip("gpiochip0");
        lines_ = {
            chip_.get_line(105),   // Left IN1 (PH.00)
            chip_.get_line(106),   // Left IN2 (PH.01)
            chip_.get_line(41),    // Right IN3 (PG.03)
            chip_.get_line(43)   // Right IN4 (PH.02)
        };
        for (auto& line : lines_) {
            line.request({"motor", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
        }

        // Serial setup
        sp_get_port_by_name("/dev/ttyACM0", &port_);
        sp_open(port_, SP_MODE_READ_WRITE);
        sp_set_baudrate(port_, 115200);
        sp_set_bits(port_, 8);
        sp_set_parity(port_, SP_PARITY_NONE);
        sp_set_stopbits(port_, 1);

        // Timer for serial reading
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&MotorControl::read_serial, this));
    }

    ~MotorControl() {
        for (auto& line : lines_) {
            line.set_value(0);
            line.release();
        }
        sp_close(port_);
        sp_free_port(port_);
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        int left_motor = 0;
        int right_motor = 0;

        if (msg->linear.x > 0) {  // W: Forward
            left_motor = 1;
            right_motor = 1;
        } else if (msg->linear.x < 0) {  // S: Backward
            left_motor = -1;
            right_motor = -1;
        }
        if (msg->angular.z > 0) {  // A: Left turn
            left_motor = -1;
            right_motor = 1;
        } else if (msg->angular.z < 0) {  // D: Right turn
            left_motor = 1;
            right_motor = -1;
        }

        lines_[0].set_value(left_motor == 1 ? 1 : 0);  // IN1
        lines_[1].set_value(left_motor == -1 ? 1 : 0); // IN2
        lines_[2].set_value(right_motor == 1 ? 1 : 0); // IN3
        lines_[3].set_value(right_motor == -1 ? 1 : 0); // IN4
    }

    void servo_callback(const std_msgs::msg::Int16::SharedPtr msg) {
        int angle = std::max(50, std::min(150, static_cast<int>(msg->data)));
        std::string command = std::to_string(angle) + "\n";
        sp_nonblocking_write(port_, command.c_str(), command.size());
    }

    void read_serial() {
        char buf[256];
        int n = sp_nonblocking_read(port_, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            std::string data(buf);
            if (data.find("Load:") != std::string::npos) {
                try {
                    float weight = std::stof(data.substr(data.find(":") + 1));
                    std_msgs::msg::Float32 msg;
                    msg.data = weight;
                    load_cell_pub_->publish(msg);
                } catch (...) {
                    // Ignore parsing errors
                }
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr servo_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr load_cell_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    gpiod::chip chip_;
    std::array<gpiod::line, 4> lines_;
    struct sp_port* port_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControl>());
    rclcpp::shutdown();
    return 0;
}