#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#ifdef USE_LIBSERIAL
#include <gpiod.hpp>
#endif

#include <array>

class MotorControl : public rclcpp::Node {
public:
    MotorControl() : Node("motor_control") {
        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MotorControl::cmd_vel_callback, this, std::placeholders::_1));
        
        chip_ = gpiod::chip("gpiochip0");
        
        // GPIO lines: 29 (PH.00, line 10), 31 (PH.01, line 11), 32 (PG.03, line 8), 33 (PH.02, line 108)
        lines_ = {
            chip_.get_line(105),  // Left IN1
            chip_.get_line(106),  // Left IN2
            chip_.get_line(41),   // Right IN3
            chip_.get_line(43)  // Right IN4
        };
        
        for (auto& line : lines_) {
            line.request({"motor", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
        }
    }

    ~MotorControl() {
        for (auto& line : lines_) {
            line.set_value(0);
            line.release();
        }
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        int left_motor = 0;
        int right_motor = 0;

        // WASD mapping
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

        // Set motor directions
        lines_[0].set_value(left_motor == 1 ? 1 : 0);  // IN1
        lines_[1].set_value(left_motor == -1 ? 1 : 0); // IN2
        lines_[2].set_value(right_motor == 1 ? 1 : 0); // IN3
        lines_[3].set_value(right_motor == -1 ? 1 : 0); // IN4
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    gpiod::chip chip_;
    std::array<gpiod::line, 4> lines_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControl>());
    rclcpp::shutdown();
    return 0;
}
