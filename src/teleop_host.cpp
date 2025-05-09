#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16.hpp>
#include <SDL2/SDL.h>
#include <chrono>

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("teleop_host") {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        servo_pub_ = create_publisher<std_msgs::msg::Int16>("/blade_angle", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TeleopNode::update, this));
        
        SDL_Init(SDL_INIT_VIDEO);
        window_ = SDL_CreateWindow("Teleop", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 300, 300, 0);
    }

    ~TeleopNode() {
        SDL_DestroyWindow(window_);
        SDL_Quit();
    }

private:
    void update() {
        geometry_msgs::msg::Twist cmd_vel_msg;
        std_msgs::msg::Int16 servo_msg;
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                rclcpp::shutdown();
            }
        }

        const Uint8* keys = SDL_GetKeyboardState(NULL);
        // Robot teleoperation (WASD)
        if (keys[SDL_SCANCODE_W]) {
            cmd_vel_msg.linear.x = 1.0;  // Forward
        } else if (keys[SDL_SCANCODE_S]) {
            cmd_vel_msg.linear.x = -1.0; // Backward
        }
        if (keys[SDL_SCANCODE_A]) {
            cmd_vel_msg.angular.z = 1.0; // Left
        } else if (keys[SDL_SCANCODE_D]) {
            cmd_vel_msg.angular.z = -1.0; // Right
        }

        // Servo control (Up/Down)
        if (keys[SDL_SCANCODE_UP]) {
            current_angle_ = std::min(current_angle_ + 1, 180);
            servo_msg.data = current_angle_;
            servo_pub_->publish(servo_msg);
        } else if (keys[SDL_SCANCODE_DOWN]) {
            current_angle_ = std::max(current_angle_ - 1, 50);
            servo_msg.data = current_angle_;
            servo_pub_->publish(servo_msg);
        }

        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr servo_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    SDL_Window* window_;
    int current_angle_ = 50; // Initial servo angle
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}