#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <SDL2/SDL.h>
#include <chrono>

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("teleop_node") {
        pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
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
        geometry_msgs::msg::Twist msg;
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                rclcpp::shutdown();
            }
        }

        const Uint8* keys = SDL_GetKeyboardState(NULL);
        if (keys[SDL_SCANCODE_W]) {
            msg.linear.x = 1.0;  // Forward
        } else if (keys[SDL_SCANCODE_S]) {
            msg.linear.x = -1.0; // Backward
        }
        if (keys[SDL_SCANCODE_A]) {
            msg.angular.z = 1.0; // Left
        } else if (keys[SDL_SCANCODE_D]) {
            msg.angular.z = -1.0; // Right
        }

        pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    SDL_Window* window_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}