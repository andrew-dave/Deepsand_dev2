#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <map>
#include <sys/ioctl.h>


using namespace std::chrono_literals;

class MotorControl : public rclcpp::Node {
public:
    MotorControl() : Node("motor_control") {
        // GPIO pin numbers (BCM or BOARD depending on /sys/class/gpio layout)
        pins_["IN1"] = 17;
        pins_["IN2"] = 27;
        pins_["IN3"] = 22;
        pins_["IN4"] = 23;

        setup_gpio();

        timer_ = this->create_wall_timer(100ms, std::bind(&MotorControl::check_key_input, this));
        RCLCPP_INFO(this->get_logger(), "Use W/A/S/D/Q to control motors.");
    }

private:
    std::map<std::string, int> pins_;
    rclcpp::TimerBase::SharedPtr timer_;

    void setup_gpio() {
        for (auto const& [label, pin] : pins_) {
            std::ofstream export_file("/sys/class/gpio/export");
            export_file << pin;
            export_file.close();

            std::ofstream direction_file("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction");
            direction_file << "out";
            direction_file.close();
        }
    }

    void write_pin(int pin, bool value) {
        std::ofstream value_file("/sys/class/gpio/gpio" + std::to_string(pin) + "/value");
        value_file << (value ? "1" : "0");
        value_file.close();
    }

    char get_key() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void check_key_input() {
        if (!isatty(STDIN_FILENO)) return;
        int bytes;
        ioctl(STDIN_FILENO, FIONREAD, &bytes);
        if (bytes == 0) return;

        char key = get_key();
        switch (key) {
            case 'w': forward(); break;
            case 's': backward(); break;
            case 'a': left(); break;
            case 'd': right(); break;
            case 'q': stop(); rclcpp::shutdown(); break;
            default: stop(); break;
        }
    }

    void forward() {
        write_pin(pins_["IN1"], true);
        write_pin(pins_["IN2"], false);
        write_pin(pins_["IN3"], true);
        write_pin(pins_["IN4"], false);
        RCLCPP_INFO(this->get_logger(), "Forward");
    }

    void backward() {
        write_pin(pins_["IN1"], false);
        write_pin(pins_["IN2"], true);
        write_pin(pins_["IN3"], false);
        write_pin(pins_["IN4"], true);
        RCLCPP_INFO(this->get_logger(), "Backward");
    }

    void left() {
        write_pin(pins_["IN1"], false);
        write_pin(pins_["IN2"], true);
        write_pin(pins_["IN3"], true);
        write_pin(pins_["IN4"], false);
        RCLCPP_INFO(this->get_logger(), "Left");
    }

    void right() {
        write_pin(pins_["IN1"], true);
        write_pin(pins_["IN2"], false);
        write_pin(pins_["IN3"], false);
        write_pin(pins_["IN4"], true);
        RCLCPP_INFO(this->get_logger(), "Right");
    }

    void stop() {
        write_pin(pins_["IN1"], false);
        write_pin(pins_["IN2"], false);
        write_pin(pins_["IN3"], false);
        write_pin(pins_["IN4"], false);
        RCLCPP_INFO(this->get_logger(), "Stop");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControl>());
    rclcpp::shutdown();
    return 0;
}
