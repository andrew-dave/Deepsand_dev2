#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstring>

volatile sig_atomic_t running = 1;

void signal_handler(int signum) {
    running = 0;
}

int main() {
    // 29, 31, 32, 33 - 105, 106, 41, 43
    const char *chip_name = "gpiochip0";
    unsigned int pins[] = {41, 43}; // Pin 31 (PQ.06, SOC_GPIO42), Pin 33 (PR.00, SOC_GPIO54)
    const char *pin_names[] = {"29 (IN1)", "31 (IN2)"};
    struct gpiod_chip *chip = nullptr;
    struct gpiod_line *lines[2] = {nullptr, nullptr};

    try {
        // Open GPIO chip
        chip = gpiod_chip_open_by_name(chip_name);
        if (!chip) {
            throw std::runtime_error("Failed to open GPIO chip: " + std::string(strerror(errno)));
        }

        // Request GPIO lines as outputs
        for (int i = 0; i < 2; ++i) {
            lines[i] = gpiod_chip_get_line(chip, pins[i]);
            if (!lines[i]) {
                throw std::runtime_error("Failed to get line " + std::string(pin_names[i]) + " (GPIO " + std::to_string(pins[i]) + "): " + std::string(strerror(errno)));
            }
            if (gpiod_line_request_output(lines[i], "motor-test", 0) < 0) {
                throw std::runtime_error("Failed to request line " + std::string(pin_names[i]) + ": " + std::string(strerror(errno)));
            }
            std::cout << "Initialized GPIO " << pin_names[i] << " (line " << pins[i] << ")" << std::endl;
        }

        // Set up signal handler
        signal(SIGINT, signal_handler);

        while (running) {
            // Forward: IN1 HIGH, IN2 LOW
            std::cout << "Forward: " << pin_names[0] << " HIGH, " << pin_names[1] << " LOW" << std::endl;
            gpiod_line_set_value(lines[0], 1); // Pin 31
            gpiod_line_set_value(lines[1], 0); // Pin 33
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Reverse: IN1 LOW, IN2 HIGH
            std::cout << "Reverse: " << pin_names[0] << " LOW, " << pin_names[1] << " HIGH" << std::endl;
            gpiod_line_set_value(lines[0], 0);
            gpiod_line_set_value(lines[1], 1);
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Stop: IN1 LOW, IN2 LOW
            std::cout << "Stop: " << pin_names[0] << " LOW, " << pin_names[1] << " LOW" << std::endl;
            gpiod_line_set_value(lines[0], 0);
            gpiod_line_set_value(lines[1], 0);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        // Cleanup
        for (int i = 0; i < 2; ++i) {
            if (lines[i]) {
                gpiod_line_release(lines[i]);
            }
        }
        if (chip) {
            gpiod_chip_close(chip);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        for (int i = 0; i < 2; ++i) {
            if (lines[i]) {
                gpiod_line_release(lines[i]);
            }
        }
        if (chip) {
            gpiod_chip_close(chip);
        }
        return 1;
    }

    std::cout << "Program terminated" << std::endl;
    return 0;
}
