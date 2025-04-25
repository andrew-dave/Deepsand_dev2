#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <libserial/SerialPort.h>
#include <string>

using namespace LibSerial;
using std::placeholders::_1;

class LoadcellPub : public rclcpp::Node 
{
    public:
        LoadcellPub(): Node("loadcell_publisher"){
            publisher_ = this->create_publisher<std_msgs::msg::Float32>("loadcell_data", 10);

            try {
                serial_.Open("/dev/ttyACM0");
                serial_.SetBaudRate(BaudRate::BAUD_115200);
                serial_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
                serial_.SetParity(Parity::PARITY_NONE);
                serial_.SetStopBits(StopBits::STOP_BITS_1);
                serial_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            } 
            catch (const OpenFailed&) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
                return;
            }
    
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&LoadcellPub::read_serial, this)
            );
        }

    private:
        void read_serial() {
            try {
                char c;
                while (serial_.IsDataAvailable()) {
                    serial_.ReadByte(c, 100);  // 100 ms timeout
                    if (c == '\n') {
                        try {
                            float value = std::stof(line_);
                            auto msg = std_msgs::msg::Float32();
                            msg.data = value;
                            publisher_->publish(msg);
                            // RCLCPP_INFO(this->get_logger(), "Published: %.2f", value);
                        } catch (...) {
                            RCLCPP_WARN(this->get_logger(), "Failed to parse: '%s'", line_.c_str());
                        }
                        line_.clear();
                    } else {
                        line_ += c;
                    }
                }
            } catch (const ReadTimeout&) {
                // Do nothing
            }
        }
        SerialPort serial_;
        std::string line_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoadcellPub>());
    rclcpp::shutdown();
    return 0;
}
