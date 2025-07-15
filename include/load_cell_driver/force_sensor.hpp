#ifndef LOAD_CELL_DRIVER__FORCE_SENSOR_HPP_
#define LOAD_CELL_DRIVER__FORCE_SENSOR_HPP_

#include <string>
#include <vector>
#include <utility>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cerrno>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace load_cell_driver
{

class ForceSensorNode : public rclcpp::Node
{
public:
    ForceSensorNode();

private:
    void loadParameters();
    bool openSerialPort(const std::string &port, int baudrate);
    std::string readPacket();
    std::vector<std::pair<int, float>> extractPacket(const std::string &packet);
    void publish_forces(const std::vector<std::pair<int, float>> &data);
    void update_loop();

    int serial_fd_;
    std::string kSerialPort = "/dev/ttyUSB0";
    std::string serial_port_;
    int kBaudrate = 57600;
    int baudrate_;
    int kTimer = 10;
    int max_attempts = 100;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_vector_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace load_cell_driver

#endif // LOAD_CELL_DRIVER__FORCE_SENSOR_HPP_
