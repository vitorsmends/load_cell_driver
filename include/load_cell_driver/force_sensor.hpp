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
    /**
     * @brief Constructor for the ForceSensorNode class.
     * 
     * Initializes parameters, opens the serial port, and sets up the publisher and timer.
     */
    ForceSensorNode();

private:
    /**
     * @brief Loads configurable parameters from the ROS 2 parameter server,
     * including serial port path, baudrate, and timer interval.
     */
    void loadParameters();

    /**
     * @brief Opens and configures the serial port.
     * 
     * @param port The path to the serial port device (e.g., "/dev/ttyUSB0").
     * @param baudrate The communication baudrate (e.g., 57600).
     * @return true if the serial port was successfully opened and configured; false otherwise.
     */
    bool openSerialPort(const std::string &port, int baudrate);

    /**
     * @brief Reads a full data packet from the serial port.
     * 
     * Accumulates characters until a full line (ending with '\n') is received.
     * @return A complete packet string, or an empty string if no full packet is available.
     */
    std::string readPacket();

    /**
     * @brief Extracts force data from a packet string.
     * 
     * Parses a string in the format "id:0,force:1.23;id:1,force:2.34;" and
     * returns a list of (id, force) pairs.
     * 
     * @param packet The input string containing force readings.
     * @return A vector of (id, force) pairs extracted from the packet.
     */
    std::vector<std::pair<int, float>> extractPacket(const std::string &packet);

    /**
     * @brief Publishes force data to a Float64MultiArray ROS 2 topic.
     * 
     * The vector is sized according to the highest sensor ID received, with
     * each value placed in the corresponding index.
     * 
     * @param data A list of (id, force) pairs.
     */
    void publish_forces(const std::vector<std::pair<int, float>> &data);

    /**
     * @brief Main update loop called periodically by the ROS 2 timer.
     * 
     * Handles reading from the serial port, parsing data, and publishing.
     */
    void update_loop();

    /** @brief File descriptor for the opened serial port. */
    int serial_fd_;

    /** @brief Default serial port path (used as fallback). */
    std::string kSerialPort = "/dev/ttyUSB0";

    /** @brief Actual serial port path loaded from parameters. */
    std::string serial_port_;

    /** @brief Default baudrate (used as fallback). */
    int kBaudrate = 57600;

    /** @brief Actual baudrate loaded from parameters. */
    int baudrate_;

    /** @brief Default timer interval in milliseconds (used as fallback). */
    int kTimer = 10;

    /** @brief Maximum number of attempts to read bytes from serial. */
    int max_attempts = 100;

    /** @brief Publisher for the Float64MultiArray topic containing force readings. */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_vector_pub_;

    /** @brief ROS 2 timer that periodically calls update_loop(). */
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace load_cell_driver

#endif // LOAD_CELL_DRIVER__FORCE_SENSOR_HPP_
