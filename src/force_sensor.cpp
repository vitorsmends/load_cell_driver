#include "load_cell_driver/force_sensor.hpp"

namespace load_cell_driver
{

ForceSensorNode::ForceSensorNode()
: Node("force_sensor_node")
{
    loadParameters();

    try {
        if (!openSerialPort(serial_port_, baudrate_)) {
            throw std::runtime_error("Serial open failed");
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Exception: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    force_vector_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/load_cell", 100);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(kTimer),
        std::bind(&ForceSensorNode::update_loop, this)
    );
}

void ForceSensorNode::loadParameters()
{
    this->declare_parameter<bool>("verbose", kVerbose);
    this->declare_parameter<std::string>("serial_port", kSerialPort);
    this->declare_parameter<int>("baudrate", kBaudrate);
    this->get_parameter<bool>("verbose", verbose_);
    this->get_parameter<std::string>("serial_port", serial_port_);
    this->get_parameter<int>("baudrate", baudrate_);
}

bool ForceSensorNode::openSerialPort(const std::string &port, int baudrate)
{
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s",
        port.c_str(), strerror(errno));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting terminal attributes: %s",
        strerror(errno));
        return false;
    }

    speed_t baud;
    switch (baudrate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported baudrate: %d",
            baudrate);
            return false;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting terminal attributes: %s",
        strerror(errno));
        return false;
    }
    else
    {
    RCLCPP_INFO(this->get_logger(), "Serial port \"%s\" successfully open",
    serial_port_.c_str());
    }

    return true;
}

std::string ForceSensorNode::readPacket()
{
    static std::string buffer;
    char byte;

    while (read(serial_fd_, &byte, 1) == 1) {
        if (byte == '\n') {
            std::string line = buffer;
            buffer.clear();
            return line;
        } else if (byte != '\r') {
            buffer += byte;
        }
    }
    return "";
}


std::vector<std::pair<int, float>> ForceSensorNode::extractPacket(const std::string &packet)
{
    std::vector<std::pair<int, float>> data;
    std::istringstream stream(packet);
    std::string segment;

    while (std::getline(stream, segment, ';')) {
        int id = -1;
        float force = 0.0f;
        std::istringstream pair_stream(segment);
        std::string token;

        while (std::getline(pair_stream, token, ','))
        {
            size_t sep = token.find(':');
            if (sep == std::string::npos) continue;

            std::string key = token.substr(0, sep);
            std::string value = token.substr(sep + 1);

            if (key == "id") {
                try {
                    id = std::stoi(value);
                } catch (...) {
                    id = -1;
                }
            } else if (key == "force") {
                try {
                    force = std::stof(value);
                } catch (...) {
                    force = 0.0f;
                }
            }
        }
        if (id >= 0) {
            data.emplace_back(id, force);
        }
    }

    return data;
}

void ForceSensorNode::publish_forces(const std::vector<std::pair<int, float>> &data)
{
    if (data.empty()) return;

    int max_id = 0;
    for (const auto &[id, _] : data) {
        if (id > max_id) max_id = id;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(max_id + 1, 0.0);

    for (const auto &[id, force] : data) {
        if (id >= 0 && static_cast<size_t>(id) < msg.data.size())
        {
            msg.data[id] = force;
        }
    }

    force_vector_pub_->publish(msg);

    if (verbose_)
    {
        RCLCPP_INFO(this->get_logger(), "Published force vector of size: %zu",
        msg.data.size());
    }
}


void ForceSensorNode::update_loop()
{
    std::string packet = readPacket();
    std::vector<std::pair<int, float>> data = extractPacket(packet);
    publish_forces(data);
    
    if (verbose_)
    {
        RCLCPP_INFO(this->get_logger(), "Received packet: %s", packet.c_str());
        RCLCPP_INFO(this->get_logger(), "Extracted %lu values", data.size());
    }
}

} // namespace load_cell_driver


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<load_cell_driver::ForceSensorNode>());
  rclcpp::shutdown();
  return 0;
}
