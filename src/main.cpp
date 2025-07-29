#include <rclcpp/rclcpp.hpp>
#include "load_cell_driver/force_sensor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<load_cell_driver::ForceSensorNode>(options);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Unhandled exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
