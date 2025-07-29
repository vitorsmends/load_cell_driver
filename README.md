# Load Cell Driver

This ROS 2 package provides a driver for reading data from a load cell via serial communication. It parses packets received over a serial port and publishes the resulting force data as a ROS topic.

## Features

- Reads force data from a serial interface.
- Publishes parsed data to a `std_msgs/msg/Float64MultiArray` message.
- Configurable serial port, baudrate, and verbosity.

## Dependencies

- ROS 2 Humble
- `std_msgs`
- `rclcpp`

## Building

```bash
cd ~/YOUR_WORKSPACE
colcon build --packages-select load_cell_driver
source install/setup.bash
```

## Launching

You can launch the node using the provided launch file:

```bash
ros2 launch load_cell_driver force_sensor.launch.py
```

This launch file loads parameters from:

```
config/config.yaml
```

### Parameters (in `config/config.yaml`)

```yaml
force_sensor_node:
  ros__parameters:
    verbose: false                # Enables debug logs
    serial_port: "/dev/ttyUSB1"  # Serial port used to read load cell data
    baudrate: 57600              # Baudrate for serial communication
    enable_serial: true          # Set false to disable serial for testing
```

## Published Topics

- `/load_cell` (`std_msgs/msg/Float64MultiArray`): Publishes parsed force data.

## Testing

To run unit tests:

```bash
colcon test --packages-select load_cell_driver
colcon test-result --verbose
```

Make sure to disable the serial connection when testing, by setting `enable_serial: false` in `config.yaml`, or by passing it as an override parameter via `NodeOptions`.
