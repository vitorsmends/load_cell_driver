
# Load Cell Driver

This ROS 2 package provides a driver node for reading force measurements from a load cell sensor connected via a serial port. The node reads serial data, parses it into force values, and publishes the values on a ROS topic.

## Features

- Serial communication with load cell sensor
- Parses custom serial packet format
- Publishes force data as `Float64MultiArray`
- Configurable parameters for serial port and baudrate
- Optional serial port initialization (useful for unit testing)

## Dependencies

- ROS 2 Humble
- `std_msgs`
- `rclcpp`

## Parameters

The following parameters can be configured through launch files or command line:

- `serial_port` (string, default: `/dev/ttyUSB0`): Path to the serial device
- `baudrate` (int, default: `115200`): Serial communication baudrate
- `verbose` (bool, default: `false`): Enable verbose logging

## Usage

### 1. Building the Package

```bash
cd ~/Workspaces/impedance_ws
colcon build --packages-select load_cell_driver
source install/setup.bash
```

### 2. Running the Node (Directly)

```bash
ros2 run load_cell_driver force_sensor_node
```

### 3. Launching with Parameters

You can use the provided launch file to start the node with custom parameters:

```bash
ros2 launch load_cell_driver force_sensor.launch.py serial_port:=/dev/ttyUSB1 baudrate:=9600 verbose:=true
```

This will launch the `force_sensor_node` with the specified `serial_port`, `baudrate`, and `verbose` settings.

## Testing

Unit tests are provided using GoogleTest. To run them:

```bash
colcon test --packages-select load_cell_driver
colcon test-result --verbose
```

## License

This project is licensed under the MIT License.