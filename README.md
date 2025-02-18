# Roboteq-USB-ROS2
ROS2 driver for Roboteq Motor Controllers. Direct serial port connection can be made over USB straight to the computer.

## Usage
The node can be used by itself or with multiple drivers in the same configuration, such as:
- Traditional four-wheel drive (two nodes receiving identical commands)
- Two-wheel drive (one node)
- Mecanum omniwheel drive (two nodes with separate commands)

## Installation
Use `colcon build` to install the package:
```sh
colcon build --packages-select roboteq_usb_ros
```
Remember to source the project after building:
```sh
source install/setup.bash
```
or
```sh
source install/setup.zsh
```

## Configuration
Some parameters are used to specify which topics the node should listen to.
### Parameters
- `mode`: (string) Communication mode, either 'serial' or 'can'.
- `device`: (string) Path to the device file for the motor controller.
- `left`: (string) Identifier for the left motor.
- `right`: (string) Identifier for the right motor.
- `cmd_vel_topic`: (string) Topic name for velocity commands.

### Custom message
The `WheelsMsg` is a custom message used to send commands to the motor controllers. It contains the following fields:
- `left_wheel`: (float) The speed command for the left wheel.
- `right_wheel`: (float) The speed command for the right wheel.

Example:
```yaml
left_wheel: 1.0
right_wheel: 1.0
```
### Custom service
Custom services are created by the node:
#### `Actuators.srv`
The `Actuators` service is used to control the actuators of the robot. It contains the following fields:
- `actuator_id`: (int32) The identifier for the actuator.
- `command`: (float) The command to be sent to the actuator.

Example:
```yaml
actuator_id: 1
command: 0.5
```

#### `SendCANCommands.srv`
The `SendCANCommands` service is used to send commands over the CAN bus. It contains the following fields:
- `can_id`: (uint32) The CAN identifier for the message.
- `data`: (uint8[8]) The data to be sent in the CAN message.

Example:
```yaml
can_id: 123
data: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
```

## Running
To run the node, you need to specify the communication mode and the device file. Depending on the mode, the command will differ slightly.

### Using Serial Communication
For serial communication, use the following command:
```sh
ros2 run roboteq_usb_ros roboteq_usb_ros_node --ros-args -p mode:=serial -p device:=/dev/ttyUSB0
```

### Using CAN Communication
For CAN communication, use the following command:
```sh
ros2 run roboteq_usb_ros roboteq_usb_ros_node --ros-args -p mode:=can -p device:=can0
```

Make sure to replace `/dev/ttyUSB0` and `can0` with the appropriate device file or CAN interface on your system.

### Additional Parameters
You can also specify additional parameters such as `left`, `right`, and `cmd_vel_topic` as needed:
```sh
ros2 run roboteq_usb_ros roboteq_usb_ros_node --ros-args -p mode:=serial -p device:=/dev/ttyUSB0 -p left:=left_motor -p right:=right_motor -p cmd_vel_topic:=/cmd_vel
```

## Launching with ROS2 Launch
You can also use a ROS2 launch file to start the node with predefined parameters. Create a launch file (e.g., `roboteq_usb_ros_launch.py`) in the `launch` directory of your package:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboteq_usb_ros',
            executable='roboteq_usb_ros_node',
            name='roboteq_usb_ros_node',
            output='screen',
            parameters=[{
                'mode': 'serial',
                'device': '/dev/ttyUSB0',
                'left': 'left_motor',
                'right': 'right_motor',
                'cmd_vel_topic': '/cmd_vel'
            }]
        )
    ])
```

To run the node using the launch file, use the following command:
```sh
ros2 launch roboteq_usb_ros roboteq_usb_ros_launch.py
```
Example launch files are already placed in the `/launch` directory.

## Contributing

We welcome contributions to the project! Here are some guidelines to help you get started:

1. **Fork the repository**: Create a personal fork of the project on GitHub.

2. **Clone the repository**: Clone your fork to your local machine.
    ```sh
    git clone https://github.com/Svndsn/roboteq-usb-ros2.git
    cd roboteq-usb-ros2
    ```

3. **Create a branch**: Create a new branch for your feature or bugfix.
    ```sh
    git checkout -b my-feature-branch
    ```

4. **Make changes**: Make your changes to the codebase.

5. **Commit changes**: Commit your changes with a descriptive commit message.
    ```sh
    git add .
    git commit -m "Description of my changes"
    ```

6. **Push changes**: Push your changes to your fork on GitHub.
    ```sh
    git push origin my-feature-branch
    ```

7. **Create a Pull Request**: Open a pull request on the original repository. Provide a clear description of your changes and any related issues.

8. **Review process**: Your pull request will be reviewed by the maintainers. Be prepared to make any necessary changes based on feedback.

9. **Merge**: Once approved, your changes will be merged into the main branch.

Thank you for contributing! This project is not actively maintained, but suggestions will be reviewed if added.

## License
The MIT License (MIT). See the LICENSE file for more information.

## Authors
Originally created by:

Robert J. Gebis (oxoocoffee) <rjgebis@yahoo.com> and Krystian R. Gebis <krgebis@gmail.com>

Updated and ported to ROS2 by:

Simon E. Svendsen (svndsn) <simon.egeris.svendsn@gmail.com>