# Robotiq 2D-85 ROS Package

ROS Noetic package for controlling and interfacing with the Robotiq 2D-85 gripper.

## Prerequisites

- ROS Noetic
- Ubuntu 20.04
- Modern C++ compiler with C++14 support

## Installation

1. Clone this package into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/robotiq_2d_85.git
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Usage

1. Launch the Robotiq 2D-85 driver:
```bash
roslaunch robotiq_2d_85 robotiq_2d_85_driver.launch
```

2. Control the gripper using the provided services and topics:
```bash
# Open gripper
rosservice call /robotiq_2d_85/open_gripper

# Close gripper
rosservice call /robotiq_2d_85/close_gripper
```

## Docker Support

You can run this package using Docker:

```bash
# Build the Docker image
docker build -t robotiq_2d_85 .

# Run the container
docker run --rm --privileged -v /dev:/dev \
    --network host \
    robotiq_2d_85
```

Note: The `--privileged` flag is needed for USB access to the gripper.

## Action Server

The package uses ROS Action Server for reliable gripper control. The action server provides:

- Non-blocking gripper commands
- Real-time feedback during movement
- Goal preemption
- Status monitoring

Action Server Details:
- Action Name: `/robotiq_2d_85/gripper_action`
- Action Type: `control_msgs/GripperCommandAction`
- Feedback: Position, effort, and status
- Results: Success/failure and final position

Example action client usage:
```python
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

client = actionlib.SimpleActionClient('/robotiq_2d_85/gripper_action', GripperCommandAction)
client.wait_for_server()

goal = GripperCommandGoal()
goal.command.position = 0.085  # fully open
goal.command.max_effort = 10.0
client.send_goal(goal)
```

## Topics

- `/robotiq_2d_85/joint_states` (sensor_msgs/JointState): Current joint states of the gripper
- `/robotiq_2d_85/status` (robotiq_2d_85_msgs/Status): Gripper status information

## Services

- `/robotiq_2d_85/open_gripper`
- `/robotiq_2d_85/close_gripper`
- `/robotiq_2d_85/set_position`

## Parameters

- `~port` (string, default: "/dev/ttyUSB0"): Serial port for the gripper
- `~baud` (int, default: 115200): Baud rate for serial communication

## License

This package is released under the BSD 3-Clause License.

## Authors

Your Name <your.email@example.com>

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.


