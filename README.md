# quest2ros-server

Dual-arm Python motion server for Quest2ROS.

The server subscribes to Quest2ROS topics:

- `/left_hand/pose`
- `/left_hand/twist`
- `/left_hand/inputs`
- `/right_hand/pose`
- `/right_hand/twist`
- `/right_hand/inputs`

It keeps a trigger-relative state machine for each hand. When the trigger crosses the threshold, that hand latches the current controller pose as its start pose. While held, the server exposes:

- `delta_pos_base`: controller displacement in the Quest2ROS frame
- `delta_pos_local`: displacement in the trigger-start controller frame
- `delta_quat_local`: relative orientation from trigger start to current pose
- `delta_rotvec_local`: 3D axis-angle vector, usually convenient for policy inputs

## Install

From this directory:

```bash
python3 -m pip install -e .
```

## ROS Environment

This server does not parse Quest2ROS TCP traffic directly. It expects Quest2ROS data to already be available as ROS topics through `ros_tcp_endpoint`.

Required ROS messages:

- `geometry_msgs/PoseStamped`
- `geometry_msgs/Twist`
- `quest2ros/OVR2ROSInputs` on ROS1, or `quest2ros/msg/OVR2ROSInputs` on ROS2

Recommended setup:

- ROS1: Ubuntu 20.04 + ROS Noetic
- ROS2: Ubuntu 22.04 + ROS2 Humble

ROS1 Noetic is past upstream EOL, but it is still the simpler path for many Quest2ROS/Unity ROS-TCP-Endpoint setups.

### ROS1 Noetic

Install ROS1 packages:

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full python3-rosdep python3-catkin-tools python3-pip
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you use zsh:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.zshrc
source ~/.zshrc
```

Create a workspace and install ROS-TCP-Endpoint:

```bash
mkdir -p ~/quest2ros_ws/src
cd ~/quest2ros_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

Add the Quest2ROS ROS package that defines `quest2ros/msg/OVR2ROSInputs.msg` into `~/quest2ros_ws/src`. Then build:

```bash
cd ~/quest2ros_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
echo "source ~/quest2ros_ws/devel/setup.bash" >> ~/.bashrc
```

If you use zsh:

```bash
echo "source ~/quest2ros_ws/devel/setup.bash" >> ~/.zshrc
```

Verify the custom Quest2ROS message:

```bash
rosmsg show quest2ros/OVR2ROSInputs
```

Start the ROS master and TCP endpoint:

```bash
roscore
```

In another terminal:

```bash
source ~/quest2ros_ws/devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch
```

Quest2ROS should connect to the PC LAN IP and the endpoint port, usually `10000`.

### ROS2 Humble

Install ROS2 Humble on Ubuntu 22.04:

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl python3-pip python3-colcon-common-extensions
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you use zsh:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.zshrc
source ~/.zshrc
```

Create a ROS2 workspace, add a ROS2-compatible `ros_tcp_endpoint` package, and add the Quest2ROS ROS2 package that defines `quest2ros/msg/OVR2ROSInputs.msg`:

```bash
mkdir -p ~/quest2ros2_ws/src
cd ~/quest2ros2_ws/src
# Add ROS2-compatible ros_tcp_endpoint here.
# Add the Quest2ROS ROS2 message package here.
cd ~/quest2ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
echo "source ~/quest2ros2_ws/install/setup.bash" >> ~/.bashrc
```

If you use zsh:

```bash
echo "source ~/quest2ros2_ws/install/setup.bash" >> ~/.zshrc
```

Verify the custom Quest2ROS message:

```bash
ros2 interface show quest2ros/msg/OVR2ROSInputs
```

Start the ROS-TCP-Endpoint using the launch command provided by your ROS2-compatible endpoint package. Then verify Quest2ROS topics:

```bash
ros2 topic list
ros2 topic echo /right_hand/pose
ros2 topic echo /right_hand/inputs
```

### Network Checks

The Quest and the PC must be on the same LAN. Use the PC LAN IP in the Quest2ROS app, not `127.0.0.1`.

```bash
ip addr
ss -ltnp | grep 10000
```

If Ubuntu firewall is enabled:

```bash
sudo ufw allow 10000/tcp
```

## Run with ROS1

```bash
rosrun ros_tcp_endpoint default_server_endpoint.py
quest2ros-server --ros ros1 --print-json
```

## Run with ROS2

```bash
ros2 run ros_tcp_endpoint default_server_endpoint
quest2ros-server --ros ros2 --print-json
```

If your Quest2ROS input message uses a different trigger field, change it:

```bash
quest2ros-server --ros ros1 --trigger-field press_index --trigger-threshold 0.5
```

## Python API

```python
from quest2ros_server import QuestMotionServerState

state = QuestMotionServerState(trigger_threshold=0.5)

# ROS callbacks should only update cached state.
state.update_pose("left", pose_msg)
state.update_input("left", input_msg)

# Your policy loop can read snapshots at a fixed rate.
frame = state.get_latest()
left = frame["left"]
right = frame["right"]
```

`frame["active_hands"]` contains `["left"]`, `["right"]`, `["left", "right"]`, or `[]`. `frame["active_hand"]` is only set when exactly one hand is active.

## Tests

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
```
