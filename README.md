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
python3 -m unittest
```
