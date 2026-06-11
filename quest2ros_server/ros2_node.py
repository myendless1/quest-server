from __future__ import annotations

import json

from .motion import QuestMotionServerState


def run(
    node_name: str = "quest_motion_server",
    rate_hz: float = 30.0,
    trigger_threshold: float = 0.5,
    trigger_field: str = "press_index",
    print_json: bool = False,
) -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from quest2ros.msg import OVR2ROSInputs
    from rclpy.node import Node

    class QuestMotionRos2Node(Node):
        def __init__(self) -> None:
            super().__init__(node_name)
            self.state = QuestMotionServerState(trigger_threshold, trigger_field)
            self._subscriptions = [
                self.create_subscription(PoseStamped, "/left_hand/pose", lambda msg: self.state.update_pose("left", msg), 10),
                self.create_subscription(PoseStamped, "/right_hand/pose", lambda msg: self.state.update_pose("right", msg), 10),
                self.create_subscription(Twist, "/left_hand/twist", lambda msg: self.state.update_twist("left", msg), 10),
                self.create_subscription(Twist, "/right_hand/twist", lambda msg: self.state.update_twist("right", msg), 10),
                self.create_subscription(OVR2ROSInputs, "/left_hand/inputs", lambda msg: self.state.update_input("left", msg), 10),
                self.create_subscription(OVR2ROSInputs, "/right_hand/inputs", lambda msg: self.state.update_input("right", msg), 10),
            ]
            if print_json:
                self.create_timer(1.0 / rate_hz, self._print_snapshot)

        def _print_snapshot(self) -> None:
            print(json.dumps(self.state.get_latest(), sort_keys=True), flush=True)

    rclpy.init()
    node = QuestMotionRos2Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
