from __future__ import annotations

import json
from typing import Optional

from .motion import QuestMotionServerState


class QuestMotionRos1Node:
    def __init__(
        self,
        rate_hz: float = 30.0,
        trigger_threshold: float = 0.5,
        trigger_field: str = "press_index",
        print_json: bool = False,
    ) -> None:
        import rospy
        from geometry_msgs.msg import PoseStamped, Twist
        from quest2ros.msg import OVR2ROSInputs

        self.rospy = rospy
        self.rate_hz = rate_hz
        self.print_json = print_json
        self.state = QuestMotionServerState(trigger_threshold, trigger_field)

        rospy.Subscriber("/left_hand/pose", PoseStamped, lambda msg: self.state.update_pose("left", msg))
        rospy.Subscriber("/right_hand/pose", PoseStamped, lambda msg: self.state.update_pose("right", msg))
        rospy.Subscriber("/left_hand/twist", Twist, lambda msg: self.state.update_twist("left", msg))
        rospy.Subscriber("/right_hand/twist", Twist, lambda msg: self.state.update_twist("right", msg))
        rospy.Subscriber("/left_hand/inputs", OVR2ROSInputs, lambda msg: self.state.update_input("left", msg))
        rospy.Subscriber("/right_hand/inputs", OVR2ROSInputs, lambda msg: self.state.update_input("right", msg))

    def spin(self) -> None:
        rate = self.rospy.Rate(self.rate_hz)
        while not self.rospy.is_shutdown():
            if self.print_json:
                print(json.dumps(self.state.get_latest(), sort_keys=True), flush=True)
            rate.sleep()


def run(
    node_name: str = "quest_motion_server",
    rate_hz: float = 30.0,
    trigger_threshold: float = 0.5,
    trigger_field: str = "press_index",
    print_json: bool = False,
) -> QuestMotionRos1Node:
    import rospy

    rospy.init_node(node_name)
    node = QuestMotionRos1Node(rate_hz, trigger_threshold, trigger_field, print_json)
    node.spin()
    return node
