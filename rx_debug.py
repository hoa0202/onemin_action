#!/usr/bin/env python3
"""토픽 들어오면 터미널에만 찍는다. 발행 없음.

  python3 rx_debug.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPICS = (
    "/action",
    "/action_check",
    "/harv_robot/line_move",
    "/harv_robot/docking_move",
    "/carry_robot/docking_move"
)


class RxDebug(Node):
    def __init__(self):
        super().__init__("rx_debug")
        for topic in TOPICS:
            self.create_subscription(
                String, topic, lambda msg, t=topic: self._on_msg(t, msg), 10
            )
            print(f"구독: {topic}", flush=True)

    def _on_msg(self, topic, msg):
        print(f"{topic}: {msg.data}", flush=True)


def main():
    rclpy.init()
    node = RxDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
