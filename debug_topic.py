#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""운반 로봇(carry_01) 디버그: /carry_robot/docking_move 발행·구독."""

import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class CarryDebugNode(Node):
    def __init__(self):
        super().__init__("carry_debug_node")

        self.carry_robot_docking_move_pub = self.create_publisher(
            String, "/carry_robot/docking_move", 10
        )

        self.carry_docking_sub = self.create_subscription(
            String,
            "/carry_robot/docking_move",
            self.carry_docking_callback,
            10,
        )

        self.running = True

        self.get_logger().info("Carry debug node started.")
        self.print_help()

    def print_help(self):
        print("\n================== carry 디버그 명령 ==================")
        print("[운반 로봇 도킹] → /carry_robot/docking_move")
        print("  docking station  — move_to_docking_station")
        print("  docking return   — move_to_return (warehouse 복귀, scenario와 동일 키)")
        print("")
        print("  (수신 로그: move_finish, docking_end)")
        print("help / quit")
        print("=======================================================\n")

    def carry_docking_callback(self, msg: String):
        if msg.data in ["move_finish", "docking_end"]:
            self.get_logger().info(f"[RECV] /carry_robot/docking_move: {msg.data}")
        else:
            self.get_logger().warn(f"[WARN] Unknown carry docking msg: {msg.data}")

    def publish_string(self, pub, topic, value):
        msg = String()
        msg.data = value
        pub.publish(msg)
        self.get_logger().info(f"[PUB] {topic}: {value}")

    def input_loop(self):
        while self.running and rclpy.ok():
            try:
                user_input = input("명령 입력 > ").strip()
            except EOFError:
                break
            except Exception:
                break

            if not user_input:
                continue

            if user_input == "help":
                self.print_help()
                continue

            if user_input == "quit":
                self.running = False
                rclpy.shutdown()
                break

            if user_input == "docking station":
                self.publish_string(
                    self.carry_robot_docking_move_pub,
                    "/carry_robot/docking_move",
                    "move_to_docking_station",
                )
                continue

            if user_input == "docking return":
                self.publish_string(
                    self.carry_robot_docking_move_pub,
                    "/carry_robot/docking_move",
                    "move_to_return",
                )
                continue

            # debug_topic.py 예전 carry_docking CLI 호환
            if user_input.startswith("carry_docking "):
                cmd = user_input.split()[1]
                if cmd == "station":
                    self.publish_string(
                        self.carry_robot_docking_move_pub,
                        "/carry_robot/docking_move",
                        "move_to_docking_station",
                    )
                elif cmd == "return":
                    self.publish_string(
                        self.carry_robot_docking_move_pub,
                        "/carry_robot/docking_move",
                        "move_to_return",
                    )
                continue

            print("알 수 없는 명령 (help)")


def main(args=None):
    rclpy.init(args=args)

    node = CarryDebugNode()

    t = threading.Thread(target=node.input_loop, daemon=True)
    t.start()

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
