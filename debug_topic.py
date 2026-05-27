#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32


class HarvestDebugNode(Node):
    def __init__(self):
        super().__init__('harvest_debug_node')

        # =========================
        # Publishers
        # =========================
        self.harv_stat_pub = self.create_publisher(String, '/harv_stat', 10)
        self.target_count_pub = self.create_publisher(Int32, '/fruit_harvest/target_count', 10)
        self.harvested_count_pub = self.create_publisher(Int32, '/fruit_harvest/harvested_count', 10)

        self.harv_robot_line_move_pub = self.create_publisher(String, '/harv_robot/line_move', 10)
        self.harv_robot_docking_move_pub = self.create_publisher(String, '/harv_robot/docking_move', 10)
        self.carry_robot_docking_move_pub = self.create_publisher(String, '/carry_robot/docking_move', 10)
        self.action_pub = self.create_publisher(String, '/action', 10)

        # =========================
        # Subscribers
        # =========================
        self.move_sub = self.create_subscription(
            String,
            '/harv_robot/move',
            self.move_callback,
            10
        )

        self.harv_docking_sub = self.create_subscription(
            String,
            '/harv_robot/docking_move',
            self.harv_docking_callback,
            10
        )

        self.carry_docking_sub = self.create_subscription(
            String,
            '/carry_robot/docking_move',
            self.carry_docking_callback,
            10
        )

        self.running = True

        self.get_logger().info('Harvest debug node started.')
        self.print_help()

    def print_help(self):
        print("\n================== 입력 가능한 명령 ==================")
        print("[수확 상태]")
        print("start / end / go")
        print("")
        print("[과실 개수]")
        print("target <숫자>")
        print("harvested <숫자>")
        print("")
        print("[라인 이동]")
        print("line <1~10> / line warehouse")
        print("")
        print("[수확 로봇 도킹]")
        print("harv_docking station / return")
        print("")
        print("[운반 로봇 도킹]")
        print("carry_docking station / return")
        print("")
        print("[수동 액션 (/action)]")
        print("end2  → entering_end")
        print("")
        print("help / quit")
        print("====================================================\n")

    # =========================
    # Callbacks (Subscriber)
    # =========================
    def move_callback(self, msg: String):
        self.get_logger().info(f"[RECV] /harv_robot/move: {msg.data}")

    def harv_docking_callback(self, msg: String):
        if msg.data in ["move_finish", "docking_end"]:
            self.get_logger().info(f"[RECV] /harv_robot/docking_move: {msg.data}")
        else:
            self.get_logger().warn(f"[WARN] Unknown harv docking msg: {msg.data}")

    def carry_docking_callback(self, msg: String):
        if msg.data in ["move_finish", "docking_end"]:
            self.get_logger().info(f"[RECV] /carry_robot/docking_move: {msg.data}")
        else:
            self.get_logger().warn(f"[WARN] Unknown carry docking msg: {msg.data}")

    # =========================
    # Publish Helpers
    # =========================
    def publish_string(self, pub, topic, value):
        msg = String()
        msg.data = value
        pub.publish(msg)
        self.get_logger().info(f"[PUB] {topic}: {value}")

    def publish_int(self, pub, topic, value):
        msg = Int32()
        msg.data = value
        pub.publish(msg)
        self.get_logger().info(f"[PUB] {topic}: {value}")

    # =========================
    # Input Loop
    # =========================
    def input_loop(self):
        while self.running and rclpy.ok():
            try:
                user_input = input("명령 입력 > ").strip()
            except:
                break

            if not user_input:
                continue

            if user_input == 'help':
                self.print_help()
                continue

            if user_input == 'quit':
                self.running = False
                rclpy.shutdown()
                break

            # /harv_stat
            if user_input in ['start', 'end', 'go']:
                self.publish_string(self.harv_stat_pub, '/harv_stat', user_input)
                continue

            # target
            if user_input.startswith('target '):
                try:
                    count = int(user_input.split()[1])
                    self.publish_int(self.target_count_pub, '/fruit_harvest/target_count', count)
                except:
                    print("예: target 5")
                continue

            # harvested
            if user_input.startswith('harvested '):
                try:
                    count = int(user_input.split()[1])
                    self.publish_int(self.harvested_count_pub, '/fruit_harvest/harvested_count', count)
                except:
                    print("예: harvested 3")
                continue

            # line move
            if user_input.startswith('line '):
                val = user_input.split()[1]
                if val in [str(i) for i in range(1, 11)] or val == "warehouse":
                    self.publish_string(self.harv_robot_line_move_pub, '/harv_robot/line_move', val)
                else:
                    print("1~10 또는 warehouse")
                continue

            # harv docking
            if user_input.startswith('harv_docking '):
                cmd = user_input.split()[1]
                if cmd == "station":
                    self.publish_string(self.harv_robot_docking_move_pub,
                                        '/harv_robot/docking_move',
                                        "move_to_docking_station")
                elif cmd == "return":
                    self.publish_string(self.harv_robot_docking_move_pub,
                                        '/harv_robot/docking_move',
                                        "move_to_return")
                continue

            # /action (manual trigger)
            if user_input == 'end2':
                self.publish_string(self.action_pub, '/action', 'entering_end')
                continue

            # carry docking
            if user_input.startswith('carry_docking '):
                cmd = user_input.split()[1]
                if cmd == "station":
                    self.publish_string(self.carry_robot_docking_move_pub,
                                        '/carry_robot/docking_move',
                                        "move_to_docking_station")
                elif cmd == "return":
                    self.publish_string(self.carry_robot_docking_move_pub,
                                        '/carry_robot/docking_move',
                                        "move_to_return")
                continue

            print("알 수 없는 명령")


def main(args=None):
    rclpy.init(args=args)

    node = HarvestDebugNode()

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


if __name__ == '__main__':
    main()