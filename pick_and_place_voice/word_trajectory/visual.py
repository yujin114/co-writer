#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
visual.py ── STT 기반 Trajectory Publisher (ROS 2 / rclpy)

Topics
------
Subscriber:
  • /write_text   (std_msgs/String)   :  STT 결과 문자열
  • /font_style   (std_msgs/Int32)    :  0 = 굵게 / 1 = 얇게

Publisher:
  • /dsr01/all_chars_trajectory (std_msgs/Float32MultiArray)
    ├─ layout.dim[0] = points (N)  ─ size = N, stride = 4N
    └─ layout.dim[1] = coords (4) ─ size = 4, stride = 4
        [x (mm), y (mm), z (mm), stroke_id]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32MultiArray, MultiArrayDimension
from word_trajectory.char_assemble3 import assemble_sentence_strokes
from typing import List, Dict
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class SttTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('stt_trajectory_publisher')

        # ---------------- Parameters ----------------
        self.declare_parameter('stt_topic', '/write_text')
        self.declare_parameter('font_style_topic', '/font_style')
        self.declare_parameter('trajectory_topic', '/dsr01/all_chars_trajectory')
        self.declare_parameter('bold_height', 2.0)  # mm (pen-down depth)
        self.declare_parameter('thin_height', 3.0)  # mm (pen-down depth)

        stt_topic: str = self.get_parameter('stt_topic').get_parameter_value().string_value
        font_style_topic: str = self.get_parameter('font_style_topic').get_parameter_value().string_value
        self.traj_topic: str = self.get_parameter('trajectory_topic').get_parameter_value().string_value

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # 퍼블리셔가 마지막 메시지를 저장
        qos.reliability = QoSReliabilityPolicy.RELIABLE


        # ---------------- State ----------------
        self.font_style: int = 0  # 0 = bold / 1 = thin

        # ---------------- ROS I/O ----------------
        self.create_subscription(String, stt_topic, self._stt_cb, qos)
        self.create_subscription(Int32, font_style_topic, self._font_cb, 10)
        self.traj_pub = self.create_publisher(Float32MultiArray, self.traj_topic, 10)


        # 예외 처리용
        self.btn_publisher = self.create_publisher(Int32, '/btn_topic', qos)

        self.get_logger().info(
            f"📝 Listening to STT text on '{stt_topic}'  →  publishing trajectories on '{self.traj_topic}'.")

    def _font_cb(self, msg: Int32):
        self.font_style = int(msg.data)
        self.get_logger().debug(f"Font‑style updated → {self.font_style}")

    def _stt_cb(self, msg: String):
        text: str = msg.data.strip()
        if not text:
            self.get_logger().warn('⛔︎ Received empty STT string. Ignoring.')
            ########
            msg = Int32()
            msg.data = 0
            self.btn_publisher.publish(msg)
            return
        self.get_logger().info(f"✒️  Assembling trajectory for: '{text}' …")
        self._publish_trajectory(text)

    def _publish_trajectory(self, text: str):
        try:
            strokes: List[Dict] = assemble_sentence_strokes(text)
        except Exception as e:
            self.get_logger().error(f"assemble_sentence_strokes() failed: {e}")
            return

        z_pen = (self.get_parameter('bold_height').get_parameter_value().double_value
                 if self.font_style == 0 else
                 self.get_parameter('thin_height').get_parameter_value().double_value)

        flat: List[float] = []
        for stroke in strokes:
            sid = stroke.get("stroke_id", 0)
            for pt in stroke["points"]:
                x_mm, y_mm = pt
                flat += [float(x_mm), float(y_mm), float(z_pen), float(sid)]

        # ---- Build message ----
        msg_arr = Float32MultiArray()
        n_points = len(flat) // 4
        msg_arr.layout.dim = [
            MultiArrayDimension(label='points', size=n_points, stride=4 * n_points),
            MultiArrayDimension(label='coords', size=4, stride=4)
        ]
        msg_arr.data = flat
        self.traj_pub.publish(msg_arr)
        self.get_logger().info(f"✅ Published {n_points} points, {len(strokes)} strokes → {self.traj_topic}")


def main(args=None):
    rclpy.init(args=args)
    node = SttTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
