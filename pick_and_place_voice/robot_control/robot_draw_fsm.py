#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Float32MultiArray, Int32
from std_srvs.srv import Trigger
import numpy as np
import time
import subprocess
import os
# ─── Doosan / OnRobot ────────────────────────────
from .onrobot import RG
from robot_control.coordinate_utils import transform_to_base

GRIPPER_NAME     = "rg2"
TOOLCHARGER_IP   = "192.168.1.1"
TOOLCHARGER_PORT = 502
GRIPPER2CAM_PATH = os.path.join(
    os.path.dirname(__file__),
    "resource",
    "T_gripper2camera.npy"
)


def main(args=None):
    rclpy.init()

    import DR_init
    DR_init.__dsr__id    = "dsr01"
    DR_init.__dsr__model = "m0609"
    DR_init.__dsr__node  = rclpy.create_node("dsr_init_node", namespace="dsr01")

    import DSR_ROBOT2
    DSR_ROBOT2.g_node = DR_init.__dsr__node
    from DSR_ROBOT2 import movel, movej, wait, set_digital_output, get_current_posx, DR_TOOL,DR_AXIS_Z, check_force_condition, DR_BASE
    from DR_common2 import posx

    movel(posx([357, 56, 180, 0, 180, 0]), vel=30, acc=30)

    class RobotDrawFSM(Node):
        def __init__(self):
            super().__init__("robot_draw_fsm")

            qos = QoSProfile(depth=10,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=QoSReliabilityPolicy.RELIABLE)

            self.text_pub      = self.create_publisher(String, '/write_text', qos)
            self.pen_color_pub = self.create_publisher(String, '/pen_color', qos)
            self.create_subscription(Float32MultiArray, '/dsr01/all_chars_trajectory', self.trajectory_callback, 10)
            self.create_subscription(Float32MultiArray, '/pen_position', self.pen_position_callback, 10)
            # self.create_subscription(String, '/pen_color', self.pen_color_callback, 10)
            self.ocr_trigger = self.create_publisher(Int32, '/ocr_check', qos)
            self.create_subscription(String, '/ocr_result', self.ocr_result_callback, 10)
            self.state_pub = self.create_publisher(String, 'state_topic',10)
            self.timer = self.create_timer(1.0, self.timer_callback)

            self.btn_sub = self.create_subscription(Int32, '/btn_topic', self.btn_callback, qos)


            self.keyword_client = self.create_client(Trigger, '/get_keyword')
            while not self.keyword_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn("⏳ /get_keyword 서비스 대기 중...")

            self.state               = 'WAIT_FOR_WAKEWORD'
            self.latest_pen_position = None
            self.trajectory          = []
            self.pen_color           = None
            self.ocr_result          = None

            self.approach = None
            self.grip = None
            self.retreat = None

            self.pen_up   = posx([480, 0, 10, 0, 180, 0])
            self.pen_down = posx([480, 0, -5, 0, 180, 0])

            self.pen_heights_up = 33
            self.pen_heights_down = 21


            self.pen_heights = {
                'black_pen': {'up': self.pen_heights_up - 22, 'down': self.pen_heights_down - 21},
                'red_pen':   {'up': self.pen_heights_up, 'down': self.pen_heights_down - 0.5},
                'blue_pen':  {'up': self.pen_heights_up, 'down': self.pen_heights_down}
            }
            self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
            self.gripper2cam = np.load(GRIPPER2CAM_PATH)

            # movel(posx([357, 56, 180, 0, 180, 0]), vel=30, acc=30)
            self.run_fsm()

        def run_fsm(self):
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

                if self.state == 'WAIT_FOR_WAKEWORD':
                    movel(posx([357, 56, 180, 0, 180, 0]), vel=30, acc=30)

                    self.get_logger().info("🎧  'Hello Rokey'라고 말하세요...")
                    self.latest_pen_position = None
                    self.ocr_result = None
                    self.state = 'CALL_KEYWORD_SERVICE'

                elif self.state == 'CALL_KEYWORD_SERVICE':
                    future = self.keyword_client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(self, future)

                    if not future.result():
                        self.get_logger().warn("❌ /get_keyword 서비스 호출 실패")
                        time.sleep(1.0)
                        continue

                    # ── ① 문자열 전처리 ───────────────────────────
                    msg_text = future.result().message.strip()

                    # (1) 아예 빈 문자열이면
                    if not msg_text:
                        self.get_logger().warn("❌ 잘못된 명령입니다. 다시 말씀해주세요.")
                        self.state = 'WAIT_FOR_WAKEWORD'
                        continue

                    # (2) “Hello Rokey” 웨이크워드일 때
                    if msg_text.lower().startswith("hello rokey"):
                        self.get_logger().info("🎤 음성 명령 대기 중…")
                        self.state = 'WAIT_FOR_COMMAND'
                        continue

                    # (3) 펜색과 텍스트를 분리 → 최소 2토큰 필요
                    tokens = msg_text.split()
                    if len(tokens) < 2:
                        self.get_logger().warn("❌ 잘못된 명령입니다. 다시 말씀해주세요.")
                        self.state = 'WAIT_FOR_WAKEWORD'
                        continue

                    pen_color, *text_tokens = tokens
                    text_to_draw = " ".join(text_tokens).strip()

                    # (4) 둘 중 하나라도 비어 있으면 오류
                    if not pen_color or not text_to_draw:
                        self.get_logger().warn("❌ 잘못된 명령입니다. 다시 말씀해주세요.")
                        self.state = 'WAIT_FOR_WAKEWORD'
                        continue

                    # ── ② 정상 처리 ───────────────────────────
                    self.pen_color = pen_color.lstrip('/')
                    self.pen_color_pub.publish(String(data=self.pen_color))

                    text_to_draw = text_to_draw.lstrip('/')
                    self.text_pub.publish(String(data=text_to_draw))

                    # self.get_logger().info(f"🖊️ 펜 색: {self.pen_color}, 텍스트: {text_to_draw}")
                    self.state = 'WAIT_FOR_TRAJECTORY'

                elif self.state == 'WAIT_FOR_COMMAND':
                    self.state = 'CALL_KEYWORD_SERVICE'

                elif self.state == 'WAIT_FOR_TRAJECTORY':
                    # self.get_logger().info(f"📢 펜 색상 발행: {self.pen_color}")
                    # self.pen_color_pub.publish(String(data=self.pen_color))
                    # self.get_logger().info("📨 펜 좌표 수신 대기 중...")

                    # timeout = 7.0
                    # start = time.time()
                    # while rclpy.ok() and self.latest_pen_position is None:
                    #     rclpy.spin_once(self, timeout_sec=0.1)
                    #     if time.time() - start > timeout:
                    #         self.get_logger().warn("⏱️ 펜 좌표 타임아웃. 상태 초기화")
                    #         self.state = 'WAIT_FOR_WAKEWORD'
                    #         return
                    # self.get_logger().info(f"📍 펜 좌표 수신 완료: {self.latest_pen_position}")
                    # self.state = 'SELECT_PEN'
                    pass

                elif self.state == 'SELECT_PEN':
                    # self.get_logger().info(f"🔍 현재 펜 색상: {self.pen_color}")

                    # self.get_logger().info(f"🔍 현재 펜 좌표: {self.latest_pen_position}")
                    if self.latest_pen_position is not None:
                        
                        self.select_pen(self.pen_color)
                        self.state = 'EXECUTE_DRAWING'
                    else:
                        self.get_logger().warn("📭 펜 좌표 수신 대기 중…")
                        time.sleep(0.5)

                elif self.state == 'EXECUTE_DRAWING':
                    self.execute_trajectory(self.trajectory)

                elif self.state == 'RUN_OCR_VALIDATION':
                    cap_position = posx([445, 6, 130, 28, -180, 28])
                    movel(cap_position, vel=50, acc=50)  # 💡 여기서 끝날 때까지 blocking 보장
                    self.get_logger().info("🏠 캡쳐 위치 완료")

                    time.sleep(7.0)  # 💡 프레임 안정화 시간

                    msg = Int32()
                    msg.data = 1
                    self.ocr_trigger.publish(msg)
                    self.get_logger().info("📤 OCR 트리거 전송 완료")

                    # OCR 결과 대기
                    timeout_sec = 15
                    start_time = time.time()
                    while rclpy.ok() and self.ocr_result is None:
                        rclpy.spin_once(self, timeout_sec=0.2)
                        if time.time() - start_time > timeout_sec:
                            self.get_logger().warn("⚠️ OCR 결과 타임아웃. 실패 처리.")
                            self.ocr_result = "unmatched"
                            break

                    if self.ocr_result == "matched":
                        self.get_logger().info("✅ OCR 통과! 종료합니다.")
                        self.state = 'DONE'
                        time.sleep(3.0)
                    else:
                        self.get_logger().warn("❌ OCR 실패. 다시 음성 명령을 기다립니다.")
                        self.state = 'WAIT_FOR_WAKEWORD'

        def timer_callback(self):
            msg = String()
            msg.data = self.state
            self.state_pub.publish(msg)

        def btn_callback(self, msg):
            btn = msg.data
        

            if btn == 0:
                self.get_logger().info("↩️ 상태를 WAIT_FOR_WAKEWORD로 변경")
                self.state = 'WAIT_FOR_WAKEWORD'
            
            elif btn == 1:
                self.get_logger().info("✋ 상태를 SELECT_PEN으로 변경")
                self.state = 'WAIT_FOR_TRAJECTORY'
            
            else:
                self.get_logger().warn("❗버튼 다시 눌러주세요")


        def update_pen_heights(self):
            heights = self.pen_heights.get(self.pen_color, {'up': 10, 'down': -5})
            self.pen_up   = posx([480, 0, heights['up'], 0, 180, 0])
            self.pen_down = posx([480, 0, heights['down'], 0, 180, 0])
            self.get_logger().info(f"🛠️ 펜 높이 설정됨 → up: {heights['up']}, down: {heights['down']}")

        def pen_color_callback(self, msg):
            self.pen_color = msg.data

        def trajectory_callback(self, msg):
            self.trajectory = msg.data
            self.update_pen_heights()  # 🛠️ 이 줄 추가!
            self.get_logger().info(f"📥 trajectory 수신, 포인트 수: {len(self.trajectory)//4}")
            self.state = 'SELECT_PEN'

        def pen_position_callback(self, msg):
            cam_xyz = np.array(msg.data[:3])
            base_xyz = transform_to_base(cam_xyz, self.gripper2cam)
            self.latest_pen_position = base_xyz
            self.get_logger().info(f"📍 펜 좌표(base): {base_xyz}")

        def ocr_result_callback(self, msg):
            self.ocr_result = msg.data.strip().lower()
            self.get_logger().info(f"🔍 OCR 결과 수신: {self.ocr_result}")

        def select_pen(self, color):
            if self.latest_pen_position is None:
                self.get_logger().warn("❌ 펜 좌표 없음")
                return

            x, y, _ = self.latest_pen_position
            current_pose = get_current_posx()[0]
            rx, ry, rz = current_pose[3:]

            self.approach = posx([x-10, y-20, 80,rx, ry, rz])
            self.grip     = posx([x-10, y-20, 30, rx, ry, rz])
            self.retreat  = posx([x-10, y-20, 200, rx, ry, rz])

            self.slow_gripper_move(800, 400)
            movel(self.approach, vel=20, acc=20)
            movel(self.grip,     vel=15, acc=15)
            self.slow_gripper_move(200, 190)
            wait(1.0)
            movel(self.retreat,  vel=40, acc=40)
            self.get_logger().info("🖊️ 펜 잡기 완료")

        def slow_gripper_move(self, start, end, steps=5, force=400, delay=0.15):
            for w in np.linspace(start, end, steps):
                self.gripper.move_gripper(int(w), force)
                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(delay)

        def execute_trajectory(self, traj):
            # movel(self.pen_up, vel=60, acc=60)
            movel(posx([480, 0, 150, 0, 180, 0]), vel=60, acc=60)


            points = [(traj[i], traj[i+1], traj[i+2], int(traj[i+3]))
                    for i in range(0, len(traj), 4)]

            strokes = {}
            for x, y, z, sid in points:
                strokes.setdefault(sid, []).append((x, y, z))

            for sid in sorted(strokes.keys()):
                stroke = strokes[sid]
                first_x, first_y, _ = stroke[0]

                approach = posx([480-first_x, first_y, self.pen_up[2], 0, 180, 0])

                movel(approach, vel=40, acc=40)

                down = posx([approach[0], approach[1], self.pen_down[2], 0, 180, 0])
                movel(down, vel=30, acc=30)
                if not check_force_condition(axis=DR_AXIS_Z, min = 10, ref= DR_BASE):
                    movel(self.retreat,  vel=40, acc=40)
                    movel(self.grip,     vel=15, acc=15)
                    self.slow_gripper_move(800, 400)
                    wait(1.0)
                    movel(self.retreat,  vel=40, acc=40)
                    self.get_logger().info("🖊️ 펜은 제자리에")
                    movel(posx([357, 56, 180, 0, 180, 0]), vel=30, acc=30)
                    self.state =  'WAIT_FOR_WAKEWORD'
                    self.pen_heights_down += 1
                    self.get_logger().info("펜 높이 재설정 ...")
                    return
                # set_digital_output(2, 1)

                for x, y, _ in stroke[1:]:
                    draw = posx([480-x, y, self.pen_down[2], 0, 180, 0])
                    movel(draw, vel=30, acc=30, radius=5)

                # set_digital_output(2, 0)
                movel(approach, vel=40, acc=40)
                wait(0.5)
                self.get_logger().info(f"✏️ Stroke {sid} 완료")
            movel(posx([0,0,-150,0,180,0]),vel=40, acc=40, ref=DR_TOOL)
            movel(self.pen_up, vel=60, acc=60)
            self.get_logger().info("✅ 글씨 쓰기 완료")

            if self.latest_pen_position is not None:
                # x, y, z = self.latest_pen_position
                # current_pose = get_current_posx()[0]
                # rx, ry, rz = current_pose[3:]

                # approach = posx([x-10, y-20, z+10, rx, ry, rz])
                # drop     = posx([x-10, y-20, z-35, rx, ry, rz])
                # retreat  = posx([x-10, y-20, z+90, rx, ry, rz])

                movel(self.retreat, vel=30, acc=30)
                movel(self.grip,     vel=20, acc=20)
                self.slow_gripper_move(190, 400)
                wait(1.0)
                movel(self.retreat,  vel=30, acc=30)
                self.get_logger().info("🧷 펜 내려놓기 완료")
            else:
                self.get_logger().warn("❗ 펜 집었던 위치 정보 없음, 드롭 생략")

            self.approach = None
            self.grip = None
            self.retreat = None

            # cap_position = posx([443, -27.6, 366.37, 96, -177, 96.37])
            # movel(cap_position, vel=50, acc=50)
            # self.get_logger().info("🏠 캡쳐 위치 완료")
            self.state = 'RUN_OCR_VALIDATION'

    node = RobotDrawFSM()
    rclpy.shutdown()

if __name__ == '__main__':
    main()