import numpy as np
import rclpy
from rclpy.node import Node
from typing import Tuple
from std_msgs.msg import String, Float32MultiArray, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import easyocr
from PIL import Image as PILImage, ImageDraw, ImageFont
from ament_index_python.packages import get_package_share_directory

from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import os
import time

PACKAGE_NAME = 'pick_and_place_voice'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
FONT_PATH = "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"
FONT = ImageFont.truetype(FONT_PATH, 24)

class ObjectDetectionNode(Node):
    def __init__(self, model_name='yolo'):
        super().__init__('detection')

        # QoS 설정
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        # 펜 색상 요청 → YOLO 감지
        self.pen_color_sub = self.create_subscription(
            String, '/pen_color', self.pen_color_callback, qos)

        self.pen_pose_pub = self.create_publisher(
            Float32MultiArray, '/pen_position', 10)

        # ✅ OCR 요청 처리용 토픽
        self.create_subscription(Int32, '/ocr_check', self.ocr_check_callback, qos)
        self.create_subscription(String, '/write_text', self.write_text_callback, qos)
        self.ocr_result_pub = self.create_publisher(String, '/ocr_result', 10)

        # 예외 처리용
        self.btn_publisher = self.create_publisher(Int32, '/btn_topic', qos)


        # 이미지 처리
        self.img_node = ImgNode()
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(["ko"])

        self.frame = None

        self.expected_text = None
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics")

        self.get_logger().info("✅ ObjectDetectionNode (YOLO + OCR) initialized.")

    # ────────────────────────────────────────
    # YOLO 감지
    # ────────────────────────────────────────
    def pen_color_callback(self, msg: String):
        pen_color = msg.data.strip().lower()
        self.get_logger().info(f"📨 요청된 펜 색상: {pen_color}")
        if pen_color not in ['red_pen', 'blue_pen', 'black_pen']:
            self.get_logger().warn(f"⚠️ 지원되지 않는 색상: {pen_color}")
            return

        box, score = self.model.get_best_detection(self.img_node, pen_color)
        if box and score > 0.3:
            coords = self._compute_position(pen_color)
            if coords:
                self.pen_pose_pub.publish(Float32MultiArray(data=list(coords)))
                self.get_logger().info(f"📤 좌표 퍼블리시: {coords}")
        else:
            self.get_logger().warn(f"❌ {pen_color} 감지 실패")
            ##############
            msg = Int32()
            msg.data = 0
            self.btn_publisher.publish(msg)


    def _load_model(self, name):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def _compute_position(self, target: str) -> Tuple[float, float, float]:
        box, score = self.model.get_best_detection(self.img_node, target)
        if not box or not score:
            self.get_logger().warn("No detection found.")
            return 0.0, 0.0, 0.0

        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        z = self._get_depth(cx, cy)
        if z is None or z == 0:
            self.get_logger().warn("Depth invalid.")
            return 0.0, 0.0, 0.0

        fx, fy = self.intrinsics["fx"], self.intrinsics["fy"]
        ppx, ppy = self.intrinsics["ppx"], self.intrinsics["ppy"]
        X = (cx - ppx) * z / fx
        Y = (cy - ppy) * z / fy
        return float(X), float(Y), float(z)

    def _get_depth(self, x, y):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            return None

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    # ────────────────────────────────────────
    # OCR 기능
    # ────────────────────────────────────────
    def write_text_callback(self, msg: String):
        self.expected_text = msg.data.strip()
        self.get_logger().info(f"📝 기준 문장 저장됨: '{self.expected_text}'")

    def ocr_check_callback(self, msg: Int32):
        self.get_logger().info("📸 OCR 체크 요청 수신됨")
        self.frame = self._get_latest_color_frame(duration_sec=5.0)

        if self.frame is None:
            self.get_logger().warn("❌ 이미지 프레임 없음")
            return

        self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
        results = self.reader.readtext(self.frame)
        # 현재 파일 기준 디버깅 이미지 저장 경로 설정
        save_path = os.path.join(os.path.dirname(__file__), "ocr_debug.jpg")
        cv2.imwrite(save_path, self.frame)
        self.get_logger().info(f"📸 디버깅용 이미지 저장됨: {save_path}")

        self.get_logger().info(f"📊 OCR 탐지 결과 개수: {len(results)}")


        for bbox, text, _ in results:
            clean = text.strip()
            is_match = self.expected_text and clean == self.expected_text
            self.get_logger().info(f"OCR 결과: '{clean}' vs 기대값: '{self.expected_text}'")
            if is_match:
                self.ocr_result_pub.publish(String(data="matched"))
                self.get_logger().info("✅ OCR 일치")
                self.frame = None
                return

        self.ocr_result_pub.publish(String(data="unmatched"))
        self.frame = None
        self.get_logger().warn("❌ OCR 불일치 또는 실패")
        

    def _get_latest_color_frame(self, duration_sec=5.0) -> np.ndarray:
        """
        지정된 시간(duration_sec) 동안 프레임을 계속 수신하고,
        가장 마지막 프레임을 반환한다.
        """
        start = time.time()
        latest_frame = None

        while time.time() - start < duration_sec:
            rclpy.spin_once(self.img_node, timeout_sec=0.1)
            frame = self.img_node.get_color_frame()
            if frame is not None and frame.any():
                latest_frame = frame

        if latest_frame is None:
            self.get_logger().warn("⚠️ 유효한 이미지 프레임을 수신하지 못했습니다.")
        return latest_frame
# ────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
