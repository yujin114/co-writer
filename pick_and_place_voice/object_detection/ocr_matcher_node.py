import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import easyocr
from PIL import Image as PILImage, ImageDraw, ImageFont

# ===== 한글 폰트 설정 =====
FONT_PATH = "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"
FONT_SIZE = 24
FONT = ImageFont.truetype(FONT_PATH, FONT_SIZE)


def draw_text_with_pil(frame, text, position, font=FONT, color=(0, 255, 0)):
    pil_img = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)
    draw.text(position, text, font=font, fill=color[::-1])
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


class OCRMatcherNode(Node):
    """ROS 토픽(/camera/color/image_raw)을 구독해서 180° 회전 OCR 수행"""

    def __init__(self):
        super().__init__("ocr_matcher_node")
        self.get_logger().info("📷 OCR Matcher Node 시작됨 (ROS 이미지 구독)")

        # ---- Parameters ----
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("rotate", 180)  # 회전 각도 (deg)
        self.declare_parameter("timeout_sec", 15.0)

        img_topic = self.get_parameter("image_topic").value
        self.rotate_deg = int(self.get_parameter("rotate").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        # ---- Publishers & Subs ----
        self.result_pub = self.create_publisher(String, "/ocr_result", 10)
        self.expected_text = None
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

        transient_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(String, "/write_text", self.expected_text_callback, transient_qos)

        self.create_subscription(Image, img_topic, self.image_callback, 10)

        # ---- Tools ----
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(["ko"])

        # ---- State ----
        self.matched = False
        self.start_time = self.get_clock().now()

    # ---------------- Callbacks ----------------
    def expected_text_callback(self, msg: String):
        self.expected_text = msg.data.strip()
        self.get_logger().info(f"✍️ 기준 문장 수신: '{self.expected_text}'")

    def image_callback(self, msg: Image):
        if self.matched:
            return  # 이미 매칭 완료

        # ---- Timeout 검사 ----
        if (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 > self.timeout_sec:
            self._publish_and_shutdown(matched=False, reason="타임아웃")
            return

        # ---- 이미지 변환 ----
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.rotate_deg == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotate_deg == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_deg == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # ---- OCR ----
        results = self.reader.readtext(frame)
        for bbox, text, _ in results:
            clean = text.strip()
            self.get_logger().info(f"OCR 인식 결과: '{clean}' vs 기대값: '{self.expected_text}'")
            is_match = self.expected_text and clean == self.expected_text
            if is_match:
                self.matched = True
                self._draw_bbox(frame, bbox, clean, match=True)
                self._publish_and_shutdown(matched=True)
                return
            else:
                self._draw_bbox(frame, bbox, clean, match=False)

    # ---------------- Helpers ----------------
    def _draw_bbox(self, frame, bbox, text, match):
        (x1, y1), (_, _), (x2, y2), _ = bbox
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        color = (0, 255, 0) if match else (0, 0, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{text} 일치" if match else f"{text} 불일치"
        frame = draw_text_with_pil(frame, label, (x1, y1 - 25), color=color)
        cv2.imshow("OCR 결과", frame)
        cv2.waitKey(1)

    def _publish_and_shutdown(self, matched: bool, reason: str = ""):
        result = "matched" if matched else "unmatched"
        self.result_pub.publish(String(data=result))
        log_fn = self.get_logger().info if matched else self.get_logger().warn
        log_fn(f"🧠 OCR 평가 결과: {result} {reason}")
        cv2.destroyAllWindows()
        # 노드 종료
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OCRMatcherNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
