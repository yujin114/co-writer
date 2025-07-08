import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from PIL import Image, ImageTk
from tkinter import Canvas
import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import time
from PIL import Image
# ROS2 노드 정의
class SpeechtoDrawGUI(Node):
    def __init__(self):
        super().__init__('speech_command_subscriber')
        
        qos = QoSProfile(depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)

        self.btn_publisher = self.create_publisher(Int32, '/btn_topic', qos)

        self.text_subscription = self.create_subscription(
            String,
            '/write_text',
            self.text_listener_callback,
            qos
        )

        self.pen_color_subscription = self.create_subscription(
            String,
            '/pen_color',
            self.pen_color_listener_callback,
            qos
        )

        self.image_subscription = self.create_subscription(
            String, '/ocr_result',self.generate_image_callback, 10
        )

        self.state_sub = self.create_subscription(String, '/state_topic', self.state_callback, 10)

        # 초기 상태 설정
        self.state = 'WAIT_FOR_COMMAND'

        # GUI 생성
        self.root = tk.Tk()
        self.root.title("Voice Command Writing GUI")
        self.root.geometry("600x760")

        # 배경 이미지 추가
        bg_path = os.path.join(
            os.path.dirname(__file__),
            "gui",
            "background.jpg"
        )
        self.bg_image = Image.open(bg_path)

        self.bg_image = self.bg_image.resize((580, 320))
        self.bg_photo = ImageTk.PhotoImage(self.bg_image)

        # Canvas 위젯 생성
        self.canvas = Canvas(self.root, width=580, height=320)
        self.canvas.pack(fill="both", expand=True)

        # 배경 이미지 삽입
        self.canvas.create_image(300, 0, image=self.bg_photo, anchor="n")

        # 폰트 로드
        try:
            self.butpen_font = tkfont.Font(family="Hakgyoansim Butpen", size=28, weight="normal")
        except Exception as e:
            self.butpen_font = tkfont.Font(family="Helvetica", size=28, weight="bold")
            print(f"폰트 로드 실패: {e}")

        # 제목 레이블 (tk.Label 사용, 폰트 적용)
        self.title_label = tk.Label(self.root, text="음성 명령으로 글씨써조", font=self.butpen_font)
        self.title_label.place(x=40, y=20, width=500, height=60)

        # 상태 레이블
        self.l1 = ttk.Label(self.root, text="Status", font=("Helvetica", 14), anchor="center")
        self.l1.place(x=40, y=130, width=100, height=20)
        self.status_label = ttk.Label(self.root,
                            text="음성 명령 대기 중...",
                            font=("Helvetica", 14),
                            borderwidth=2,
                            relief="sunken",
                            anchor="center")
        self.status_label.place(x=150, y=120, width=400, height=30)

        # 텍스트 레이블
        self.l2 = ttk.Label(self.root, text="Text", font=("Helvetica", 14), anchor="center")
        self.l2.place(x=40, y=170, width=100, height=20)
        self.text_label = ttk.Label(self.root,
                            text="",
                            font=("Helvetica", 14),
                            borderwidth=2,
                            relief="sunken",
                            anchor="center")
        self.text_label.place(x=150, y=160, width=400, height=30)

        # 펜 색상 레이블
        self.l3 = ttk.Label(self.root, text="Pen Color", font=("Helvetica", 14), anchor="center")
        self.l3.place(x=40, y=210, width=100, height=20)
        self.pen_color_label = ttk.Label(self.root,
                            text="",
                            font=("Helvetica", 14),
                            borderwidth=2,
                            relief="sunken",
                            anchor="center")
        self.pen_color_label.place(x=150, y=200, width=400, height=30)

        # "Speak Again" 버튼
        self.speak_again_button = tk.Button(
                            self.root,
                            text="Speak Again",
                            font=("Helvetica", 14),
                            command=self.handle_speak_again,
                            borderwidth=5,
                            bg="#000000",
                            fg="white")
        self.speak_again_button.place(x=150, y=250, width=180, height=40)

        # "Draw Again" 버튼
        # self.draw_again_button = tk.Button(
        #                     self.root,
        #                     text="Draw Again",
        #                     font=("Helvetica", 14),
        #                     command=self.handle_draw_again,
        #                     borderwidth=5,
        #                     bg="#000000",
        #                     fg="white"
        # )
        # self.draw_again_button.place(x=340, y=250, width=180, height=40)

    def generate_image_callback(self, msg):
        result = msg.data
        if result:
            time.sleep(3.0)
            ocr_path = os.path.join(os.path.dirname(__file__), "ocr_debug.jpg")
            self.ocr_image = Image.open(ocr_path)
            self.ocr_image = self.ocr_image.resize((560,400))
            self.ocr_image = ImageTk.PhotoImage(self.ocr_image)
            self.image_label = tk.Label(self.root, image=self.ocr_image)
            self.image_label.place(x=20, y=340)

    def state_callback(self, msg):
        self.state = msg.data

        if self.state == 'WAIT_FOR_WAKEWORD':
            self.status_label.config(text="음성 명령 대기 중...")
            # self.clear_image()
        elif self.state == 'CALL_KEYWORD_SERVICE':
            self.status_label.config(text="서비스 대기 중...")
        elif self.state == 'EXECUTE_DRAWING':
            self.status_label.config(text="글 쓰기 중...")
        # elif self.state == 'DRAW_COMPLETED':
        #     self.status_label.config(text="그리기 완료!")
        # elif self.state == 'ERROR':
        #     self.status_label.config(text="에러 발생!")
        elif self.state == 'SELECT_PEN':
            self.status_label.config(text="펜 집는중...")
        elif self.state == 'RUN_OCR_VALIDATION':
            self.status_label.config(text="OCR 검사 중...")
        elif self.state == 'DONE':
            self.status_label.config(text="OCR 성공 !")    
        else:
            pass
    
    def handle_speak_again(self):
        msg = Int32()
        msg.data = 0
        self.btn_publisher.publish(msg)
        self.get_logger().info("🎤 'Speak Again' 버튼 눌림, 상태 변경: WAIT_FOR_COMMAND")
       
    def handle_draw_again(self):
        msg = Int32()
        msg.data = 1
        self.btn_publisher.publish(msg)
        self.get_logger().info("🖊️ 'Draw Again' 버튼 눌림, 상태 변경: EXECUTE_DRAWING")

    def text_listener_callback(self, msg):
        command = msg.data  # 텍스트 수신
        self.get_logger().info(f"음성 명령 텍스트 수신: {command}")
        self.text_label.config(text=f"text: {command}")
        self.state = 'EXECUTE_DRAWING'  # 그리기 시작 상태로 변경
        
    def pen_color_listener_callback(self, msg):
        pen_color = msg.data
        self.get_logger().info(f"펜 색상 수신: {pen_color}")

        # 펜 색상을 GUI에 표시
        if pen_color == "black_pen":
            self.pen_color_label.config(text="black pen")
        elif pen_color == "red_pen":
            self.pen_color_label.config(text="red pen")
        elif pen_color == "blue_pen":
            self.pen_color_label.config(text="blue pen")
        else:
            self.pen_color_label.config(text="Unknown color")

    # def clear_image(self):
    #     self.image_label.config(image=None)
    #     image_path = ""
    #     if os.path.exists(image_path):
    #         os.remove(image_path)
    #         self.get_logger().info(f"파일 삭제 완료: {image_path}")
    #     else:
    #         self.get_logger().warn(f"파일을 찾을 수 없습니다: {image_path}")

    # # Tkinter GUI 루프 업데이트
    # def update_gui_loop(self):
    # # 주기적으로 상태 업데이트
    #     self.root.after(100, self.update_gui_loop)

    def ros_spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)  # ROS 콜백 처리
        self.root.after(100, self.ros_spin_once) 

def main(args=None):
    rclpy.init(args=args)
    
    # ROS2 노드 실행
    gui = SpeechtoDrawGUI()

    gui.root.after(100, gui.ros_spin_once)
    # Tkinter GUI 실행
    gui.root.mainloop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
