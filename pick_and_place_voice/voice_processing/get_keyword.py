# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"
import os
import rclpy
import pyaudio
from rclpy.node import Node
import re
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from std_msgs.msg import String
from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("pick_and_place_voice")
is_laod = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")
############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):
        super().__init__("get_keyword_node")

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )
        # self.text_pub = self.create_publisher(String, '/write_text', 10)

        prompt_content = """
            당신은 사용자의 문장에서 특정 도구와 글자(또는 단어)를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에서 각 도구로 쓰라고 지시한 "글자 또는 단어"를 추출하세요.
            - 글자나 단어는 어떤 단어든 가능하며, 미리 정의된 리스트는 없습니다.


            <도구 리스트>
            - red, blue, black

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / 글자1 글자2 ...]
            - 도구와 글자는 각각 공백으로 구분
            - 도구가 없으면 앞쪽은 공백 없이 비우고, 글자가 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 도구와 글자의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "파란 펜" → blue)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구와 글자가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.

            <추가 규칙>
            - 도구 이름은 반드시 "red_pen", "blue_pen", "black_pen" 중 하나로 추출하되, 다음과 같은 유사 표현도 포함됩니다:
            - "빨간색", "빨강이", "레드" → red_pen
            - "파란색", "파랑이", "블루" → blue_pen
            - "검정색", "검정이", "블랙" → black_pen

            - 도구 이름 뒤에 붙는 조사나 접미사(예: "으로", "갖고", "가지고", "을 써서")는 무시하고 핵심 도구만 추출하세요.

            - 명확한 도구 명칭이 없더라도 문맥상 해당 도구를 지시한다고 판단되면 적절한 도구로 추론하세요.
            (예: "파란 걸로 써줘" → blue_pen)

            - 도구 이름에 오타가 있는 경우에도 유사 발음을 고려해 적절한 도구로 추론하세요.
            (예: "빨강팬", "파란섹", "검정팩" → red_pen, blue_pen, black_pen)

            - 도구 이름은 반드시 "red_pen", "blue_pen", "black_pen" 중 하나로 추출하세요.

            - 띄어쓰기 오류를 유발하지 않도록, 사용자가 말한 문장 전체를 자연스럽게 분석한 후 결과에 반영하세요.

            - 글자는 문장에서 사용된 그대로의 **띄어쓰기, 철자, 순서**를 최대한 유지해서 추출하세요.
                (예: "코카콜라라고 써줘" → 코카콜라, "잘 자"라고 써줘 → 잘 자)


            <예시>
            - 입력: "빨간색 펜으로 코카콜라라고 써줘"  
            출력: red_pen / 코카콜라

            - 입력: "빨강색 펜으로 다라고 써줘"  
            출력: red_pen / 다

            - 입력: "파란색 펜으로 집중이라고 써줘"  
            출력: blue_pen / 집중

            - 입력: "검정색 펜 가져와"  
            출력: black_pen /

            - 입력: "검정색 펜으로 안녕하세요라 써줘"  
            출력: black_pen / 안녕하세요

            - 입력: "빨간색 펜으로 가라고 쓰고 파란색 펜으로 자라고 써줘"  
            출력: red_pen blue_pen / 가 자

            - 입력: "검은색 펜으로 만나서 반갑습니다라고 써줘"
            출력: black_pen / 만나서 반갑습니다.

            - 입력: "검은 걸로 어디가세요라고 써줘"
            출력: black_pen / 어디가세요

            - 입력: "파랑 펜으로 피피티 언제 만드냐라고 써줘"
            출력: blue_pen / 피피티 언제 만드냐 

            - 입력: "빨간색 펜으로 아이스 아메리카노는 좀 진해야 제 맛이지라고 써줘"
            출력: red_pen / 아이스 아메리카노는 좀 진해야 제 맛이지
            

            <사용자 입력>
            "{user_input}"                
        """
        
        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)

        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response["text"]

        object, target = result.strip().split("/")

        object = object.split()
        target = target.split()

        # ✅ 한글만 남기기
        target = [re.sub(r'[^가-힣]', '', word) for word in target]

        target = [word for word in target if word]

        print(f"llm's response: {result}")
        print(f"object: {object}")
        print(f"target: {target}")
        return object, target

    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return None

        while not self.wakeup_word.is_wakeup():
            pass

        # STT --> Keword Extract --> Embedding
        output_message = self.stt.speech2text()
        object_list, target_list = self.extract_keyword(output_message)

        self.get_logger().warn(f"Detected tools: {object_list}")
        self.get_logger().warn(f"Detected text : {target_list}")

        # ✅ 퍼블리시
        # text_msg = String()
        # text_msg.data = " ".join(target_list)
        # self.text_pub.publish(text_msg)
        # self.get_logger().info(f"📝 퍼블리시 완료: {text_msg.data}")

        # ✅ 응답 메시지 구성
        response.success = True
        response.message = " ".join(object_list) + " / " + " ".join(target_list)
        return response
    
def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
