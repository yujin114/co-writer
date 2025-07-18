<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python" />
  <img src="https://img.shields.io/badge/OpenCV-4.x-green?logo=opencv" />
  <img src="https://img.shields.io/badge/Doosan-M0609-lightgrey?logo=doosan" />
  <img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg?logo=apache" />
</p>


# 🖋️ co-writer

✍️ **필기 로봇팔 프로젝트 - `co-writer`**  
**LLM**을 활용해 문장에서 키워드를 추출하고, 해당 키워드를 기반으로 **YOLO 객체 인식** 모델을 통해 펜 색을 인식 및 추적했습니다. 이후, 또 다른 키워드를 통해 글자를 추출하고, 이를 자모 단위로 분해하여 YAML 파일에서 경로 데이터를 불러온 뒤, **자모 조합 알고리즘**을 통해 글자 단위의 경로를 생성하였습니다. 두산 협동 로봇 **M0609**과 **OnRobot RG2 그리퍼**를 활용하여 **글자 모방을 수행**하는 ROS 2 기반 프로젝트입니다.

---

## ✅ 주요 기능

- LLM을 활용한 키워드 추출 및 글자 생성
- YOLO 기반 펜 색 추적 알고리즘 구현
- 자모 단위 YAML 데이터 조합을 통한 글자 경로 생성
- 로봇 팔 제어를 통한 자동 글씨 작성 시스템 구현

---

## 🔧 사전 준비

### Teach Pendant 설정
- **Tool 설정**: `Tool Weight10`  
- **TCP 설정**: `GripperDA_v10`

---

## 🚀 실행 방법

터미널 1 (로봇 제어 노드 실행)
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real model:=m0609 host:=192.168.1.100
```

터미널 2 (UI 실행)
```bash
ros2 run pick_and_place_voice gui
```

터미널 3 (전체 제어)
```bash
ros2 launch pick_and_place_voice draw_robot_launch
```

---
  </tr>
</table>


