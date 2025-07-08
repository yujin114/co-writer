from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        

        # 2. YOLO 기반 펜 색상 감지
        Node(
            package='pick_and_place_voice',
            executable='detection',
            name='detection',
            output='screen'
        ),

        # 3. 텍스트 → Trajectory 생성
        Node(
            package='pick_and_place_voice',
            executable='visual',
            name='visual',
            output='screen'
        ),

        # 4. FSM: 전체 흐름 제어 (펜 선택 + 로봇 제어)
        Node(
            package='pick_and_place_voice',
            executable='robot_draw_fsm',
            name='robot_draw_fsm',
            output='screen'
        ),
        # 1. 음성 → 키워드 추출
        Node(
            package='pick_and_place_voice',
            executable='get_keyword',
            name='get_keyword',
            output='screen')
    ])
