from setuptools import find_packages, setup
import glob
import os

package_name = 'pick_and_place_voice'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=find_packages(include=[
        'robot_control', 
        'voice_processing', 
        'object_detection',
        'word_trajectory',
        'gui',
    ]),

        data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
        ('share/' + package_name + '/resource', glob.glob('resource/.env')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')), 
        ('share/' + package_name + '/config/jamo', glob.glob('config/jamo/*.yaml')),
        # ('share/ament_index/resource_index/packages',['resource/' + 'voice_processing']),
        # ('share/voice_processing', ['package.xml']),
        # ('share/object_detection', ['package.xml']),
        # ('share/robot_control', ['package.xml']),

        # ('share/' + package_name + '/launch', glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey4090',
    maintainer_email='rokey4090@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'robot_control = robot_control.robot_control:main',
            'robot_draw_fsm = robot_control.robot_draw_fsm:main',
            'robot_draw_fsm1 = robot_control.robot_draw_fsm1:main',
            'robot_draw_test = robot_control.robot_draw_test:main',
            'detection = object_detection.detection:main',
            'ocr_matcher_node = object_detection.ocr_matcher_node:main',
            'get_keyword = voice_processing.get_keyword:main',
            'visual = word_trajectory.visual:main',
            'control = word_trajectory.control_robot:main',
            'r = word_trajectory.r:main',
            'gui = gui.writing_gui:main',
        ],
    },
)
