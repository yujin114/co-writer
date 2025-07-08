import yaml
import os
import matplotlib.pyplot as plt

from typing import Dict, List

# 자모 파일 이름 (예: 'g.yaml', 'a.yaml', 'ng.yaml')
JAMO_YAML_DIR = "/home/yujin/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/config/jamo"  # 경로 맞게 수정

def load_jamo_yaml(jamo_filename: str) -> Dict:
    file_path = os.path.join(JAMO_YAML_DIR, jamo_filename)
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"파일이 없습니다: {file_path}")
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def visualize_strokes(strokes: List[Dict], title=""):
    plt.figure(figsize=(5, 5))
    for i, stroke in enumerate(strokes):
        pts = stroke['points']
        xs, ys = zip(*pts)
        plt.plot(xs, ys, marker='o', label=f"Stroke {i+1}")
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.show()

def test_single_jamo(filename: str):
    jamo_data = load_jamo_yaml(filename)
    strokes = jamo_data['strokes']
    visualize_strokes(strokes, title=filename)

if __name__ == "__main__":
    # 예시: 'g.yaml', 'a.yaml', 'ng.yaml' 등 하나씩 확인
    test_single_jamo("wae.yaml")
