# -*- coding: utf-8 -*-
"""
stroke_generator_b.py  (B안, 업그레이드)
==============================

글자당 고정 셀 폭(FIXED_ADVANCE) 내에서 자모 수(2글자/3글자)와 모음의 방향성(세로/가로/복합)에 따라 다른 오프셋을 적용하고,
자모 YAML 기준 좌상단 (0, 0) 정렬을 전제로 위치 보정을 수행.
"""

import os
import yaml
from typing import List, Dict

from jamo import h2j, j2hcj
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# ────────────────────────────────────────────────────
# 상수 정의
# ──────────────────────────────────────────────────
JAMO_YAML_DIR = "config/jamo"
VERTICAL_JUNGSEONG = ["ㅏ", "ㅑ", "ㅓ", "ㅕ", "ㅣ", "ㅐ", "ㅒ", "ㅔ", "ㅖ"]
COMPLEX_JUNGSEONG = ["ㅚ", "ㅙ", "ㅞ", "ㅢ", "ㅘ", "ㅝ", "ㅟ"]
FIXED_ADVANCE = 22
DEFAULT_SCALE = 1.0

# 모든 글자 크기를 균일하게 만들기 위한 고정 스케일
POSITION_SCALE_3 = {0: 0.98, 1: 1.2, 2: 0.9}
POSITION_SCALE_2 = {0: DEFAULT_SCALE, 1: 1.5}

# ──────────────────────────────────────────────────
# 기본 유틸
# ──────────────────────────────────────────────────
def decompose_hangul(text: str) -> List[List[str]]:
    return [[j for j in j2hcj(h2j(ch))] for ch in text]

def _convert_jamo_to_filename(jamo: str) -> str:
    jamo_map = {
        "ㄱ": "g", "ㄲ": "gg", "ㄴ": "n", "ㄷ": "d", "ㄸ": "dd", "ㄹ": "r",
        "ㅁ": "m", "ㅂ": "b", "ㅃ": "bb", "ㅅ": "s", "ㅆ": "ss", "ㅇ": "ng",
        "ㅈ": "j", "ㅉ": "jj", "ㅊ": "ch", "ㅋ": "k", "ㅌ": "t", "ㅍ": "p", "ㅎ": "h",
        "ㅏ": "a", "ㅐ": "ae", "ㅑ": "ya", "ㅒ": "yae", "ㅓ": "eo", "ㅔ": "e",
        "ㅕ": "yeo", "ㅖ": "ye", "ㅗ": "o", "ㅘ": "wa", "ㅙ": "wae", "ㅚ": "oe",
        "ㅛ": "yo", "ㅜ": "u", "ㅝ": "wo", "ㅞ": "we", "ㅟ": "wi", "ㅠ": "yu",
        "ㅡ": "eu", "ㅢ": "ui", "ㅣ": "i",
    }
    return jamo_map.get(jamo, jamo)

def _load_jamo_yaml(jamo: str) -> Dict:
    filename = _convert_jamo_to_filename(jamo)
    share_dir = get_package_share_directory("pick_and_place_voice")
    fp = os.path.join(share_dir, JAMO_YAML_DIR, f"{filename}.yaml")
    if not os.path.exists(fp):
        raise FileNotFoundError(f"자모 YAML 파일이 없습니다: {fp}")
    with open(fp, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

# 자모별 offset 계산 (YAML 기준 좌상단 0,0)
def _get_jamo_offsets(jamos: List[str]) -> List[Dict]:
    """초·중·종 위치 오프셋 계산.

    ▸ 글자 셀은 29 mm(종성 O) / 28 mm(종성 X) 기준 설계.
    ▸ 모음(세로·가로·복합)에 따라 **종성 위치(x·y)** 를 다르게 둔다.
      - 세로형 모음 : 종성 x≈8, y≈mid_y+4 (모음 기둥 바로 아래)
      - 가로형 모음 : 종성 x≈6, y≈mid_y+6 (가로선 아래쪽)
      - 복합   모음 : 종성 x≈6, y≈mid_y+4 (복합 중앙 하단)
    """

    has_jong = len(jamos) == 3
    jung = jamos[1]

    total_h = 29 if has_jong else 28
    mid_y = total_h / 2
    offsets: List[Dict] = []

    # ── 초성 ─────────────────────────────
    is_seoro = jung in VERTICAL_JUNGSEONG
    if is_seoro and has_jong:
        x_off = 4.5
    elif is_seoro and not has_jong:
        x_off = 4.5
    else:
        x_off = 6

    y_off = mid_y - (10.5 if has_jong else 8)

    offsets.append({
        "jamo": jamos[0],
        "offset": [x_off, y_off]
    })


    # ── 중성 ─────────────────────────────
    if jung in VERTICAL_JUNGSEONG:
        offsets.append({"jamo": jung, "offset": [17.5, mid_y - 11]})

    elif jung in COMPLEX_JUNGSEONG:
        offsets.append({"jamo": jung, "offset": [3, mid_y - 6.5]})
    else:  # 가로형(ㅗ·ㅜ·ㅡ)
        y_off = mid_y - 1 if has_jong else mid_y + 6
        offsets.append({"jamo": jung, "offset": [3.25, y_off]})

    # ── 종성 ─────────────────────────────
    if has_jong:
        if jung in VERTICAL_JUNGSEONG:
            jong_off = [8, mid_y + 2.5]          # 세로형 모음 아래 중앙
        elif jung in COMPLEX_JUNGSEONG:
            jong_off = [6, mid_y + 4]          # 복합 모음 중앙 하단
        else:  # 가로형
            jong_off = [6, mid_y + 5]          # 가로 모음 아래쪽(더 낮음)
        offsets.append({"jamo": jamos[2], "offset": jong_off})

    return offsets

def _assemble_full_stroke(jamos: List[str]) -> List[Dict]:
    scale_tbl = POSITION_SCALE_3 if len(jamos) == 3 else POSITION_SCALE_2
    strokes: List[Dict] = []
    has_jong = len(jamos) == 3
    jung = jamos[1]

    for idx, comp in enumerate(_get_jamo_offsets(jamos)):
        jamo = comp["jamo"]
        off_x, off_y = comp["offset"]

        # 기본 scale
        scale = scale_tbl.get(idx, DEFAULT_SCALE)

        # ✅ 중성(jung)이고, 세로형 + 종성 있는 경우 → 축소 적용
        if idx == 1 and has_jong and jung in VERTICAL_JUNGSEONG:
            scale *= 0.8  # 예: 기존보다 10% 축소 (원하면 0.85 등도 가능)

        for stroke in _load_jamo_yaml(jamo)["strokes"]:
            pts = [[x * scale + off_x, y * scale + off_y] for x, y in stroke["points"]]
            strokes.append({"points": pts, "z_offset": stroke.get("z_offset", 5.0)})

    return strokes


def assemble_sentence_strokes(text: str, spacing: int = FIXED_ADVANCE) -> List[Dict]:
    strokes: List[Dict] = []
    stroke_id = 0
    char_idx = 0  # 띄어쓰기도 반영할 글자 인덱스

    for ch in text:
        if ch == " ":
            char_idx += 1  # spacing 만큼 이동만 하고 stroke는 없음
            continue

        try:
            jamos = [j for j in j2hcj(h2j(ch))]  # 자모 분해 시도
        except Exception:
            print(f"⚠️ 자모 분해 실패 → '{ch}' 무시됨")
            char_idx += 1
            continue

        offset_x = char_idx * spacing

        for s in _assemble_full_stroke(jamos):
            strokes.append({
                "points": [[x + offset_x, y] for x, y in s["points"]],
                "z_offset": s["z_offset"],
                "stroke_id": stroke_id  # stroke ID로 사용
            })
            stroke_id += 1

        char_idx += 1

    return strokes


def strokes_to_multiarray(strokes: List[Dict], fixed_z: float = 2.0) -> Float32MultiArray:
    data = []
    for s in strokes:
        for x, y in s["points"]:
            data.extend([x, y, fixed_z, float(s["stroke_id"])])   # ★ char_idx → stroke_id
    msg = Float32MultiArray()
    msg.data = data
    msg.layout.dim.append(
        MultiArrayDimension(label="coords_xyzs", size=len(data)//4, stride=len(data))
    )
    return msg

def main():
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm

    plt.rc("font", family=fm.FontProperties(fname="/usr/share/fonts/truetype/nanum/NanumGothic.ttf").get_name())

    text = "안녕 잘 가"
    strokes = assemble_sentence_strokes(text)

    plt.figure(figsize=(7, 6))
    for idx, s in enumerate(strokes):
        xs, ys = zip(*s["points"])
        plt.plot(xs, ys, marker="o"); plt.text(xs[0], ys[0], str(idx), fontsize=8)
    plt.gca().invert_yaxis(); plt.axis("equal"); plt.grid(True)
    plt.title("네모칸 기준 좌상단 정렬 + 셀 간격 적용")
    plt.show()

if __name__ == "__main__":
    main()
