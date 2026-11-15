#!/usr/bin/env python3
"""生成同心双码纹理；直接用 OpenCV 字典图案，物理尺寸见模型 SDF。"""
from pathlib import Path
import cv2


def generate():
    """0.28 m 对应 1120 px：ID5 黑边 800 px/0.20 m，ID18 黑边 80 px/0.02 m。"""
    dictionary = cv2.aruco.getPredefinedDictionary(10)
    outer = cv2.aruco.drawMarker(dictionary, 5, 800)
    texture = cv2.copyMakeBorder(outer, 160, 160, 160, 160,
                               cv2.BORDER_CONSTANT, value=255)
    inner = cv2.aruco.drawMarker(dictionary, 18, 80)
    # 5 mm 白色隔离边。整个覆盖区为 3 cm，大小码中心完全重合。
    inner = cv2.copyMakeBorder(inner, 20, 20, 20, 20,
                             cv2.BORDER_CONSTANT, value=255)
    texture[500:620, 500:620] = inner
    _, ids, _ = cv2.aruco.detectMarkers(texture, dictionary)
    if ids is None or set(ids.flatten()) != {5, 18}:
        raise RuntimeError('双码纹理未通过 OpenCV 识别检查')
    target = Path(__file__).resolve().parents[1] / 'models/aruco_demo_marker/materials/textures/marker5.png'
    if not cv2.imwrite(str(target), texture):
        raise OSError('纹理写入失败: ' + str(target))
    print(target)


if __name__ == '__main__':
    generate()
