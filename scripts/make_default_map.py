"""app/assets/maps/default.png を作る(10 m 四方、0.05 m/px、原点は中心)。

外周の壁、部屋を分ける壁と出入口、柱、円い障害物を置く。
使い方: python3 scripts/make_default_map.py
"""
from pathlib import Path

from PIL import Image, ImageDraw

RES = 0.05
SIZE = 10.0
N = int(SIZE / RES)  # 200 px


def px(x, y):
    """世界座標 [m] を画像の画素に直す(原点は中心、上が +y)"""
    return int(round((x + SIZE / 2) / RES)), int(round((SIZE / 2 - y) / RES))


def rect(d, x0, y0, x1, y1):
    a, b = px(x0, y1), px(x1, y0)
    d.rectangle([a, (b[0] - 1, b[1] - 1)], fill=0)


img = Image.new("L", (N, N), 255)
d = ImageDraw.Draw(img)
t = 0.1  # 壁の厚さ
rect(d, -5, -5, 5, -5 + t)
rect(d, -5, 5 - t, 5, 5)
rect(d, -5, -5, -5 + t, 5)
rect(d, 5 - t, -5, 5, 5)
# x = 1.5 の仕切り壁。y = -1〜1 が出入口
rect(d, 1.5, -5, 1.5 + t, -1)
rect(d, 1.5, 1, 1.5 + t, 5)
# 柱
rect(d, -3.0, 2.5, -2.5, 3.0)
rect(d, -3.0, -3.0, -2.5, -2.5)
# 右の部屋の円い障害物
cx, cy = px(3.3, 2.5)
r = int(0.4 / RES)
d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=0)
# 右下の不明な領域(灰色)
rect(d, 3.5, -4.9, 4.9, -3.5)
a, b = px(3.5, -3.5), px(4.9, -4.9)
d.rectangle([a, (b[0] - 1, b[1] - 1)], fill=205)

out = Path(__file__).resolve().parent.parent / "app" / "assets" / "maps" / "default.png"
img.save(out, optimize=True)
print(out)
