"""Align the thermal view to the visible view. Run on the PC with a saved pair from --record.

  python calibrate_fusion.py PAIR_vis.jpg PAIR_thm.png
Click 4+ matching points: first a point on the VISIBLE window, then the same real-world point on
the THERMAL window (use a heated object, a window edge, a warm cup, a person's head). Press Enter
when done (Esc to abort). Writes fusion_calib.json next to this script. Use points spread across the
image and at the distance you will really use: parallax makes close/far objects disagree.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

vis, thm = cv2.imread(sys.argv[1]), cv2.imread(sys.argv[2])
if vis is None or thm is None:
    sys.exit("could not read images")
sv = 900 / vis.shape[1]
st = 3
vshow, tshow = cv2.resize(vis, None, fx=sv, fy=sv), cv2.resize(thm, None, fx=st, fy=st, interpolation=cv2.INTER_NEAREST)
vp, tp, want = [], [], "v"


def click(event, x, y, flags, which):
    global want
    if event != cv2.EVENT_LBUTTONDOWN or want != which:
        return
    if which == "v":
        vp.append((x / sv, y / sv))
        want = "t"
    else:
        tp.append((x / st, y / st))
        want = "v"
    print(f"pair {len(tp)} {'needs thermal click' if want == 't' else 'done'}")


cv2.namedWindow("visible"); cv2.namedWindow("thermal")
cv2.setMouseCallback("visible", click, "v"); cv2.setMouseCallback("thermal", click, "t")
while True:
    a, b = vshow.copy(), tshow.copy()
    for p in vp: cv2.circle(a, (int(p[0] * sv), int(p[1] * sv)), 5, (0, 255, 0), -1)
    for p in tp: cv2.circle(b, (int(p[0] * st), int(p[1] * st)), 5, (0, 255, 0), -1)
    cv2.imshow("visible", a); cv2.imshow("thermal", b)
    k = cv2.waitKey(30) & 0xFF
    if k == 27:
        sys.exit("aborted")
    if k == 13 and len(tp) >= 4 and want == "v":
        break
H, inl = cv2.findHomography(np.float32(tp), np.float32(vp), cv2.RANSAC, 15.0)
err = np.linalg.norm(cv2.perspectiveTransform(np.float32(tp)[None], H)[0] - np.float32(vp), axis=1)
print(f"mean reprojection error {err.mean():.1f} px (visible pixels) over {len(tp)} points")
out = Path(__file__).resolve().parent / "fusion_calib.json"
out.write_text(json.dumps({"H": H.tolist(), "vis_w": vis.shape[1], "vis_h": vis.shape[0]}, indent=2))
print("wrote", out)
