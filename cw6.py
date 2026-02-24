import brickpi3
import time
import cv2
import numpy as np
from picamera2 import Picamera2

BP = brickpi3.BrickPi3()

picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()

starttime = time.time()
white = (255, 255, 255)
font = cv2.FONT_HERSHEY_SIMPLEX

# ----------------------------
# HOMOGRAPHY CALIBRATION SETUP
# ----------------------------
(x1, y1, u1, v1) = (12, -8, 111.6845553822153, 439.6443057722309)
(x2, y2, u2, v2) = (12, 10, 586.3474809160306, 431.92763358778626)
(x3, y3, u3, v3) = (46, -14, 162.99382716049382, 126.81481481481481)
(x4, y4, u4, v4) = (46, 16, 494.9620253164557, 119.65822784810126)

A = np.array([
    [x1, y1, 1, 0, 0, 0, -u1*x1, -u1*y1],
    [0, 0, 0, x1, y1, 1, -v1*x1, -v1*y1],
    [x2, y2, 1, 0, 0, 0, -u2*x2, -u2*y2],
    [0, 0, 0, x2, y2, 1, -v2*x2, -v2*y2],
    [x3, y3, 1, 0, 0, 0, -u3*x3, -u3*y3],
    [0, 0, 0, x3, y3, 1, -v3*x3, -v3*y3],
    [x4, y4, 1, 0, 0, 0, -u4*x4, -u4*y4],
    [0, 0, 0, x4, y4, 1, -v4*x4, -v4*y4],
])
b = np.array([u1, v1, u2, v2, u3, v3, u4, v4])

R, residuals, rank, sing = np.linalg.lstsq(A, b, rcond=None)

H = np.array([
    [R[0], R[1], R[2]],
    [R[3], R[4], R[5]],
    [R[6], R[7], 1.0]
])

HInv = np.linalg.inv(H)

def HtransformUVtoXY(HInv, uin, vin):
    uvec = np.array([uin, vin, 1.0], dtype=float)
    xvec = HInv.dot(uvec)
    xout = xvec[0] / xvec[2]
    yout = xvec[1] / xvec[2]
    return (xout, yout)

# ----------------------------
# MAIN LOOP
# ----------------------------
for i in range(1000):
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Red threshold (two ranges)
    lower_red0 = np.array([0, 50, 50])
    upper_red0 = np.array([10, 255, 255])
    mask0 = cv2.inRange(hsv, lower_red0, upper_red0)

    lower_red1 = np.array([170, 50, 50])
    upper_red1 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

    mask = cv2.bitwise_or(mask0, mask1)
    mask_bin = (mask > 0).astype(np.uint8) * 255

    # Optional cleanup helps stabilize boxes
    kernel = np.ones((5, 5), np.uint8)
    mask_bin = cv2.morphologyEx(mask_bin, cv2.MORPH_OPEN, kernel, iterations=1)
    mask_bin = cv2.morphologyEx(mask_bin, cv2.MORPH_CLOSE, kernel, iterations=1)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_bin, connectivity=8)

    min_area = 200

    # For visualization (don’t draw on the masked output image directly)
    vis = img.copy()

    objects = []  # each entry: {"pix": (bl, br), "robot": (bl_xy, br_xy)}

    for label in range(1, num_labels):  # skip background label 0
        x, y, w, h, area = stats[label]
        if area < min_area:
            continue

        x_min, y_min = x, y
        x_max, y_max = x + w - 1, y + h - 1

        # Bottom corners in IMAGE PIXELS (u,v)
        bl_uv = (int(x_min), int(y_max))
        br_uv = (int(x_max), int(y_max))

        # Convert IMAGE (u,v) -> ROBOT (x,y)
        bl_xy = HtransformUVtoXY(HInv, bl_uv[0], bl_uv[1])
        br_xy = HtransformUVtoXY(HInv, br_uv[0], br_uv[1])

        objects.append({"pix": (bl_uv, br_uv), "robot": (bl_xy, br_xy)})

        # Debug draw
        cv2.circle(vis, bl_uv, 5, (0, 255, 0), -1)
        cv2.circle(vis, br_uv, 5, (0, 255, 0), -1)
        cv2.rectangle(vis, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        # Optional: label with robot coords near bottom-left
        label_txt = f"BL=({bl_xy[0]:.1f},{bl_xy[1]:.1f})"
        cv2.putText(vis, label_txt, (bl_uv[0], bl_uv[1] - 10), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

    # Print robot coordinates for bottom corners
    for idx, obj in enumerate(objects):
        (bl_uv, br_uv) = obj["pix"]
        (bl_xy, br_xy) = obj["robot"]
        print(
            f"Obj {idx}: "
            f"BL pix={bl_uv} -> robot=({bl_xy[0]:.3f}, {bl_xy[1]:.3f}), "
            f"BR pix={br_uv} -> robot=({br_xy[0]:.3f}, {br_xy[1]:.3f})"
        )

    cv2.imwrite("demo.jpg", vis)
    print("drawImg:" + "/home/pi/prac-files/demo.jpg")
    print("Captured image", i, "at time", time.time() - starttime)

picam2.stop()
