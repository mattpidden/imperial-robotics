import brickpi3
import time
import cv2
import numpy as np
from picamera2 import Picamera2
import math

BP = brickpi3.BrickPi3()
BP.set_motor_limits(BP.PORT_A, 100, 720)
BP.set_motor_limits(BP.PORT_B, 100, 720)

BARRIERRADIUS = 6
ROBOTRADIUS = 12
W = 2 * ROBOTRADIUS # width of robot
SAFEDIST = 20      # used in the cost function for avoiding obstacles

MAXVELOCITY = 50     #ms^(-1) max speed of each wheel
MAXACCELERATION = 50

FORWARDWEIGHT = 12
OBSTACLEWEIGHT = 16

target = (350, 0)

locationhistory = []

x = 0
y = 0
theta = 0

vL = 0.00
vR = 0.00

dt = 0.2
barriers = []
pathstodraw = []


picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()

starttime = time.time()
white = (255, 255, 255)
font = cv2.FONT_HERSHEY_SIMPLEX

CM_PER_DEGREE = 107 /(360 * 5)
CM_PER_DEG = 14.1/90 
# ----------------------------
# DRIVING FUNCTIONS
# ----------------------------
def velocity_to_dps(vel_cm_s):
    """Convert linear wheel velocity (cm/s) to motor degrees-per-second.
    Uses calibrated CM_PER_DEGREE from mattscw3.py."""
    return vel_cm_s / CM_PER_DEGREE

def set_motor_speeds(vL_cm_s, vR_cm_s):
    """Send wheel velocities to BrickPi motors (Port A = left, Port B = right)."""
    BP.set_motor_dps(BP.PORT_A, int(velocity_to_dps(vL_cm_s)))
    BP.set_motor_dps(BP.PORT_B, int(velocity_to_dps(vR_cm_s)))


def stop_motors():
    BP.set_motor_dps(BP.PORT_A, 0)
    BP.set_motor_dps(BP.PORT_B, 0)

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

def HtransformUVtoXY(HInv, uin, vin):
    uvec = np.array([uin, vin, 1.0], dtype=float)
    xvec = HInv.dot(uvec)
    xout = xvec[0] / xvec[2]
    yout = xvec[1] / xvec[2]
    return (xout, yout)


HInv = np.linalg.inv(H)

# ----------------------------
# FUNCTIONS
# ----------------------------
# Function to predict new robot position based on current pose and velocity controls
# Uses time deltat in future
# Returns xnew, ynew, thetanew
# Also returns path. This is just used for graphics, and returns some complicated stuff
# used to draw the possible paths during planning. Don't worry about the details of that.
def predictPosition(vL, vR, x, y, theta, deltat):
        # Simple special cases
        # Straight line motion
        if (vL == vR): 
                xnew = x + vL * deltat * math.cos(theta)
                ynew = y + vL * deltat * math.sin(theta)
                thetanew = theta
                path = (0, vL * deltat)   # 0 indicates pure translation
        # Pure rotation motion
        elif (vL == -vR):
                xnew = x
                ynew = y
                thetanew = theta + ((vR - vL) * deltat / W)
                path = (1, 0) # 1 indicates pure rotation
        else:
                # Rotation and arc angle of general circular motion
                # Using equations given in Lecture 2
                R = W / 2.0 * (vR + vL) / (vR - vL)
                deltatheta = (vR - vL) * deltat / W
                xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
                ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
                thetanew = theta + deltatheta
                path = (1, 0)
                # To calculate parameters for arc drawing (complicated Pygame stuff, don't worry)
                # We need centre of circle
                # (cx, cy) = (x - R * math.sin(theta), y + R * math.cos (theta))
                # # Turn this into Rect
                # Rabs = abs(R)
                # ((tlx, tly), (Rx, Ry)) = ((int(u0 + k * (cx - Rabs)), int(v0 - k * (cy + Rabs))), (int(k * (2 * Rabs)), int(k * (2 * Rabs))))
                # if (R > 0):
                #         start_angle = theta - math.pi/2.0
                # else:
                #         start_angle = theta + math.pi/2.0
                # stop_angle = start_angle + deltatheta
                # path = (2, ((tlx, tly), (Rx, Ry)), start_angle, stop_angle) # 2 indicates general motion

        return (xnew, ynew, thetanew, path)

# Function to calculate the closest obstacle at a position (x, y)
# Used during planning
def calculateClosestObstacleDistance(x, y):
        closestdist = 100000.0  
        # Calculate distance to closest obstacle
        for barrier in barriers:
                dx = barrier[0] - x
                dy = barrier[1] - y
                d = math.sqrt(dx**2 + dy**2)
                # Distance between closest touching point of circular robot and circular barrier
                dist = d - BARRIERRADIUS -      ROBOTRADIUS
                if (dist < closestdist):
                        closestdist = dist
        return closestdist

def convertFromRobotCoordsToWorldCoords(x_r, y_r):
    # Convert robot-centric coordinates to world coordinates
    x_w = x + x_r * math.cos(theta) - y_r * math.sin(theta)
    y_w = y + x_r * math.sin(theta) + y_r * math.cos(theta)
    return (x_w, y_w)


def observe():
    global barriers
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
    barriers = []
    for idx, obj in enumerate(objects):
        (bl_uv, br_uv) = obj["pix"]
        (bl_xy, br_xy) = obj["robot"]
        # calculate center of object in world coordinates
        center_xy = ((bl_xy[0] + br_xy[0]) / 2, (bl_xy[1] + br_xy[1]) / 2)
        center_xy_wc = convertFromRobotCoordsToWorldCoords(center_xy[0], center_xy[1])
        # if new barrier is not similar to any existing barrier, add to barriers list
        # if all(math.sqrt((center_xy_wc[0] - b[0])**2 + (center_xy_wc[1] - b[1])**2) > BARRIERRADIUS*2 for b in barriers):
        barriers.append(center_xy_wc)
        

    print(f"Barriers: {len(barriers)}: {[f'({b[0]:.1f},{b[1]:.1f})' for b in barriers]}")
    # cv2.imwrite("demo.jpg", vis)
    # print("drawImg:" + "/home/pi/prac-files/demo.jpg")

# ----------------------------
# MAIN LOOP
# ----------------------------
prev_time = time.time()
try:
    while(1):
        loop_start = time.time()
        actual_dt = loop_start - prev_time
        prev_time = loop_start

        locationhistory.append((x, y))
        observe()

        bestBenefit = -100000
        # Range of possible motions: each of vL and vR could go up or down a bit
        vLpossiblearray = (vL - MAXACCELERATION * dt, vL, vL + MAXACCELERATION * dt)
        vRpossiblearray = (vR - MAXACCELERATION * dt, vR, vR + MAXACCELERATION * dt)
        print (f"RobotX: {x:.1f}, RobotY: {y:.1f}")

        for vLpossible in vLpossiblearray:
            for vRpossible in vRpossiblearray:
                # We can only choose an action if it's within velocity limits
                if (vLpossible <= MAXVELOCITY and vRpossible <= MAXVELOCITY and vLpossible >= -MAXVELOCITY and vRpossible >= -MAXVELOCITY):
                    # Predict new position in TAU seconds
                    TAU = 2.5 
                    (xpredict, ypredict, thetapredict, path) = predictPosition(vLpossible, vRpossible, x, y, theta, TAU)
                    # What is the distance to the closest obstacle from this possible position?
                    distanceToObstacle = 1e6
                    steps = 10
                    for i in range(1, steps+1):
                        t = TAU * i / steps
                        xp, yp, _, _ = predictPosition(vLpossible, vRpossible, x, y, theta, t)
                        d = calculateClosestObstacleDistance(xp, yp)
                        distanceToObstacle = min(distanceToObstacle, d)
                    # Calculate how much close we've moved to target location
                    previousTargetDistance = math.sqrt((x - target[0])**2 + (y - target[1])**2)
                    newTargetDistance = math.sqrt((xpredict - target[0])**2 + (ypredict - target[1])**2)
                    distanceForward = previousTargetDistance - newTargetDistance
                    # Alternative: how far have I moved forwards?
                    # distanceForward = xpredict - x
                    # Positive benefit
                    distanceBenefit = FORWARDWEIGHT * distanceForward
                    # Small background cost always present (gentle nudge away)
                    # + strong linear ramp when within SAFEDIST
                    obstacleCost = OBSTACLEWEIGHT * (1.0 / max(distanceToObstacle, 0.1))
                    if distanceToObstacle < SAFEDIST:
                        obstacleCost += OBSTACLEWEIGHT * (SAFEDIST - distanceToObstacle)
                    
                    #print(f"Distance to obstacle: {distanceToObstacle:.1f}, distance forward: {distanceForward:.1f}, benefit: {distanceBenefit:.1f}, obstacle cost: {obstacleCost:.1f}")
                    # Total benefit function to optimise
                    benefit = distanceBenefit - obstacleCost
                    if (benefit > bestBenefit):
                        vLchosen = vLpossible
                        vRchosen = vRpossible
                        bestBenefit = benefit
        vL = vLchosen
        vR = vRchosen

        print(f"Chosen vL: {vL:.2f}, vR: {vR:.2f}, benefit: {bestBenefit:.2f}")

        # DRIVE MOTORS
        set_motor_speeds(vR, vL)


        (x, y, theta, tmppath) = predictPosition(vL, vR, x, y, theta, actual_dt)
        disttotarget = math.sqrt((x - target[0])**2 + (y - target[1])**2)

        hz = 1.0 / actual_dt if actual_dt > 0 else 0.0
        print(f"Loop Hz: {hz:.1f}  actual_dt: {actual_dt:.4f}s")

        if (disttotarget < 10): 
            break

        time.sleep(max(0, dt - (time.time() - loop_start)))

except KeyboardInterrupt:
    print("Keyboard interrupt — stopping motors")

finally:
    stop_motors()
    picam2.stop()
    BP.reset_all()


picam2.stop()
