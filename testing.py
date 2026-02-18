import math

CM_PER_DEG = 14.1/90 

def drive_rotate(angle_radians):
    print(f"drive_rotate called with angle_radians: {angle_radians}")
    if angle_radians > math.pi:
        angle_radians = math.pi - angle_radians
    if angle_radians < -math.pi:
        angle_radians = -math.pi - angle_radians
    print(f"adjusted angle_radians: {angle_radians}")
    angle_degrees = math.degrees(angle_radians)
    print(f"angle degrees: {angle_degrees}")
    left = angle_degrees > 0
    print(f"Rotation direction: {'left' if left else 'right'}")
    calc_distance = CM_PER_DEG * angle_degrees
    print(f"Calculated distance for rotation left: {-calc_distance}, right: {calc_distance}")
    # drive_distance(-calc_distance, calc_distance)


waypoints = []
waypoints.append((180,30))
waypoints.append((180,54))
waypoints.append((138,54))
waypoints.append((138,168))
waypoints.append((114,168))
waypoints.append((114,84))
waypoints.append((84,84))
waypoints.append((84,30))     

robot_x = 84
robot_y = 30
robot_a = 0

for waypoint in waypoints:
    print(f"\nSTARTING NEW WAYPOINT: {waypoint}")
    w_x, w_y = waypoint

    dx = w_x - robot_x
    dy = w_y - robot_y
    print(f"rob_x: {robot_x}, rob_y: {robot_y}, rob_a: {robot_a}, dx: {dx}, dy: {dy}")
    angle = math.atan2(dy, dx) - robot_a
    # if angle > math.pi:
    #     angle = math.pi - angle
    # distance = math.sqrt(dx*dx + dy*dy)
    # print(f"distance: {distance}, angle: {angle}")


    drive_rotate(angle)
    robot_x = w_x
    robot_y = w_y
    robot_a += angle
    print(f"Updated robot position: x: {robot_x}, y: {robot_y}, a: {robot_a}")

    