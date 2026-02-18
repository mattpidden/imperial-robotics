#!/usr/bin/env python 
import sys
import time
import random
import math
import sys
import time
import random
import math                           
import brickpi3 
BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP.reset_all()

# GLOBAL CONSTANTS
MOTOR_TOLERANCE = 5
CM_PER_DEGREE = 107 /(360 * 5)
CM_PER_DEG = 14.1/90 
SONAR_OFFSET = 5
NUM_PARTICLES = 200
MU = 0
SIGMA_FORWARD = 5
SIGMA_ROTATE = 0.1
SIGMA_ROTATE_ONLY = 0.1
SIGMA_SONAR = 3
EPS = 1e-3
K = 0.01
STEP_SIZE = 20

# DEBUGGING
num_min_walls_none = 0


# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size    # in cm
        self.canvas_size = 768         # in pixels
        self.margin      = 0.05*map_size
        self.scale       = self.canvas_size/(map_size+2*self.margin)

    def drawLine(self,line):
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print ("drawLine:" + str((x1,y1,x2,y2)))

    def drawParticles(self,data):
        display = []
        for d in data:
            p =  (self.__screenX(d.x), self.__screenY(d.y), d.a, d.w)
            display.append(p)
            #print(p)
        print ("drawParticles:" + str(display))

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = []

    def add_wall(self,wall):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall)

class Particle:
    def __init__(self, n_particles, x = 0, y = 0, a = 0):
        self.x = x
        self.y = y
        self.a = a
        self.w = 1 / n_particles
        self.p_lowerbound = 0
        self.p_upperbound = 1
        
    def update(self, x, y, a, w):
        self.x = x
        self.y = y
        self.a = a
        self.w = w
        
    def get_particle(self):
        return self.x, self.y, self.a
    
    def update_weight(self, w):
        self.w = w
        
    def update_probabilites(self, p_lowerbound, p_upperbound):
        self.p_lowerbound = p_lowerbound
        self.p_upperbound = p_upperbound


# Simple Particles set
class Particles:
    def __init__(self, x=0, y=0, a=0):
        self.n = NUM_PARTICLES    
        self.data = []
        
        for i in range(self.n):
            particle = Particle(self.n, x, y, a)
            self.data.append(particle)

    def update(self, distance, sonar_readings):
        total_weight = 0
        
        for particle in self.data:
            x, y, a = particle.get_particle()

            dx = (distance + random.gauss(MU, SIGMA_FORWARD)) * math.cos(particle.a)
            dy = (distance + random.gauss(MU, SIGMA_FORWARD)) * math.sin(particle.a)
            da = random.gauss(MU, SIGMA_ROTATE)

            particle.update(x + dx, y + dy, a + da, particle.w)
            x, y, a = particle.get_particle()

            combined_likelihood = 1.0
            for sensor_angle, dist_measured in sonar_readings.items():
                # The actual heading the sonar was pointing
                target_theta = a + math.radians(sensor_angle)
                L = calculate_likelihood(x, y, target_theta, dist_measured)
                
                # If calculate_likelihood returns -1, use the penalty constant K
                combined_likelihood *= (L if L != -1 else K)
            
            particle.update_weight(particle.w * combined_likelihood)
            total_weight += particle.w
            
        cumulative_weight = 0
        for i, particle in enumerate(self.data):
            #normalized_weight = 0
            # if likelihood_sum == 0:
            #     normalized_weight = 1 / self.n
            normalized_weight = particle.w / total_weight
            particle.update_weight(normalized_weight)
            if i == len(self.data) - 1:
                particle.update_probabilites(cumulative_weight, 1)
            else:
                particle.update_probabilites(cumulative_weight, cumulative_weight + normalized_weight)
                cumulative_weight += normalized_weight
        
    def update_rotate(self, angle, sonar_readings):
        total_weight = 0
        
        for particle in self.data:
            x, y, a = particle.get_particle()

            dx = 0
            dy = 0
            da = angle + random.gauss(MU, SIGMA_ROTATE)

            particle.update(x + dx, y + dy, a + da, particle.w)
            x, y, a = particle.get_particle()

            combined_likelihood = 1.0
            for sensor_angle, dist_measured in sonar_readings.items():
                target_theta = a + math.radians(sensor_angle)
                L = calculate_likelihood(x, y, target_theta, dist_measured)
                combined_likelihood *= (L if L != -1 else K)
            
            particle.update_weight(particle.w * combined_likelihood)
            total_weight += particle.w
            
        cumulative_weight = 0
        for i, particle in enumerate(self.data):
            #normalized_weight = 0
            # if likelihood_sum == 0:
            #     normalized_weight = 1 / self.n
            normalized_weight = particle.w / total_weight
            particle.update_weight(normalized_weight)
            if i == len(self.data) - 1:
                particle.update_probabilites(cumulative_weight, 1)
            else:
                particle.update_probabilites(cumulative_weight, cumulative_weight + normalized_weight)
                cumulative_weight += normalized_weight
        
        
    
    def resample(self):
        temp_particles = []
        
        for i in range(self.n):
            random_num = random.random()
            for particle in self.data:
                if particle.p_lowerbound <= random_num and random_num < particle.p_upperbound:
                    new_particle = Particle(self.n, particle.x, particle.y, particle.a)
                    temp_particles.append(new_particle)          
                    
        if len(temp_particles) != self.n:
            print(f"ERROR: Temp particles incorrect length {len(temp_particles)}. Should be {self.n}")
        else:
            self.data = temp_particles
            
            
        #self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)]
    
    def draw(self):
        canvas.drawParticles(self.data)

    
    def estimate_current_pos(self):
        mean_x = 0
        mean_y = 0
        sum_sin = 0
        sum_cos = 0
        
        for particle in self.data:
            mean_x += particle.x * particle.w
            mean_y += particle.y * particle.w
            sum_sin += math.sin(particle.a) * particle.w
            sum_cos += math.cos(particle.a) * particle.w
        
        mean_a = math.atan2(sum_sin, sum_cos)
        return mean_x, mean_y, mean_a


def calculate_likelihood(x, y, theta, sonar_distance):
    min_dist = sys.float_info.max
    min_wall = None

    #print(f"calculate_likelihood for new particle: x: {x}, y: {y}, theta: {theta}, z: {z}")

    for wall in mymap.walls:
        ax, ay, bx, by = wall

        #print(f"checking wall for new particle: ax: {ax}, ay: {ay}, bx: {bx}, by: {by}")

        # Maybe add a tiny epsilon for the denominator to avoid divide by zero
        m_numerator = (by - ay)*(ax - x) - (bx-ax)*(ay - y)
        m_denominator = (by - ay)*math.cos(theta) - (bx - ax)*math.sin(theta)

        if abs(m_denominator) < EPS:
            print("DENOMINATOR IS 0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            continue

        m = m_numerator/m_denominator
        #print(f"m: {m}")

        if m > EPS:
            intersect_x = x + m*math.cos(theta)
            intersect_y = y + m*math.sin(theta)
            #print(f"intersect_x: {intersect_x}, intersect_y: {intersect_y}")

            in_horizontal_range = min(ax,bx) - EPS <= intersect_x <= max(ax,bx) + EPS
            in_vertical_range   = min(ay,by) - EPS <= intersect_y <= max(ay,by) + EPS

            #print(f"in horizontal range: {in_horizontal_range}, in vertical range: {in_vertical_range}")

            if in_horizontal_range and in_vertical_range:
                #print("passed in range checks")
                if m < min_dist:
                    #print("m is less than mis dist, updating min_dist and min_wall")
                    min_dist = m
                    min_wall = wall

    if min_wall is not None:
        likelihood = math.exp((-(sonar_distance-min_dist)**2) / (2*SIGMA_SONAR**2)) + K
        return likelihood
    else:
        #print(f"NO WALL: x: {x}, y: {y}, theta: {theta}, z: {z}")
        #print(f"NO WALL: m: {m}, y: {y}, theta: {theta}, z: {z}")
        return -1


def drive_distance(left_cm, right_cm):
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))

    left_target_degrees = left_cm / CM_PER_DEGREE
    right_target_degrees = right_cm / CM_PER_DEGREE
    BP.set_motor_position(BP.PORT_A, left_target_degrees)
    BP.set_motor_position(BP.PORT_B, right_target_degrees)


    done_a = False
    done_b = False

    while True:
        status_a, power_a, enc_a, dps_a = BP.get_motor_status(BP.PORT_A)
        status_b, power_b, enc_b, dps_b = BP.get_motor_status(BP.PORT_B)
        
        if (not done_a) and abs(enc_a - left_target_degrees) < MOTOR_TOLERANCE:
            BP.set_motor_power(BP.PORT_A, 0)
            done_a = True
        if (not done_b) and abs(enc_b - right_target_degrees) < MOTOR_TOLERANCE:
            BP.set_motor_power(BP.PORT_B, 0)
            done_b = True
            
        if done_a and done_b:
            print("Finished via MOTOR_TOLERANCE")
            break
            
        time.sleep(0.05)
    time.sleep(0.1)


def drive_rotate(angle_radians):
    print(f"drive_rotate called with angle_radians: {angle_radians}")
    print(f"adjusted angle_radians: {angle_radians}")
    angle_degrees = math.degrees(angle_radians)
    print(f"angle degrees: {angle_degrees}")
    left = angle_degrees > 0
    print(f"Rotation direction: {'left' if left else 'right'}")
    calc_distance = CM_PER_DEG * angle_degrees
    print(f"Calculated distance for rotation left: {-calc_distance}, right: {calc_distance}")
    drive_distance(-calc_distance, calc_distance)



canvas = Canvas()
mymap = Map()
mymap.add_wall((0,0,0,168))        # a
mymap.add_wall((0,168,84,168))     # b
mymap.add_wall((84,126,84,210))    # c
mymap.add_wall((84,210,168,210))   # d
mymap.add_wall((168,210,168,84))   # e
mymap.add_wall((168,84,210,84))    # f
mymap.add_wall((210,84,210,0))     # g
mymap.add_wall((210,0,0,0))        # h
mymap.draw()

waypoints = []
waypoints.append((180,30))
waypoints.append((180,54))
waypoints.append((138,54))
waypoints.append((138,168))
waypoints.append((114,168))
waypoints.append((114,84))
waypoints.append((84,84))
waypoints.append((84,30))     


BP.set_motor_limits(BP.PORT_A, 50, 360)
BP.set_motor_limits(BP.PORT_B, 50, 360)
BP.set_motor_limits(BP.PORT_D, 50, 360)
BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.NXT_ULTRASONIC)
time.sleep(1)

# global robot positions and angle
robot_x = 84
robot_y = 30
robot_a = 0

particles = Particles(robot_x, robot_y, robot_a)

def rotate_sonar():
    # measure sonar 90 left, 180 back, 0 forward, 90 right
    # sonar is mounted on motor port D, facing ahead at the start
    BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))
    time.sleep(0.1)
    sonar_readings = {}
    for angle in [0, 90, 180, -90]:
        BP.set_motor_position(BP.PORT_D, angle)
        # wait until motor movement done in blocking
        while True:
            status_d, power_d, enc_d, dps_d = BP.get_motor_status(BP.PORT_D)
            if abs(enc_d - angle) < MOTOR_TOLERANCE:
                BP.set_motor_power(BP.PORT_D, 0)
                break
            time.sleep(0.02)
        time.sleep(0.1)
        sonar_readings[angle] = get_median_sonar(BP.PORT_2)
        print(f"Sonar reading at angle {angle}: {sonar_readings[angle]} cm")
    BP.set_motor_position(BP.PORT_D, 0)
    return sonar_readings


def get_median_sonar(port, num_readings=10):
    readings = []
    while len(readings) < num_readings:
        try:
            val = BP.get_sensor(port)
            if val > 0: # Ignore impossible 0 readings
                readings.append(val)
        except brickpi3.SensorError:
            print("INVALID SONAR")
            pass # Just skip failed attempts
        time.sleep(0.02) # Small delay between pings
    
    readings.sort()
    return readings[len(readings) // 2] + SONAR_OFFSET


for waypoint in waypoints:
    print(f"\n STARTING NEW WAYPOINT to {waypoint}")
    w_x, w_y = waypoint

    dx = w_x - robot_x
    dy = w_y - robot_y
    angle = math.atan2(dy, dx) - robot_a
    angle = math.atan2(math.sin(angle), math.cos(angle))
    print(f"rob_x: {robot_x}, rob_y: {robot_y}, rob_a: {robot_a}, dx: {dx}, dy: {dy}")
    # if angle > math.pi:
    #     angle = math.pi - angle
    distance = math.sqrt(dx*dx + dy*dy)
    print(f"distance: {distance}, angle: {angle}")

    drive_rotate(angle)
    sonar_readings = rotate_sonar()
    print(f"sonar: {sonar_readings}")
    particles.update_rotate(angle, sonar_readings)
    particles.draw()
    time.sleep(1)

    total_dist_to_go = math.sqrt(dx**2 + dy**2)

    while total_dist_to_go > 0.5: # Use a small epsilon instead of 0
        # Determine step size
        last_step = False
        current_step = min(STEP_SIZE, total_dist_to_go)
        if current_step == total_dist_to_go:
            last_step = True
        print(f"Driving step: {current_step:.1f}cm (Remaining: {total_dist_to_go:.1f}cm)")

        # Move the physical robot
        drive_distance(current_step, current_step)
        
        # Sense and Update
        sonar_readings = rotate_sonar()
        print(f"sonar: {sonar_readings}")
        particles.update(current_step, sonar_readings)
        particles.draw()
        time.sleep(1)
        particles.resample()
        particles.draw()
        
        # Get updated position estimate to adjust for the next chunk
        robot_x, robot_y, robot_a = particles.estimate_current_pos()
        
        
        # Recalculate distance to waypoint based on NEW estimated position
        dx = w_x - robot_x
        dy = w_y - robot_y
        total_dist_to_go = math.sqrt(dx**2 + dy**2)
        print(f"Estimated robot position: x: {robot_x:.1f}, y: {robot_y:.1f}, a: {math.degrees(robot_a):.1f} degrees")

        if not last_step and total_dist_to_go > STEP_SIZE / 2:
            # Calculate the required heading to the waypoint
            correction_angle = math.atan2(dy, dx) - robot_a
            print(f"Calculated correction angle (radians): {correction_angle}, (degrees): {math.degrees(correction_angle)}")
            # Find the difference from our current estimated heading
            # Normalize to shortest path (-pi to pi)
            correction_angle = math.atan2(math.sin(correction_angle), math.cos(correction_angle))
            print(f"Normalized correction angle (radians): {correction_angle}, (degrees): {math.degrees(correction_angle)}")

            # Only correct if the error is meaningful (e.g., > 2-3 degrees)
            # Constant adjustments for tiny errors can actually add more noise
            if abs(math.degrees(correction_angle)) > 3.0:
                print(f"Adjusting heading by {math.degrees(correction_angle):.1f} degrees")
                drive_rotate(correction_angle)
                
                # Update particles for the rotation we just did
                sonar_readings = rotate_sonar()
                particles.update_rotate(correction_angle, sonar_readings)
                particles.draw()
                time.sleep(1)
                
                # Update our variables again after the rotation correction
                robot_x, robot_y, robot_a = particles.estimate_current_pos()

        if last_step:
            break

    robot_x, robot_y, robot_a = particles.estimate_current_pos()
    print(f"robot position: {robot_x}, {robot_y}, {robot_a}")

BP.reset_all()