from __future__ import print_function 
from __future__ import division                               
import time    
import brickpi3 
import random
import math

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.


tolerance = 5
# with pen 107, without 106
cm_per_degree = 107 /(360 * 5)
# 110, 97, 104, 103.5, 104, 104
# 18.2 * pi /4 = 14.2942465738
NUM_PARTICLES = 100
MU = 0
SIGMA = 0.5
SIGMA_ROTATE = 0.5
SIGMA_ROTATE_ONLY = 0.5

SCALE = 10
OFFSET = 100

class Particle:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.a = 0
        self.w = 1 / NUM_PARTICLES
        
    def update_position(self, dx, dy, da):
        self.x += dx
        self.y += dy
        self.a += da
        

    def update_weights(self, new_w):
        self.w = new_w
        
    def get_particle(self):
        return self.x, self.y, self.a

        
def draw_line(x0, y0, x1, y1):
    
    line = (x0*SCALE + OFFSET, y0*SCALE + OFFSET, x1*SCALE + OFFSET, y1*SCALE + OFFSET)
    print("drawLine:" + str(line))
    
    
def draw_particles(particles):
    points = []
    for particle in particles:
        x, y, a = particle.get_particle()
        point = (x*SCALE + OFFSET, y*SCALE + OFFSET, a)
        points.append(point)
    
    print("drawParticles:" + str(points))
    


def driveDistance(left_cm, right_cm):
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))

    left_target_degrees = left_cm / cm_per_degree
    right_target_degrees = right_cm / cm_per_degree
    BP.set_motor_position(BP.PORT_A, left_target_degrees)
    BP.set_motor_position(BP.PORT_B, right_target_degrees)


    done_a = False
    done_b = False

    while True:
        status_a, power_a, enc_a, dps_a = BP.get_motor_status(BP.PORT_A)
        status_b, power_b, enc_b, dps_b = BP.get_motor_status(BP.PORT_B)
        
        if (not done_a) and abs(enc_a - left_target_degrees) < tolerance:
            BP.set_motor_power(BP.PORT_A, 0)
            done_a = True
        if (not done_b) and abs(enc_b - right_target_degrees) < tolerance:
            BP.set_motor_power(BP.PORT_B, 0)
            done_b = True
            
        if done_a and done_b:
            print("Finished via tolerance")
            break
            
        time.sleep(0.05)


        

def main():
    BP.set_motor_limits(BP.PORT_A, 25, 180)
    BP.set_motor_limits(BP.PORT_B, 25, 180)
    
    draw_line(0,0,40,0)
    draw_line(40,0,40,40)
    draw_line(40,40,0,40)
    draw_line(0,40,0,0)
    
    #global robot_x
    robot_x = 0
    #global robot_y
    robot_y = 0
    robot_a = 0
    
    #global robot_x
    robot_x = 0
    #global robot_y
    robot_y = 0
    robot_a = 0
    
    particle_set = []
    for i in range(NUM_PARTICLES):
        particle = Particle()
        particle_set.append(particle)
    draw_particles(particle_set)
        
        
    def update_particle_set_distance(distance):
        for particle in particle_set:
            _, _, angle = particle.get_particle()
            dx = (distance + random.gauss(MU, SIGMA)) * math.cos(math.radians(angle))
            dy = (distance + random.gauss(MU, SIGMA)) * math.sin(math.radians(angle))
            dx = (distance + random.gauss(MU, SIGMA)) * math.cos(math.radians(angle))
            dy = (distance + random.gauss(MU, SIGMA)) * math.sin(math.radians(angle))
            da = random.gauss(MU, SIGMA_ROTATE)
            particle.update_position(dx, dy, da)
        draw_particles(particle_set)
        
    def update_particle_set_rotation(angle_a):
        for particle in particle_set:
            da = angle_a + random.gauss(MU, SIGMA_ROTATE_ONLY)
            particle.update_position(0, 0, da)
        draw_particles(particle_set)
    
    def drive_and_set_particle_square(iterations=4, distance=10):
        for i in range(iterations):
            driveDistance(distance, distance)
            update_particle_set_distance(distance)
            time.sleep(2)
        rotate_by(-90)
        robot_x, robot_y, robot_a = estimate_current_pos()
        
    def drive_and_set_particle(distance=10, angle=90):
        rotate_by(angle)
        driveDistance(distance, distance)
        update_particle_set_distance(distance)
        time.sleep(2)
        nonlocal robot_x, robot_y, robot_a
        robot_x, robot_y, robot_a = estimate_current_pos()
    
    def rotate_by(angle):
        CM_PER_DEG = 14.1/90
        calc_distance = CM_PER_DEG*angle
        driveDistance(calc_distance, -calc_distance)
        update_particle_set_rotation(angle)
        time.sleep(2)
    
    def estimate_current_pos():
        mean_x = 0
        mean_y = 0
        mean_a = 0
        for particle in particle_set:
            mean_x += particle.x*particle.w
            mean_y += particle.y*particle.w
            mean_a += particle.a*particle.w
        
        print("mean_x: ",mean_x)
        print("mean_y: ",mean_y)
        print("mean_y: ",mean_a)
        return mean_x, mean_y, mean_a
    
    def navigate_to_waypoint(w_x, w_y):
        dx = w_x - robot_x
        dy = w_y - robot_y
        
        print("rob_x: ",robot_x)
        print("rob_y: ",robot_y)
        print("dx: ",dx)
        print("dy: ",dy)
        angle = math.degrees(math.atan2(dy, dx)) - robot_a
        
        magnitude = math.sqrt(dx*dx + dy*dy)
        print("magnitude: ",magnitude)
        print("angle: ",angle)
        drive_and_set_particle(magnitude, angle)
        
    #Square     
    #drive_and_set_particle_square()
    #drive_and_set_particle_square()
    #drive_and_set_particle_square()
    #drive_and_set_particle_square()
        
    #navigate_to_waypoint(30, 30)
    #navigate_to_waypoint(23, 5)
    #navigate_to_waypoint(0, 0)
    
    navigate_to_waypoint(50, 50)
    navigate_to_waypoint(40, 0)
    navigate_to_waypoint(20, 40)
    navigate_to_waypoint(0, 0)
    
    
    #navigate_to_waypoint(5, 0)
    #navigate_to_waypoint(-5, 0)
    #navigate_to_waypoint(0, 40)
    #navigate_to_waypoint(0, 40)
    #navigate_to_waypoint(40, 40)
    #navigate_to_waypoint(20, -10)
    #navigate_to_waypoint(0, 0)
    
    BP.reset_all()
    
main()
