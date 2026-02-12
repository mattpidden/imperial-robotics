#!/usr/bin/env python 

# Some suitable functions and data structures for drawing a map and particles
import sys
import time
import random
import math

SIGMA = 3
K = 1
num_min_walls_none = 0

class Particle:
    def __init__(self, n_particles):
        self.x = 0
        self.y = 0
        self.a = 0
        self.w = 1 / n_particles
        
    def update(self, x, y, a, w):
        self.x = x
        self.y = y
        self.a = a
        self.w = w
        
    def get_particle(self):
        return self.x, self.y, self.a
    
    def update_weight(self, w):
        self.w = w

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)) # in cm

def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)) # in cm

def calcW():
    return random.random()

def calcTheta():
    return random.randint(0,360)

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
        display = [(self.__screenX(d.x),self.__screenY(d.y)) + (d.a, d.w) for d in data]
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

# Simple Particles set
class Particles:
    def __init__(self):
        self.n = 10    
        self.data = []
        
        for i in range(self.n):
            particle = Particle(self.n)
            self.data.append(particle)

    def update(self):
        likelihood_sum = 0
        
        
        for particle in self.data:
            x, y, a = particle.get_particle()
            likelihood = calculate_likelihood(x, y, a)
            if not likelihood:
                global num_min_walls_none
                num_min_walls_none += 1
                print(f"Number of None walls: {num_min_walls_none}")
            particle.update(calcX(), calcY(), calcTheta(), likelihood)
            likelihood_sum += likelihood
        
        for particle in self.data:
            normalized_weight = particle.w / likelihood_sum
            particle.update_weight(normalized_weight)
        
        
        #self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)]
    
    def draw(self):
        canvas.drawParticles(self.data)
        
def calculate_likelihood(x, y, theta, z=0):
    min_dist = sys.float_info.max
    min_wall = None
    
    for wall in mymap.walls:
        ax, ay, bx, by = wall
        
        # Maybe add a tiny epsilon for the denominator to avoid divide by zero
        m_numerator = (by - ay)*(ax - x) - (bx-ax)*(ay - y)
        m_denominator = (by - ay)*math.cos(theta) - (bx - ax)*math.sin(theta)
        
        if m_denominator == 0:
            continue
        
        m = m_numerator/m_denominator
    
        intersect_x = x + m*math.cos(theta)
        intersect_y = y + m*math.sin(theta)
        
        in_horizontal_range = intersect_x >= min(ax,bx) and intersect_x <= max(ax,bx)
        in_vertical_range = intersect_y >= min(ay,by) and intersect_y <= max(ay,by)
        
        if in_horizontal_range and in_vertical_range:
            if m < min_dist:
                min_dist = m
                min_wall = wall
                    
    if min_wall is not None:
        likelihood = math.exp( ((-(z-min_dist)**2) / (2*SIGMA**2)) + K )
        return likelihood
    else:
        return 0
        
        
    

canvas = Canvas()    # global canvas we are going to draw on

mymap = Map()
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
# ax, ay, bx, by
mymap.add_wall((0,0,0,168))        # a
mymap.add_wall((0,168,84,168))     # b
mymap.add_wall((84,126,84,210))    # c
mymap.add_wall((84,210,168,210))   # d
mymap.add_wall((168,210,168,84))   # e
mymap.add_wall((168,84,210,84))    # f
mymap.add_wall((210,84,210,0))     # g
mymap.add_wall((210,0,0,0))        # h
mymap.draw()

particles = Particles()

t = 0
while True:
    particles.update()
    particles.draw()
    t += 0.05
    time.sleep(0.05)
