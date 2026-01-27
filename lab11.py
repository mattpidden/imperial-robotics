#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for running a motor a target speed (specified in Degrees Per Second) set by the encoder of another motor.
# 
# Hardware: Connect EV3 or NXT motors to the BrickPi3 motor ports A and D. Make sure that the BrickPi3 is running on a 9v power supply.
#
# Results:  When you run this program, motor A speed will be controlled by the position of motor D. Manually rotate motor D, and motor A's speed will change.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

tolerance = 5
cm_per_degree = 106 /(360 * 5)
# 110, 97, 104, 103.5, 104, 104
# 18.2 * pi /4 = 14.2942465738

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
    
    for i in range(1):
        for i in range(3):
            driveDistance(40, 40)
            time.sleep(0.5)
            driveDistance(13.1, -13.1)
            time.sleep(0.5)
        driveDistance(40, 40)
        

    BP.reset_all()
    
main()