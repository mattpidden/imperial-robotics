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

# cm_per_degree = (360 * 5) / Distance in cm

# def driveDistance(cm):
#     BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
#     BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))

#     target_deg = cm / cm_per_degree
#     BP.set_motor_position(BP.PORT_A, target_deg)
#     BP.set_motor_position(BP.PORT_B, target_deg)

#     while True:
#         status_a, power_a, enc_a, dps_a = BP.get_motor_status(BP.PORT_A)
#         status_b, power_b, enc_b, dps_b = BP.get_motor_status(BP.PORT_B)

#         if status_a == 0 and status_b == 0:
#             break


def main():
    try:
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder D
    except IOError as error:
        print(error)
    
    BP.set_motor_limits(BP.PORT_A, 50, 360)
    BP.set_motor_limits(BP.PORT_B, 50, 360)

    target_degrees = 360 * 5     # 5 revolutions
    tolerance = 5
    
    BP.set_motor_position(BP.PORT_A, target_degrees)
    BP.set_motor_position(BP.PORT_B, target_degrees)

    while True:
        status_a, power_a, enc_a, dps_a = BP.get_motor_status(BP.PORT_A)
        status_b, power_b, enc_b, dps_b = BP.get_motor_status(BP.PORT_B)
        print(f"Motor A | status={status_a} power={power_a}% enc={enc_a:.1f}° dps={dps_a:.1f}")
        print(f"Motor B | status={status_b} power={power_b}% enc={enc_b:.1f}° dps={dps_b:.1f}")

        if status_a == 0 and status_b == 0:
            print("Finished via status = 0 signals")
            break

        if abs(enc_a - target_degrees) < tolerance and abs(enc_b - target_degrees) < tolerance:
           print("Finished via tolerance < 5")
           break

        time.sleep(0.01)

    BP.reset_all()