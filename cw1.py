from __future__ import print_function 
from __future__ import division                               
import time    
import brickpi3 

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

# run3: x = 0.15, y = 0.2
# run4: x = -0.1, y = -0.4
# run5: x = 0.6,  y = -0.7
# run6: x = 0.2,  y = -0.2
# run7: x = 0.3,  y = 1.2
# run8: x = 2.2,  y = 0.85
# run9: x = 0.9,  y = 0.1
# run10:x = -0.3, y = 0.6
# run11:x = 0.7,  y = -1.2
# run12:x = -0.9, y = 0.3

# mean: x= 0.375, y = 0.075
# x covariance = 0.615625
# y covariance = 0.473625
# xy covariance= 0.035865

# [ 0.616  0.036 ]
# [ 0.036  0.474 ]



tolerance = 5
# with pen 107, without 106
cm_per_degree = 107 /(360 * 5)
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
            # with pen 12.9, without 13.1
            driveDistance(12.9, -12.9)
            time.sleep(0.5)
        driveDistance(40, 40)
        

    BP.reset_all()
    
main()