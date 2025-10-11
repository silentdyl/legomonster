from hub import light_matrix, motion_sensor
from motor import ERROR, SMART_BRAKE
import runloop, motor_pair, sys
from hub import port
import motor_pair
import orientation
import math
import motor
import color_sensor



gyro_error_rate=.93
slowingdown = 360
slow_speed = 400


def degreesfordistance(distance_cm):
    return int (((distance_cm/16.014)*360))


async def gyro_reset ():
    motion_sensor.reset_yaw(0)
    while (motion_sensor.tilt_angles() [0]!=0):
        motion_sensor.reset_yaw(0)
        runloop.sleep_ms(1)


async def gyro_turn_right(degrees,speed):
    degrees=-degrees*gyro_error_rate
    await gyro_reset()
    print(motion_sensor.tilt_angles()[0]*0.1)
    while (motion_sensor.tilt_angles()[0]*0.1)>degrees:
        gap=degrees-(motion_sensor.tilt_angles()[0]*0.1)
        if (gap>-35):
            speed = 40
        if(gap>-10):
            speed = 20
        motor_pair.move_tank(motor_pair.PAIR_1,speed, -speed)

        print("stop motors")
        motor_pair.stop(motor_pair.PAIR_1, stop=SMART_BRAKE)
        await runloop.sleep_ms (500)
        print(motion_sensor.tilt_angles()[0]*0.1)






async def gyro_straight(distance_cm, speed):
    #below is resetting yaw angle and waiting for it to become stable
    await gyro_reset()
    print("3")
    print(motion_sensor.tilt_angles()[0]*0.1)


    distancedegrees = degreesfordistance(distance_cm)
    motor.reset_relative_position(port.B,0) #reset reletive degrees counted to 0
    while math.fabs(motor.relative_position(port.B))< distancedegrees:


        distanceremaining = distancedegrees - math.fabs(motor.relative_position(port.B))
        if (distanceremaining < slowingdown):
                            speed=slow_speed

    ERROR = motion_sensor.tilt_angles()[0]* -0.1
    correction = (ERROR * -2)

    motor_pair.move(motor_pair.PAIR_1, int(correction),
velocity=speed)
    motor_pair.stop(motor_pair.PAIR_1,stop=SMART_BRAKE)

async def gyro_turn_left(degrees, speed):
     #you know what this does
    await gyro_reset()
    degrees=degrees*gyro_error_rate
    print(motion_sensor.tilt_angles()[0]*0.1)
    #this makes the turn stop
    print("before")
    print(degrees)
    print(motion_sensor.tilt_angles()[0]*0.1)
    while (motion_sensor.tilt_angles()[0]*0.1)<degrees:
        #makes it turn
        motor_pair.move_tank(motor_pair.PAIR_1,-speed, speed)
    motor_pair.stop(motor_pair.PAIR_1,stop=SMART_BRAKE)



async def main():
    
    await motor_pair.pair(motor_pair.PAIR_1,port.A,port.B)
    await gyro_straight (45,280)

    '''
    motor_pair.move(motor_pair.PAIR_1,0,velocity=1000)
    await runloop.sleep_ms(2000) #for every 1000 it is 1 second
    motor_pair.stop(motor_pair.PAIR_1)
    '''

    #await motor_pair.move_for_time(motor_pair.PAIR_1,2000,100,velocity=1000,stop=motor_pair.break,acceleration=500)
    #await motor_pair.move_for_time(motor_pair.PAIR_1,740,0,velocity=1000,)#in move for time every 1000 = 1 second

    motion_sensor.tilt_angles()[0]*0.1
    

runloop.run(main())