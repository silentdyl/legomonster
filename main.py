from hub import light_matrix
import runloop
import time
#from . import types
from hub import light_matrix, motion_sensor, port
import motor
#from types import MemberDescriptorType
#from typing import Awaitable
"""robot_moves.py

Helper functions for driving the robot using the hub motion_sensor and
motor_pair helpers. These functions are written as async coroutines to be
run by the Spike runloop. This file focuses on higher-level motions:
- gyro-based turning
- gyro-straight driving with simple proportional correction
- backing up using the gyro

Notes:
- Many motion_sensor APIs return values in tenths of degrees; the code
multiplies by 0.1 to convert to degrees before using them.
"""

from hub import light_matrix, motion_sensor, port
import motor
import math
import runloop
import motor_pair
import sys
from motor import ERROR, SMART_BRAKE
import color_sensor

# Ports and speed constants — change to match your robot's wiring
hook = port.E
hammer = port.F
supaslow = 30
ultraslow_speed = 200
slow_speed = 500
medium_speed = 750
high_speed = 1110
silly_speed = 2500
slowing_down = 360# approx degrees at which to slow down (one rotation)
backward_high_speed = -1000
gyro_error_rate = 1.00# empirical multiplier to compensate gyro bias
turn_error_rate = 0.92# tolerance used for finishing turns (degrees)


def degrees_for_distance(distance_cm):
    """Convert a linear distance in cm to an approximate wheel degree rotation.

    The conversion factor (16.014 cm per wheel revolution) is project-specific
    and may need adjustment for your wheel size/gear ratio.
    """
    return int((distance_cm / 18.5) * 360)


async def robot_setup():
    """Perform basic setup: pair left/right drive motors.

    This function should be awaited during initialization so that the
    motor_pair is configured before driving functions run.
    """
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)


async def gyro_reset():
    """Reset the yaw reading to zero and wait until the sensor stabilizes.

    Many hub motion sensors take a few cycles to settle after a reset.
    This routine actively waits (with small sleeps) until tilt_angles[0]
    reads zero.
    """
    motion_sensor.reset_yaw(0)
    while motion_sensor.tilt_angles()[0] != 0:
        motion_sensor.reset_yaw(0)
        await runloop.sleep_ms(1)


async def gyro_turn_right(degrees, speed):
    """Turn the robot to the right by approximately `degrees`.

    Parameters:
    - degrees: positive number of degrees to turn right
    - speed: nominal speed to apply while turning (may be reduced near target)

    Implementation notes:
    - The hub's yaw reading is scaled (the code uses *0.1 to convert from
    tenths of degrees to degrees). Adjust scaling if your firmware differs.
    - The routine reduces speed when the remaining gap is below thresholds
    to avoid overshoot.
    """
    # Convert to hub sign convention and apply empirical error multiplier
    degrees = -degrees * gyro_error_rate
    await gyro_reset()
    # motion_sensor.tilt_angles()[0] is often in tenths of degrees, so *0.1 => degrees
    print(motion_sensor.tilt_angles()[0] * 0.1)

    # Keep turning until we reach the desired yaw
    while (motion_sensor.tilt_angles()[0] * 0.1) > degrees:
        gap = degrees - (motion_sensor.tilt_angles()[0] * 0.1)
        # Reduce speed as we near the target to limit overshoot
        if gap > -35:
            speed = 40
        if gap > -10:
            speed = 20
        motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)

    # Stop and hold briefly so the robot stabilizes
    print("stop motors")
    motor_pair.stop(motor_pair.PAIR_1, stop=SMART_BRAKE)
    await runloop.sleep_ms(500)
    print(motion_sensor.tilt_angles()[0] * 0.1)


async def gyro_straight(distance_cm, speed):
    """Drive forward approximately `distance_cm`, maintaining heading.

    The function converts the requested linear distance into wheel degrees,
    then uses a very simple proportional correction based on the yaw error
    (motion_sensor.tilt_angles()[0]).
    """
    await gyro_reset()
    print("start gyro_straight")
    print(motion_sensor.tilt_angles()[0] * 0.1)

    dist_deg = degrees_for_distance(distance_cm)
    # Reset the encoder for the left/right motor used to measure distance
    motor.reset_relative_position(port.B, 0)

    # Drive until the encoder reports we've traveled the expected degrees
    while math.fabs(motor.relative_position(port.B)) < dist_deg:
        dist_remaining = dist_deg - math.fabs(motor.relative_position(port.B))
        # Slow as we approach the target distance
        if dist_remaining < slowing_down:
            speed = slow_speed

        # motion_sensor.tilt_angles()[0] is often tenths of degrees; *-0.1 = signed degree error
        err = motion_sensor.tilt_angles()[0] * -0.1
        # correction is proportional to error (empirically chosen gain)
        correction = int(err * -2)
        # Apply steering correction while driving forward
        motor_pair.move(motor_pair.PAIR_1, correction, velocity=speed)

    motor_pair.stop(motor_pair.PAIR_1, stop=SMART_BRAKE)


async def gyro_turn_left(degrees, speed):
    """Turn the robot left by approximately `degrees`."""
    await gyro_reset()
    degrees = degrees * gyro_error_rate
    print(motion_sensor.tilt_angles()[0] * 0.1)
    print("before")
    print(degrees)
    print(motion_sensor.tilt_angles()[0] * 0.1)

    while (motion_sensor.tilt_angles()[0] * 0.1) < degrees:
        gap = degrees - (motion_sensor.tilt_angles()[0] * 0.1)
        if gap < 30:
            speed = 40
        if gap < 7:
            speed = 20
        motor_pair.move_tank(motor_pair.PAIR_1, -speed, speed)

    motor_pair.stop(motor_pair.PAIR_1, stop=SMART_BRAKE)


async def gyro_turn(degrees, speed):
    """Turn to an absolute relative heading using the gyro with small
    settling checks before finishing.

    The function applies short movement bursts while checking whether the
    robot is within a small angular tolerance. It performs up to a few
    confirmation cycles (turnchecks) to ensure the robot has stabilized.
    """
    await gyro_reset()
    degrees = degrees * gyro_error_rate
    print(motion_sensor.tilt_angles()[0] * 0.1)
    print("before")
    print(degrees)
    print(motion_sensor.tilt_angles()[0] * 0.1)

    turnchecks = 1
    gap = degrees + (motion_sensor.tilt_angles()[0] * 0.1)
    agap = math.fabs(gap)

    # Wait up to a few cycles to confirm we are within tolerance
    while turnchecks < 3:
        if agap < 30:
            speed = 40
        if agap < 10:
            speed = 20

        if (gap < 0) and (gap < -turn_error_rate):
            # Need to rotate one way
            motor_pair.move_tank(motor_pair.PAIR_1, -speed, speed)
        if (gap > 0) and (gap > turn_error_rate):
            # Need to rotate the other way
            motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)

        # If we're within the turn error tolerance, stop and count a check
        if (gap >= -turn_error_rate) and (gap <= turn_error_rate):
            motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
            speed = 8
            turnchecks = turnchecks + 1
            await runloop.sleep_ms(50)

        # Recompute gap/gap magnitude for next cycle
        gap = degrees + (motion_sensor.tilt_angles()[0] * 0.1)
        agap = math.fabs(gap)

    # Final stop and short delay to stabilize
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
    print("stop motors")
    await runloop.sleep_ms(500)
    print(motion_sensor.tilt_angles()[0] * 0.1)


async def gyro_back(distance_cm, speed):
    """Back the robot up while maintaining heading using gyro correction.

    Note: original code compares encoder absolute position to the provided
    `distance_cm` value directly. If `distance_cm` is in centimeters and the
    encoder reports degrees, convert before comparing. This function leaves
    the original behavior but flags that mismatch as a possible bug.
    """
    print("starting gyro_backup")
    await gyro_reset()
    motor.reset_relative_position(port.B, 0)# reset encoder
    dist_deg = degrees_for_distance(distance_cm) #converting number we input into paremeter distance cm to degrees using our prebuilt function and we name this a constant called dist_deg

    print("gyro back loop starting")

    # WARNING: distance_cm may be in cm; original code compares it to
    # encoder degrees. If you expect to pass cm, convert with
    # degrees_for_distance(distance_cm) before comparing.
    while math.fabs(motor.relative_position(port.B)) < dist_deg:
        # compute the error in degrees (negative sign to correct heading)
        err = motion_sensor.tilt_angles()[0] * -0.1
        # simple proportional correction (empirical gain)
        correction = int(err * 2)
        # steering to correct error while backing up
        motor_pair.move(motor_pair.PAIR_1, correction, velocity=-speed)

    motor_pair.stop(motor_pair.PAIR_1)


async def emergency_stop():
## """Immediately stop all motors and indicate safe state."""
    try:
        motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
    except Exception:
        pass
    try:
        motor_pair.stop(motor_pair.PAIR_2)
        motor_pair.stop(motor_pair.PAIR_3)
    except Exception:
        pass
    # optionally light hub or log
    try:
        light_matrix.write('X')# or set a pattern if available
    except Exception:
        pass



#Currently set to Block 3
async def MissionRedBlue():
    # Perform the first mission Run
    # from Red Side (left side) - We need to figure out positioning
        await robot_setup()
        await motor.run_for_degrees(hammer,-90,high_speed) #putting the hammer up so we know where we are starting
    # await gyro_straight(60,high_speed)
        await gyro_straight(56,medium_speed)#Why do we go straight and then straight again
        await motor.run_for_degrees(hammer,74,high_speed)
        await gyro_turn_left(130,medium_speed)
        #await gyro_straight(27,medium_speed)
        await gyro_turn_right(130,slow_speed)
        #await motor.run_for_degrees(hook,60,high_speed)
        await motor.run_for_degrees(hammer,-90,high_speed)
        await gyro_turn_right(65,medium_speed)
        await motor.run_for_degrees(hammer,130,high_speed)
        await gyro_straight(34,high_speed)
        await gyro_turn_left(20,ultraslow_speed)
        await motor.run_for_degrees(hammer,-265,supaslow)
        #await motor.run_for_degrees(hammer,-20,supaslow)
        await motor.run_for_degrees(hook,3,slow_speed)
        await motor.run_for_degrees(hammer,285,medium_speed)
        await gyro_turn_left(-30,medium_speed)
        await gyro_back(36,high_speed)
        await gyro_turn_left(45,medium_speed)
        await gyro_back(57,medium_speed) 

        # next mission




        #Next mission - Hook mission starting at first red
        #await gyro_straight(16,medium_speed)
        #await gyro_turn_right(90,slow_speed)

        #await gyro straight(19,medium_speed)

        #await gyro_turn_left(45,slow_speed)

        #await gyro_back(12,medium_speed)

        #await gyro_turn_right(90,slow_speed)

        #await motor.run_for_degrees(hook,-75,high_speed)

        #await gyro_straight(19,slow_speed)



        #next mission
        #await gyro_turn_right(20,medium_speed)

        #await motor.run_for_degrees(hook,90,high_speed)

        #await motor.run_for_degrees(hammer,90,medium_speed)

        #await gyro_straight(5,medium_speed)

        #await motor.run_for_degrees(hammer,110,medium_speed)

        #await gyro_straight(10,medium_speed)



async def MissionBlueStart():
    # Perform the 1st mission from Blue Side (right side)
    # This one will start without starting with doing the hammer
    # upload to 6
    await robot_setup()

    await motor.run_for_degrees(hammer,-90,high_speed) #putting the hammer up so we know where we are starting
    await gyro_straight(37.5,high_speed)


    await gyro_back(10,medium_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_straight(18,medium_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_straight(23,high_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_straight(4,ultraslow_speed)
    await motor.run_for_degrees(hammer,120,medium_speed)
    await gyro_back(1,medium_speed)
    await motor.run_for_degrees(hammer,-90,medium_speed)
    ### above Tested and good
    await gyro_back(8,slow_speed)
    await gyro_turn_left(40,medium_speed)
    await gyro_straight(12,ultraslow_speed)
    await motor.run_for_degrees(hammer,50,medium_speed)
    await gyro_turn_left(25,medium_speed)
    await gyro_back(3,slow_speed)


    await gyro_turn_left(50,medium_speed)
    await motor.run_for_degrees(hammer,35,slow_speed)
    await gyro_straight(40,medium_speed)
    await gyro_turn_right(45,slow_speed)
    await gyro_straight(5,ultraslow_speed)
    await gyro_turn_left(45,slow_speed)
    await gyro_straight(30,medium_speed)
    await motor.run_for_degrees(hammer,90,slow_speed)
    await gyro_back(5,medium_speed)
    await gyro_turn_left(45,slow_speed)
    await gyro_straight(10,medium_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_straight(5,slow_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_straight(60,medium_speed)

    await gyro_back(5,slow_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_back(2,ultraslow_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_back(20.5,medium_speed)
    await motor.run_for_degrees(hammer,-45,high_speed)
    await gyro_straight(10,medium_speed)
    await motor.run_for_degrees(hammer,90,medium_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_back(52,high_speed)
    await gyro_turn_left(-90,medium_speed)
    await gyro_back(75,high_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_back(15,high_speed)
    await emergency_stop()
...



#Currently Set to Block 2

async def MissionThreeBlue():
    # Perform the 1st mission from Blue Side (right side)
    # We need to figure out positioning
    await robot_setup()
#Mission one
    await motor.run_for_degrees(hammer,-90,high_speed) #putting the hammer up so we know where we are starting
    await gyro_straight(37.5,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)


    #try:
        #for i in range(10):#for go forward 5cm, go back 5cm 10x
        # await gyro_back(5,medium_speed)
        # await gyro_straight(5,medium_speed)
# except Exception:
        #pass
    await gyro_back(10,medium_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_straight(18,medium_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_straight(23,high_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_straight(2,ultraslow_speed)
    await motor.run_for_degrees(hammer,120,medium_speed)
    await gyro_back(1,medium_speed)
    await motor.run_for_degrees(hammer,-90,medium_speed)
    ### above Tested and good
    await gyro_back(8,slow_speed)
    await gyro_turn_left(40,medium_speed)
    await gyro_straight(11.5,ultraslow_speed)
    await motor.run_for_degrees(hammer,50,medium_speed)
    await gyro_turn_left(25,medium_speed)
    await gyro_back(3,slow_speed)
    await gyro_turn_left(50,medium_speed)
    await motor.run_for_degrees(hammer,35,slow_speed)
    await gyro_straight(40,medium_speed)
    await gyro_turn_right(45,slow_speed)
    await gyro_straight(5,ultraslow_speed)
    await gyro_turn_left(45,slow_speed)
    await gyro_straight(30,medium_speed)
    await motor.run_for_degrees(hammer,90,slow_speed)
    await gyro_back(5,medium_speed)
    await gyro_turn_left(45,slow_speed)
    await gyro_straight(10,medium_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_straight(5,slow_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_straight(60,medium_speed)

    await gyro_back(5,slow_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_back(2,ultraslow_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_back(20.5,medium_speed)
    await motor.run_for_degrees(hammer,-45,high_speed)
    await gyro_straight(10,medium_speed)
    await motor.run_for_degrees(hammer,90,medium_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_back(52,high_speed)
    await gyro_turn_left(-90,medium_speed)
    await gyro_back(75,high_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_back(15,high_speed)
    await emergency_stop()

async def Thor():
    await robot_setup()

    await motor.run_for_degrees(hammer,-90,high_speed) #putting the hammer up so we know where we are starting
    await gyro_straight(37.5,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)
    await motor.run_for_degrees(hammer,100,high_speed)
    await motor.run_for_degrees(hammer,-100,high_speed)
    await gyro_back(37.5,high_speed) #goes to home


async def Multimash():
    await robot_setup()
    await gyro_straight(10,slow_speed)
    await gyro_turn_left(45,slow_speed)
    await gyro_straight(34,slow_speed)
    await motor.run_for_degrees(hammer,15,slow_speed)
    await gyro_turn_left(85,medium_speed)
    await gyro_straight(23,slow_speed)
    await motor.run_for_degrees(hammer,-60,slow_speed)
    await gyro_turn_right(90,slow_speed)
    await gyro_straight(15,slow_speed)
    await gyro_turn_left(10,slow_speed)
    await gyro_back(57,slow_speed)

async def ThorPart2():
    await robot_setup()
    await motor.run_for_degrees(hammer,-50,medium_speed)
    await gyro_straight(37.5,high_speed)
    await gyro_back(10,medium_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_straight(18,medium_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_straight(23,high_speed)
    await gyro_turn_right(45,medium_speed)
    await gyro_straight(2,ultraslow_speed)
    await motor.run_for_degrees(hammer,120,medium_speed)
    await gyro_back(1,medium_speed)
    await motor.run_for_degrees(hammer,-90,medium_speed)
    await gyro_back(8,slow_speed)
    await gyro_turn_left(40,medium_speed)
    await gyro_straight(11.5,ultraslow_speed)
    await motor.run_for_degrees(hammer,50,medium_speed)
    await gyro_turn_left(25,medium_speed)
    await gyro_straight(3,slow_speed)
    await gyro_turn_left(50,medium_speed)
    await motor.run_for_degrees(hammer,35,slow_speed)
    #evelynn start
    await gyro_turn_left (20, medium_speed)
    await gyro_straight (36, medium_speed)
    await motor.run_for_degrees(hook,-20, medium_speed)
    await gyro_turn_left(90,medium_speed)
    await gyro_straight(2,medium_speed)
    await motor.run_for_degrees(hook,20,medium_speed)
    await gyro_back(10,medium_speed)
    await gyro_turn_right(90,medium_speed)
    await gyro_straight(41,medium_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_straight(17,medium_speed)
    await gyro_turn_left(45,medium_speed)
    await gyro_straight(10,medium_speed)
    await motor.run_for_degrees(hammer,-20,medium_speed)
    await gyro_back(10,medium_speed)
    await gyro_turn_right(90,medium_speed)
    await gyro_straight(30,medium_speed)
    await gyro_turn_left(80,medium_speed)
    await gyro_straight(90,medium_speed)
    await emergency_stop()

async def Turn():
    await robot_setup()
    await gyro_turn_left(90,slow_speed)

async def hulksmash():
    # Perform the first mission
        await robot_setup()
        await gyro_straight(69,high_speed)
        await gyro_back(70,high_speed)

async def slapathing():
        await robot_setup()
        await motor.run_for_degrees(hammer,-90,medium_speed)
        await gyro_straight(47,medium_speed)
        await gyro_turn_left(90,slow_speed)
        await gyro_straight(20,medium_speed)
        await gyro_turn_right(90,medium_speed)
        await gyro_straight(37,medium_speed)
        await gyro_turn_right(85,medium_speed)
        await motor.run_for_degrees(hammer,88,medium_speed)
        await gyro_straight(7,medium_speed)
        await gyro_turn_left(35,silly_speed)
        await gyro_turn_right(35,silly_speed)
        await gyro_turn_left(35,silly_speed)
        await gyro_turn_right(35,silly_speed)
        await gyro_turn_left(35,silly_speed)
        await gyro_turn_right(35,silly_speed)
        await gyro_turn_left(35,silly_speed)
        await gyro_turn_right(35,silly_speed)
        await gyro_back(5,medium_speed)
        await gyro_turn_left(5,medium_speed)
        await motor.run_for_degrees(hammer,-80,medium_speed)
        
        #await gyro_turn_right(6,medium_speed)
        await gyro_back(50,medium_speed)
        await gyro_turn_left(60,medium_speed)
        await motor.run_for_degrees(hammer,145,medium_speed)
        await gyro_straight(30,silly_speed)
        await gyro_back(25,medium_speed)
        await motor.run_for_degrees(hammer,-120,medium_speed)
        await gyro_turn_left(20,medium_speed)
        await gyro_straight(40,medium_speed)
        await gyro_turn_right(40,medium_speed)
        await gyro_straight(70,medium_speed)

        

async def MissionRedsideBlue():
    # Perform the first mission
        await robot_setup()
        await gyro_straight(185,high_speed)

#Function is to test that the hammer is working
async def testhammer():

 await motor.run_for_degrees(port.F, -90, 500)


 async def MissionIronMan():
    await robot_setup()
    await gyro_straight(67,high_speed)

async def grindstone():
    await robot_setup()
    await gyro_straight(5,slow_speed)
    await gyro_turn_left(135,slow_speed)
    await gyro_back(32,slow_speed)
    await gyro_turn_left(38,slow_speed)
    await gyro_back(45,slow_speed)
    await motor.run_for_degrees(hook,60,supaslow)
    await gyro_straight(4.5,supaslow)
    await motor.run_for_degrees(hook,-65,supaslow)
    await gyro_turn_left(5,supaslow)
    await gyro_straight(38,slow_speed)
    await gyro_turn_right(45,slow_speed)
    await gyro_straight(32,slow_speed)
    

#def main():
    #Initialize the robot

async def main():
    try:
        await ThorPart2()
    finally:
        # Ensure motors are stopped even if something fails
        try:
            await emergency_stop() # stop all motors
        except Exception:
            pass
# 6 - MissionBlueStart() - New mission for BLue
# 4 - Across the board MissionRedsideBlue()
# 2 - Blue side - MissionThreeBlue()
# 3 - Red side - Missionredblue()
# Start the sequence
runloop.run(main())


#saturday
#mission 7
#Mission 9
