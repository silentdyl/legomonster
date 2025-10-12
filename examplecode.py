from hub import light_matrix, motion_sensor, port, motion_sensor
import motor
import math
import runloop, motor_pair, sys
from motor import ERROR, SMART_BRAKE
import color_sensor
#from typing import Awaitable

left_arm=port.A
right_arm=port.F
ultraslowspeed=200
slow_speed=400
medium_speed=750
high_speed=1000
from hub import light_matrix, motion_sensor, port
import motor
import math
import runloop
import motor_pair
import sys
from motor import ERROR, SMART_BRAKE
import color_sensor

# Ports and constants
left_arm = port.A
right_arm = port.F
ultraslow_speed = 200
slow_speed = 400
medium_speed = 750
high_speed = 1000
slowing_down = 360  # 360 is one wheel rotation
backward_high_speed = -1000
gyro_error_rate = 1.00
turn_error_rate = 0.5


def degrees_for_distance(distance_cm):
    """Convert a linear distance in cm to wheel degrees (approx)."""
    return int((distance_cm / 16.014) * 360)


async def robot_setup():
    """Perform any robot setup (pair motors, etc.)."""
    motor_pair.pair(motor_pair.PAIR_1, port.B, port.D)


async def gyro_reset():
    motion_sensor.reset_yaw(0)
    # Wait until the yaw reads zero (or as close as possible)
    while motion_sensor.tilt_angles()[0] != 0:
        motion_sensor.reset_yaw(0)
        await runloop.sleep_ms(1)


async def gyro_turn_right(degrees, speed):
    """Turn right by degrees using the gyro."""
    # Make degrees negative because of hub coordinate sign
    degrees = -degrees * gyro_error_rate
    await gyro_reset()
    print(motion_sensor.tilt_angles()[0] * 0.1)
    # Keep turning until we pass the target
    while (motion_sensor.tilt_angles()[0] * 0.1) > degrees:
        gap = degrees - (motion_sensor.tilt_angles()[0] * 0.1)
        if gap > -35:
            speed = 40  # slow down when we get closer
        if gap > -10:
            speed = 20
        motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
    # Stop the motors
    print("stop motors")
    motor_pair.stop(motor_pair.PAIR_1, stop=SMART_BRAKE)
    await runloop.sleep_ms(500)
    print(motion_sensor.tilt_angles()[0] * 0.1)


async def gyro_straight(distance_cm, speed):
    await gyro_reset()
    print("start gyro_straight")
    print(motion_sensor.tilt_angles()[0] * 0.1)
    distancedegrees = degrees_for_distance(distance_cm)
    motor.reset_relative_position(port.B, 0)  # reset relative degrees
    while math.fabs(motor.relative_position(port.B)) < distancedegrees:
        distanceremaining = distancedegrees - math.fabs(motor.relative_position(port.B))
        if distanceremaining < slowing_down:
            speed = slow_speed
        # compute the error in degrees
        err = motion_sensor.tilt_angles()[0] * -0.1
        # correction is an integer which is the negative of the error
        correction = int(err * -2)
        # steering to correct error
        motor_pair.move(motor_pair.PAIR_1, correction, velocity=speed)
    motor_pair.stop(motor_pair.PAIR_1, stop=SMART_BRAKE)


async def gyro_turn_left(degrees, speed):
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
    await gyro_reset()
    degrees = degrees * gyro_error_rate
    print(motion_sensor.tilt_angles()[0] * 0.1)
    print("before")
    print(degrees)
    print(motion_sensor.tilt_angles()[0] * 0.1)
    turnchecks = 1
    gap = degrees + (motion_sensor.tilt_angles()[0] * 0.1)
    agap = math.fabs(gap)
    # wait up to 3 loop cycles to make sure we are stable
    while turnchecks < 3:
        if agap < 30:
            speed = 40
        if agap < 10:
            speed = 20
        if (gap < 0) and (gap < -turn_error_rate):
            motor_pair.move_tank(motor_pair.PAIR_1, -speed, speed)
        if (gap > 0) and (gap > turn_error_rate):
            motor_pair.move_tank(motor_pair.PAIR_1, speed, -speed)
        if (gap >= -turn_error_rate) and (gap <= turn_error_rate):
            motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
            speed = 8
            turnchecks = turnchecks + 1
            await runloop.sleep_ms(50)
        gap = degrees + (motion_sensor.tilt_angles()[0] * 0.1)
        agap = math.fabs(gap)

    # stops the motors
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.HOLD)
    print("stop motors")
    await runloop.sleep_ms(500)
    print(motion_sensor.tilt_angles()[0] * 0.1)


async def gyro_back(distance_cm, speed):
    print("starting gyro_backup")
    await gyro_reset()
    motor.reset_relative_position(port.B, 0)  # reset relative degrees
    print("gyro back loop starting")
    while math.fabs(motor.relative_position(port.B)) < distance_cm:
        # compute the error in degrees
        err = motion_sensor.tilt_angles()[0] * -0.1
        # correction is an integer which is the negative of the error
        correction = int(err * 2)
        # steering to correct error
        motor_pair.move(motor_pair.PAIR_1, correction, velocity=-speed)
    motor_pair.stop(motor_pair.PAIR_1)


# This function makes the robot move forward until it reaches a black
# line, adjusting its speed based on the readings from the color sensors.
async def square_on_black_line(speed):
    left_speed = speed
    right_speed = speed
    # Keep moving as long as at least one motor is still going.
    while (left_speed + right_speed) > 0:
        # Check color sensors (ports C and E used in original code)
        try:
            if color_sensor.reflection(port.C) < 40:
                left_speed = 0
        except Exception:
            pass
        try:
            if color_sensor.reflection(port.E) < 40:
                right_speed = 0
        except Exception:
            pass
        motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)
    motor_pair.stop(motor_pair.PAIR_1)


async def HighTorqueAttachmentDown():
    motor_pair.pair(motor_pair.PAIR_2, port.F, port.A)
    motor_pair.move_for_time(motor_pair.PAIR_2, 2000, ultraslow_speed)


async def HighTorqueAttachmentUP():
    motor_pair.pair(motor_pair.PAIR_3, port.A, port.F)
    motor_pair.move_for_time(motor_pair.PAIR_2, 2000, ultraslow_speed)


async def runt():
    print("start Test Run")
    await robot_setup()
    # await motor.run_for_time(left_arm,1000, backward_high_speed) # flips up the sound mixer
    # await motor.run_for_time(right_arm,1000, backward_high_speed) # flips up the sound mixer
    # await gyro_turn_left(90,600)
    # await gyro_straight(30,500)
    await gyro_turn(-90, 500)
    # await gyro_straight(20,500)
    await gyro_turn(90, 500)


async def YellowRun():
    await robot_setup()
    await gyro_straight(35, medium_speed)
    await motor.run_for_time(port.B, 450, backward_high_speed)  # Turn left quickly to push down dragon lever
    await gyro_back(10, medium_speed)
    await motor.run_for_time(port.D, 650, high_speed)  # turn to come home
    await gyro_back(500, medium_speed)


async def lemonrun():
    await robot_setup()
    await gyro_straight(40, medium_speed)
    await gyro_turn(-55, ultraslow_speed)
    await gyro_straight(5, medium_speed)
    await gyro_back(150, medium_speed)
    await gyro_turn(90, slow_speed)
    await gyro_back(1500, medium_speed)


async def GreenRun():
    # Theater scene change, orange person in skateboard, museum drop
    await robot_setup()
    await gyro_straight(52, medium_speed)  # go straight
    await gyro_turn(-63, slow_speed)  # turn toward museum
    await gyro_straight(80, medium_speed)  # go to museum and leave curater, masterpiece, and a person
    await gyro_back(160, medium_speed)  # back up from museum
    await gyro_turn(-90, medium_speed)  # turn toward immersive experience
    await motor.run_for_time(right_arm, 2000, -750)  # activate immersive experience
    await gyro_straight(5, medium_speed)
    await motor.run_for_degrees(right_arm, 250, medium_speed)
    await gyro_back(30, medium_speed)  # back away from immersive experience
    await gyro_turn(-45, medium_speed)
    await gyro_straight(20, medium_speed)
    await gyro_turn(90, medium_speed)
    await gyro_straight(4, medium_speed)  # move towards skateboard drop
    await motor.run_for_time(left_arm, 1500, 300)
    await motor.run_for_degrees(left_arm, 90, -medium_speed)


async def RedRun():
    await robot_setup()


async def BlackRun():
    await robot_setup()
    await gyro_back(700, medium_speed)
    await gyro_straight(7, medium_speed)
    await gyro_turn(90, slow_speed)
    await gyro_back(150, medium_speed)
    await gyro_turn(-90, slow_speed)
    await gyro_back(210, medium_speed)
    await gyro_turn(80, slow_speed)
    await gyro_back(650, medium_speed)
    await gyro_straight(8, medium_speed)
    await gyro_turn(135, slow_speed)
    await gyro_back(1500, medium_speed)


async def OrangeRun():
    await robot_setup()
    await gyro_straight(83, medium_speed)  # push the boat and an orange man into the movie scene
    await gyro_back(50, medium_speed)  # back up
    await gyro_turn(65, medium_speed)  # turn toward center to drop off another orange guy
    await gyro_straight(20, medium_speed)
    await motor.run_for_time(left_arm, 1100, high_speed)
    await motor.run_for_time(left_arm, 1000, -high_speed)
    await gyro_turn(-65, medium_speed)
    await gyro_straight(200, medium_speed)


async def TealRun():
    await robot_setup()


runloop.run(GreenRun())