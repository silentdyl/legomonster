from hub import light_matrix, motion_sensor, port
import motor
import math
import runloop
import motor_pair
import sys
from motor import ERROR, SMART_BRAKE
import color_sensor
#import robot_moves

# Ports and speed constants — change to match your robot's wiring
left_arm = port.A
right_arm = port.F
ultraslow_speed = 200
slow_speed = 400
medium_speed = 750
high_speed = 1000
slowing_down = 360# approx degrees at which to slow down (one rotation)
backward_high_speed = -1000
gyro_error_rate = 1.00# empirical multiplier to compensate gyro bias
turn_error_rate = 0.5# tolerance used for finishing turns (degrees)

async def robot_setup():
    """Perform basic setup: pair left/right drive motors.

    This function should be awaited during initialization so that the
    motor_pair is configured before driving functions run.
    """
    motor_pair.pair(motor_pair.PAIR_1, port.B, port.D)

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

def degrees_for_distance(distance_cm):
    """Convert a linear distance in cm to an approximate wheel degree rotation.

    The conversion factor (16.014 cm per wheel revolution) is project-specific
    and may need adjustment for your wheel size/gear ratio.
    """
    return int((distance_cm / 16.014) * 360)

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



def main():
    # Initialize the robot
    #initialize_robot()

    async def TestMissionJed():
    await gyro_straight(140,50)

    #async def TestMission():
    # Perform the first mission
        await robot_setup()
        await gyro_straight(140, 50)

async def MissionOne():
    # Perform the first mission
        await robot_setup()

async def MissionTwo():
    # Perform the second mission
    await robot_setup()

async def main_sequence():
    try:
        await TestMission()# runs until finished
    # await MissionTwo() # runs after GreenRun completes
    finally:
        # Ensure motors are stopped even if something fails
        try:
            motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
        except Exception:
            pass

# Start the sequence

runloop.run(main())
