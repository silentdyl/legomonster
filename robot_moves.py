from hub import light_matrix, motion_sensor, port, motion_sensor
import motor
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
left_arm = port.A
right_arm = port.F
ultraslow_speed = 200
slow_speed = 400
medium_speed = 750
high_speed = 1000
slowing_down = 360  # approx degrees at which to slow down (one rotation)
backward_high_speed = -1000
gyro_error_rate = 1.00  # empirical multiplier to compensate gyro bias
turn_error_rate = 0.5   # tolerance used for finishing turns (degrees)


def degrees_for_distance(distance_cm):
    """Convert a linear distance in cm to an approximate wheel degree rotation.

    The conversion factor (16.014 cm per wheel revolution) is project-specific
    and may need adjustment for your wheel size/gear ratio.
    """
    return int((distance_cm / 16.014) * 360)

async def turn_motor_degrees_motorclass(port_obj, degrees, speed=300, hold=True):
    """
    Turn a single motor a number of degrees using the Motor class helper.

    Inputs:
    - port_obj: port object (e.g. port.A) or a Motor instance
    - degrees: signed degrees to rotate (positive/negative)
    - speed: power/speed parameter the API expects (units depend on implementation)
    - hold: if True, brake/hold at the end; if False, coast

    Behavior:
    - Uses Motor.run_for_degrees where available (blocking). Wrap in async to
      keep API consistent with your runloop.
    """
    # If 'port_obj' is a port identifier, construct Motor; if it's already a Motor use it.
    motor = port_obj if isinstance(port_obj, Motor) else Motor(port_obj)

    # Many Motor.run_for_degrees signatures are: run_for_degrees(degrees, speed, brake=True)
    # Adapt the call if your API uses different parameter names.
    motor.run_for_degrees(degrees, speed, brake=hold)

    # Small await to yield to runloop if desired
    await runloop.sleep_ms(1)

async def robot_setup():
    """Perform basic setup: pair left/right drive motors.

    This function should be awaited during initialization so that the
    motor_pair is configured before driving functions run.
    """
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)  # left/right drive motors


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
    motor.reset_relative_position(port.B, 0)  # reset encoder
    print("gyro back loop starting")

    # WARNING: distance_cm may be in cm; original code compares it to
    # encoder degrees. If you expect to pass cm, convert with
    # degrees_for_distance(distance_cm) before comparing.
    while math.fabs(motor.relative_position(port.B)) < distance_cm:
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
        light_matrix.write('X')  # or set a pattern if available
    except Exception:
        pass

