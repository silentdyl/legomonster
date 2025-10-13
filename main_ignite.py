from hub import light_matrix, motion_sensor, port
import motor
import math
import runloop
import motor_pair
import sys
from motor import ERROR, SMART_BRAKE
import color_sensor
import robot_moves
import grabber


# GreenRun - main_ignite.py
# This is the main program file for the GreenRun project.  
# It initializes the robot and calls functions to perform tasks.

def main():
    # Initialize the robot
    robot_moves.initialize_robot()
    
async def MissionOne():
    # Perform the first mission
    await robot_moves.robot_setup()

async def MissionTwo():
    # Perform the second mission
    await robot_moves.robot_setup()

async def main_sequence():
    try:
        await MissionOne()   # runs until finished
        await MissionTwo() # runs after GreenRun completes
    finally:
        # Ensure motors are stopped even if something fails
        try:
            motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
        except Exception:
            pass

# Start the sequence
runloop.run(main_sequence())       # Release the grabber