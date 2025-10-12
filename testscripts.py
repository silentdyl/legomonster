
#used to control the motors
from spike import Motor

from spike import MotionSensor, MotorPair

# Initialize the motors and motion sensor
left_motor = Motor("C")
right_motor = Motor("D")
motor_pair = MotorPair(left_motor, right_motor)

# Move the robot forward
motor_pair.move_for_degrees(360, 50)

# Turn the robot right
motor_pair.move_for_degrees(180, 50, turn_right=True)
# Move the robot backward
motor_pair.move_for_degrees(-360, 50)
# Turn the robot left
motor_pair.move_for_degrees(180, 50, turn_left=True)
# Stop the motors
motor_pair.stop()

motion_sensor = MotionSensor("A")
# Check if the motion sensor detects motion
if motion_sensor.is_motion_detected():
    print("Motion detected!")

# Get the distance measured by the motion sensor
distance = motion_sensor.get_distance_cm()
print(f"Distance: {distance} cm")
# Get the ambient light level measured by the motion sensor
light_level = motion_sensor.get_ambient_light()
print(f"Ambient Light Level: {light_level}")
# Get the color detected by the motion sensor
color = motion_sensor.get_color()
print(f"Detected Color: {color}")

    

