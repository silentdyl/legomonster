"""Spike Prime demo (object-oriented API)

Ready to paste into the SPIKE Prime app. Uses PrimeHub, Motor and MotorPair.
"""

try:
	from spike import PrimeHub, Motor, MotorPair
	SPIKE = True
except Exception:
	# Editor or desktop Python won't have `spike`. Provide light stubs so
	# the file remains editable and the editor stops complaining.
	SPIKE = False

	try:
		import hub as _hub
		PrimeHub = getattr(_hub, 'PrimeHub', lambda: None)
	except Exception:
		PrimeHub = lambda: None

	class Motor:
		def __init__(self, port):
			self.port = port

		def run_for_rotations(self, *args, **kwargs):
			print('(stub) run_for_rotations', args, kwargs)

		def run_for_degrees(self, *args, **kwargs):
			print('(stub) run_for_degrees', args, kwargs)

		def start(self, *args, **kwargs):
			pass

		def stop(self, *args, **kwargs):
			pass

	class MotorPair:
		PAIR_1 = 1

		def __init__(self, a, b):
			self.a = a
			self.b = b

		def move_tank(self, *args, **kwargs):
			print('(stub) move_tank', args, kwargs)

		def start_tank(self, *args, **kwargs):
			pass

		def stop(self, *args, **kwargs):
			pass

import runloop

# Robot geometry (adjust to your robot)
CIRCUMFERENCE_CM = 17.6  # wheel circumference in cm

# Create hub and motors
hub = PrimeHub()
left = Motor('A')
right = Motor('B')
pair = MotorPair('A', 'B')


def degrees_for_distance(distance_cm):
	"""Return wheel degrees needed to travel distance_cm."""
	rotations = distance_cm / CIRCUMFERENCE_CM
	return int(rotations * 360)


def drive_cm(distance_cm, speed=50):
	"""Drive forward distance_cm at given speed. Uses MotorPair when
	available, otherwise falls back to individual motors.
	"""
	deg = degrees_for_distance(distance_cm)

	# Try MotorPair move_tank signature first (degrees, 'degrees', l, r)
	try:
		if hasattr(pair, 'move_tank'):
			try:
				pair.move_tank(deg, 'degrees', speed, speed)
				return
			except TypeError:
				# fallback to simpler start/stop
				pair.start_tank(speed, speed)
				runloop.sleep_ms(int(abs(distance_cm) * 10))
				pair.stop()
				return

		# Some firmwares expose move(start/stop) style
		if hasattr(pair, 'move'):
			pair.move(pair.PAIR_1, 0, velocity=speed)
			runloop.sleep_ms(int(abs(distance_cm) * 10))
			pair.stop()
			return
	except Exception:
		pass

	# Fallback: run both motors by degrees/rotations
	try:
		left.run_for_degrees(deg, speed)
		right.run_for_degrees(deg, speed)
	except Exception:
		try:
			rotations = distance_cm / CIRCUMFERENCE_CM
			left.run_for_rotations(rotations, speed)
			right.run_for_rotations(rotations, speed)
		except Exception:
			left.start(speed)
			right.start(speed)
			runloop.sleep_ms(int(abs(distance_cm) * 10))
			left.stop()
			right.stop()


def demo():
	# Example sequence: rotate right motor, rotate left motor, drive forward
	try:
		right.run_for_rotations(2, 50)
	except Exception as e:
		print('right.run_for_rotations failed:', e)

	try:
		left.run_for_degrees(360, 720)
	except Exception as e:
		print('left.run_for_degrees failed:', e)

	drive_cm(30, speed=50)


if __name__ == '__main__':
	demo()

