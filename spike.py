"""Local spike stub for editor/language-server resolution only.

This file is intended to live in the repository to satisfy editors (Pylance,
Pyright) that can't see the hub runtime. It is NOT required on the hub and
contains minimal no-op implementations of the commonly-used classes.

If you copy/paste the script into the SPIKE Prime app, the hub's real
`spike` module will be used instead of this stub.
"""

class PrimeHub:
    def __init__(self):
        """Minimal PrimeHub stub."""
        pass

class Motor:
    def __init__(self, port: str):
        self.port = port

    def run_for_rotations(self, rotations: float, speed: int, *args, **kwargs):
        pass

    def run_for_degrees(self, degrees: int, speed: int, *args, **kwargs):
        pass

    def start(self, speed: int):
        pass

    def stop(self):
        pass

class MotorPair:
    PAIR_1 = 1

    def __init__(self, a: str, b: str):
        self.a = a
        self.b = b

    def move_tank(self, amount: int, unit: str, left: int, right: int):
        pass

    def start_tank(self, left: int, right: int):
        pass

    def move(self, pair_id: int, angle: int, velocity: int = None):
        pass

    def stop(self):
        pass

__all__ = ["PrimeHub", "Motor", "MotorPair"]
