"""Reset robot to default starting pose."""

import sys
import time
import numpy as np

sys.path.insert(0, 'c:/junkyard/ai/autobots/dofbot_lib')
from hardware import DofbotRobot

print("=== Resetting Robot to Default Pose ===\n")

# Connect to robot
print("Connecting to robot...")
robot = DofbotRobot(port="COM5")
robot.set_rgb(0, 0, 255)  # Blue = working

# Default sim2real starting pose: [90, 80, 55, 30, 90, 90]
default_pose = [90, 80, 55, 30, 90, 90]

print(f"Current position before move: {robot.get_current_positions()}")
print(f"Moving to default pose: {default_pose}")

# Move in stages for stuck servos
robot.set_all_servos(default_pose, time_ms=3000)
time.sleep(3.5)

# Verify and retry if needed
current = robot.get_current_positions()
if not np.allclose(current, default_pose, atol=2.0):
    print(f"First attempt incomplete: {np.round(current, 1)}")
    print("Retrying movement...")
    robot.set_all_servos(default_pose, time_ms=3000)
    time.sleep(3.5)

# Confirm position
current = robot.get_current_positions()
print(f"Current position: {np.round(current, 1)}")

robot.set_rgb(0, 255, 0)  # Green = done
robot.buzzer(duration_ms=100)

print("\n✓ Robot reset complete")
robot.close()
