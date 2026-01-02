"""
Verify physical arm pose matches expected default pose.
Moves arm to sim2real default pose and reads actual servo angles.
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add dofbot_lib to path
sys.path.insert(0, 'c:/junkyard/ai/autobots/dofbot_lib')

from hardware import DofbotRobot

# Expected sim2real default pose
EXPECTED_POSE = [90, 80, 55, 30, 90, 90]

def verify_pose():
    """Move arm to default pose and verify actual angles."""
    print("="*70)
    print("SIM2REAL POSE VERIFICATION")
    print("="*70)
    
    print("\nExpected sim2real default pose:")
    print(f"  Servo angles: {EXPECTED_POSE}")
    print(f"  [Base, Shoulder, Elbow, Wrist1, Wrist2, Gripper]")
    
    # Connect to robot
    print("\n1. Connecting to physical robot...")
    robot = DofbotRobot(port="COM5")
    robot.set_rgb(255, 255, 0)  # Yellow = verifying
    
    # Read initial position
    print("\n2. Reading initial position...")
    initial_angles = robot.get_current_positions(use_hardware_read=True)
    print(f"   Initial angles: {initial_angles}")
    
    # Move to expected pose
    print("\n3. Moving to sim2real default pose...")
    robot.set_all_servos(EXPECTED_POSE, time_ms=2000)
    time.sleep(2.5)  # Wait for movement to complete + settling
    
    # Read actual position multiple times for accuracy
    print("\n4. Reading actual servo positions (3 readings)...")
    readings = []
    for i in range(3):
        time.sleep(0.3)
        angles = robot.get_current_positions(use_hardware_read=True)
        readings.append(angles)
        print(f"   Reading {i+1}: {angles}")
    
    # Average readings
    avg_angles = [int(round(sum(r[i] for r in readings) / len(readings))) 
                  for i in range(6)]
    
    print("\n5. Analysis:")
    print(f"   Average angles: {avg_angles}")
    print(f"   Expected:       {EXPECTED_POSE}")
    
    # Calculate differences
    differences = [avg_angles[i] - EXPECTED_POSE[i] for i in range(6)]
    print(f"   Differences:    {differences}")
    
    # Check if within tolerance
    tolerance = 3  # degrees
    print(f"\n6. Verification (tolerance: ±{tolerance}°):")
    all_good = True
    joint_names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Gripper"]
    
    for i, name in enumerate(joint_names):
        expected = EXPECTED_POSE[i]
        actual = avg_angles[i]
        diff = differences[i]
        
        if abs(diff) <= tolerance:
            status = "✓ OK"
        else:
            status = "✗ MISMATCH"
            all_good = False
        
        print(f"   {name:10} Expected: {expected:3}°  Actual: {actual:3}°  Diff: {diff:+3}°  {status}")
    
    print("\n" + "="*70)
    if all_good:
        print("✓ VERIFICATION PASSED - All joints within tolerance")
        robot.set_rgb(0, 255, 0)  # Green = success
    else:
        print("✗ VERIFICATION FAILED - Some joints outside tolerance")
        robot.set_rgb(255, 0, 0)  # Red = failed
    print("="*70)
    
    # Cleanup
    time.sleep(1)
    robot.close()
    
    return all_good, avg_angles, differences


if __name__ == '__main__':
    try:
        success, actual, diffs = verify_pose()
        
        if not success:
            print("\n⚠️  TROUBLESHOOTING SUGGESTIONS:")
            print("1. Check if servo calibration is correct")
            print("2. Verify servo power supply is stable")
            print("3. Check for mechanical obstructions")
            print("4. Re-run verification to confirm readings")
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n\nVerification cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nError during verification: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
