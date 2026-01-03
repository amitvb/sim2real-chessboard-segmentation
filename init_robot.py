"""
Standalone Robot Initialization Script

Moves the DofBot robot to the default starting pose for sim2real deployment.
Run this before starting run_sim2real.py to ensure proper initialization.

Usage:
    python init_robot.py [--pose-type {sim2real,autobot8,v10}]
"""

import sys
import time
import argparse
import numpy as np
sys.path.append('c:/junkyard/ai/autobots/dofbot_lib')

from hardware import DofbotRobot

# Default starting poses (hardcoded to avoid loading model)
POSES = {
    'sim2real': np.array([90.0, 80.0, 55.0, 30.0, 90.0, 90.0]),
    'autobot8': np.array([90.0, 61.4, 135.8, 90.0, 118.6, 90.0]),
    'v10': np.array([90.0, 135.9, 124.4, 112.9, 90.0, 90.0])
}

def move_to_starting_pose(pose_type='sim2real', retries=3):
    """Move robot to default starting pose with retries.
    
    Args:
        pose_type: Type of starting pose ('sim2real', 'autobot8', or 'v10')
        retries: Number of attempts to move to starting pose
    """
    print("=" * 60)
    print("Robot Initialization Script")
    print("=" * 60)
    
    # Get target starting pose
    print(f"\n1. Loading pose configuration...")
    starting_pose = POSES[pose_type]
    print(f"   Target pose ({pose_type}): {starting_pose.astype(int)}")
    
    # Connect to robot
    print(f"\n2. Connecting to robot on COM5...")
    robot = DofbotRobot(port="COM5")
    robot.set_rgb(255, 165, 0)  # Orange = initializing
    time.sleep(0.5)
    
    # Move to home first for clean state
    print(f"\n3. Moving to home position...")
    robot.home()
    time.sleep(2.0)
    print("   Home position reached")
    
    # Attempt to move to starting pose with retries
    success = False
    for attempt in range(1, retries + 1):
        print(f"\n4. Moving to starting pose (attempt {attempt}/{retries})...")
        
        # Send command with long movement time
        robot.set_all_servos(starting_pose.tolist(), time_ms=3000)
        time.sleep(4.0)  # Wait for full movement
        
        # Read actual position from hardware
        actual_pos = robot.get_current_positions(use_hardware_read=True)
        print(f"   Target:  {starting_pose.astype(int)}")
        print(f"   Actual:  {[int(p) for p in actual_pos]}")
        
        # Check if close enough (within 5 degrees per joint)
        differences = [abs(a - t) for a, t in zip(actual_pos, starting_pose)]
        max_diff = max(differences)
        
        if max_diff < 5.0:
            print(f"   ✓ Position reached (max error: {max_diff:.1f}°)")
            success = True
            break
        else:
            print(f"   ✗ Position not reached (max error: {max_diff:.1f}°)")
            if attempt < retries:
                print(f"   Retrying in 2 seconds...")
                time.sleep(2.0)
    
    if success:
        robot.set_rgb(0, 255, 0)  # Green = success
        print(f"\n{'=' * 60}")
        print("✓ Robot successfully initialized to starting pose")
        print("  You can now run: python run_sim2real.py")
        print(f"{'=' * 60}")
    else:
        robot.set_rgb(255, 0, 0)  # Red = failed
        print(f"\n{'=' * 60}")
        print("✗ Failed to initialize robot after {retries} attempts")
        print("  Please manually move the robot to starting pose:")
        print(f"  {starting_pose.astype(int)}")
        print(f"{'=' * 60}")
    
    # Cleanup
    robot.close()
    
    return success


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Initialize robot to starting pose")
    parser.add_argument('--pose-type', type=str, default='sim2real',
                        choices=['sim2real', 'autobot8', 'v10'],
                        help='Type of starting pose (default: sim2real)')
    parser.add_argument('--retries', type=int, default=3,
                        help='Number of retry attempts (default: 3)')
    
    args = parser.parse_args()
    
    try:
        success = move_to_starting_pose(args.pose_type, args.retries)
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
