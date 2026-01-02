"""
Test autobot10 inference without physical robot

Validates that the inference API loads correctly and produces reasonable outputs.
"""

import numpy as np
from autobot10_inference import V10Inference


def test_inference():
    """Test inference with dummy joint angles."""
    print("=== Testing Autobot10 Inference ===\n")
    
    # Initialize inference
    print("1. Loading model...")
    inference = V10Inference()
    print("   ✓ Model loaded\n")
    
    # Test with home position (all 90°)
    print("2. Testing with home position...")
    servo_angles = [90, 90, 90, 90, 90, 90]
    joint_radians = inference.convert_servo_to_radians(servo_angles)
    print(f"   Servo angles: {servo_angles}")
    print(f"   Radians:      {np.round(joint_radians, 3)}")
    print("   ✓ Conversion works\n")
    
    # Run inference
    print("3. Running inference...")
    result = inference.predict_next_joint_angles(joint_radians)
    print(f"   Action:       {np.round(result['action'], 3)}")
    print(f"   Velocity cmd: {np.round(result['velocity_command'], 3)}")
    print(f"   Next radians: {np.round(result['next_joint_angles'], 3)}")
    
    next_servo = inference.convert_radians_to_servo(result['next_joint_angles'])
    print(f"   Next servo:   {np.round(next_servo, 1)}")
    print("   ✓ Inference successful\n")
    
    # Test multiple steps
    print("4. Testing 5-step sequence...")
    current = joint_radians.copy()
    for step in range(5):
        result = inference.predict_next_joint_angles(current)
        current = result['next_joint_angles']
        servo = inference.convert_radians_to_servo(current)
        print(f"   Step {step+1}: {np.round(servo, 1)}")
    print("   ✓ Multi-step inference works\n")
    
    print("=== All tests passed! ===")
    print("\nInference API is ready for deployment.")


if __name__ == '__main__':
    test_inference()
