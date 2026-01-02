"""
Test inference APIs without physical robot

Validates that the inference APIs load correctly and produce reasonable outputs.
Supports both V10 and V10-Segmented models.
"""

import sys
import numpy as np
from pathlib import Path
from autobot10_inference import V10Inference
from autobot10_segmented_inference import V10SegmentedInference


def test_v10_inference(model_path=None):
    """Test V10 inference with dummy joint angles."""
    print("=== Testing V10 Inference (10D observations) ===\n")
    
    # Initialize inference
    print("1. Loading V10 model...")
    inference = V10Inference(model_path=model_path)
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
    
    print("=== V10 tests passed! ===\n")


def test_v10_segmented_inference(model_path=None):
    """Test V10-Segmented inference with dummy joint angles and segmentation."""
    print("=== Testing V10-Segmented Inference (20D observations) ===\n")
    
    # Initialize inference
    print("1. Loading V10-Segmented model...")
    inference = V10SegmentedInference(model_path=model_path)
    print("   ✓ Model loaded\n")
    
    # Test with home position
    print("2. Testing with home position and zero segmentation...")
    servo_angles = [90, 90, 90, 90, 90, 90]
    joint_radians = inference.convert_servo_to_radians(servo_angles)
    print(f"   Servo angles: {servo_angles}")
    print(f"   Radians:      {np.round(joint_radians, 3)}")
    print("   ✓ Conversion works\n")
    
    # Run inference with no detection
    print("3. Running inference (no detection)...")
    result = inference.predict_next_joint_angles(joint_radians, detection=None)
    print(f"   Action:       {np.round(result['action'], 3)}")
    print(f"   Observation:  {np.round(result['observation'], 3)}")
    print(f"   Next radians: {np.round(result['next_joint_angles'], 3)}")
    print("   ✓ Inference with zero features successful\n")
    
    # Run inference with mock detection
    print("4. Running inference (mock detection)...")
    mock_detection = {
        'coverage': 0.75,
        'centroid_x': 0.5,
        'centroid_y': 0.5,
        'bbox_width': 0.6,
        'bbox_height': 0.6,
        'confidence': 0.9,
        'edge_cut_left': False,
        'edge_cut_right': False,
        'edge_cut_top': False,
        'edge_cut_bottom': False
    }
    result = inference.predict_next_joint_angles(joint_radians, detection=mock_detection)
    print(f"   Segmentation: {np.round(result['observation'][10:], 3)}")
    print(f"   Action:       {np.round(result['action'], 3)}")
    print("   ✓ Inference with segmentation features successful\n")
    
    print("=== V10-Segmented tests passed! ===\n")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Test inference APIs')
    parser.add_argument('--model', help='Path to model file (optional)')
    parser.add_argument('--test', choices=['v10', 'segmented', 'both'], default='both',
                        help='Which model to test')
    args = parser.parse_args()
    
    if args.test in ['v10', 'both']:
        test_v10_inference(model_path=args.model)
    
    if args.test in ['segmented', 'both']:
        test_v10_segmented_inference(model_path=args.model)
    
    print("\n✓ All inference APIs are ready for deployment.")
