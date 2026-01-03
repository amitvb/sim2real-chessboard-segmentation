"""
Sim2Real Control Loop - Deploy Autobot10 to Physical Robot

Captures camera feed and joint angles from physical DofBot, runs autobot10 inference,
and applies predicted actions back to the robot in a closed loop.
"""

import sys
import time
import numpy as np
import cv2
from pathlib import Path
from datetime import datetime

# Add dofbot_lib to path
sys.path.insert(0, 'c:/junkyard/ai/autobots/dofbot_lib')

from hardware import DofbotRobot, Camera
from autobot10_inference import V10Inference
from autobot10_segmented_inference import V10SegmentedInference
from sim2real_visualizer import Sim2RealVisualizer

# Add chess-model-evaluation to path for ChessboardSegmenter
chess_model_path = Path(__file__).parent.parent / "chess-model-evaluation" / "models" / "YOLO-amit-seg"
sys.path.insert(0, str(chess_model_path))

try:
    from segment_and_filter import ChessboardSegmenter
    SEGMENTATION_AVAILABLE = True
except ImportError:
    SEGMENTATION_AVAILABLE = False
    ChessboardSegmenter = None
    print("Warning: ChessboardSegmenter not available. V10-segmented model will use zero features.")


class Sim2RealController:
    """Main controller for sim2real deployment."""
    
    def __init__(self, num_steps=30, model_path=None, save_images=True, 
                 servo_time_ms=200, settling_delay_s=0.25, use_default_start=True,
                 pose_type='sim2real', enable_visualization=False, use_segmented_model=False):
        """Initialize controller.
        
        Args:
            num_steps: Number of control steps to execute
            model_path: Path to autobot10 model
            save_images: Whether to save captured images
            servo_time_ms: Servo movement time in milliseconds
            settling_delay_s: Additional delay after command for servo settling
            use_default_start: Move to default starting pose before execution
            pose_type: Starting pose type ('sim2real', 'autobot8', or 'v10')
            enable_visualization: Show real-time arm pose and camera feed
            use_segmented_model: Use v10-segmented model (20D obs with segmentation)
        """
        self.num_steps = num_steps
        self.save_images = save_images
        self.servo_time_ms = servo_time_ms
        self.settling_delay_s = settling_delay_s
        self.use_default_start = use_default_start
        self.pose_type = pose_type
        self.enable_visualization = enable_visualization
        self.use_segmented_model = use_segmented_model
        self.output_dir = Path("outputs") / datetime.now().strftime("%Y%m%d_%H%M%S")
        self.visualizer = None
        self.segmenter = None
        
        if save_images:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            print(f"Saving outputs to: {self.output_dir}")
        
        # Initialize components
        print("\n=== Initializing Sim2Real Controller ===")
        
        if self.use_segmented_model:
            print("\n1. Loading autobot10-segmented V10 model (20D with segmentation)...")
            self.inference = V10SegmentedInference(model_path=model_path, image_width=640, image_height=480)
            
            # Initialize segmentation detector
            if SEGMENTATION_AVAILABLE:
                print("   Initializing ChessboardSegmenter...")
                self.segmenter = ChessboardSegmenter()
                print("   Segmenter ready")
            else:
                print("   WARNING: Segmentation not available, using zero features")
        else:
            print("\n1. Loading autobot10 V10 model (10D joints only)...")
            self.inference = V10Inference(model_path=model_path)
        
        print("\n2. Connecting to physical robot...")
        self.robot = DofbotRobot(port="COM5")
        self.robot.set_rgb(0, 0, 255)  # Blue = initializing
        
        # Move to default starting pose if requested
        if self.use_default_start:
            print(f"   Moving to {self.pose_type} default starting pose...")
            
            # First, explicitly move to home position to ensure clean state
            print(f"   Step 1: Moving to home position [90, 90, 90, 90, 90, 90]...")
            self.robot.home()
            time.sleep(1.5)
            
            # Now move to actual starting pose
            starting_pose = self.inference.get_default_starting_pose(
                as_servo=True, 
                use_autobot8=(self.pose_type == 'autobot8'),
                use_sim2real=(self.pose_type == 'sim2real')
            )
            print(f"   Step 2: Moving to target pose {np.round(starting_pose, 1)}")
            
            # Send command with longer movement time to ensure completion
            self.robot.set_all_servos(starting_pose.tolist(), time_ms=3000)
            time.sleep(4.0)  # Wait for full movement (3s movement + 1s settling)
            
            print(f"   Robot initialization complete")
        
        self.robot.set_rgb(0, 255, 0)  # Green = ready
        
        print("\n3. Opening camera...")
        self.camera = Camera(camera_index=0, width=640, height=480)
        self.camera.open()
        
        # Initialize visualizer if enabled
        if self.enable_visualization:
            print("\n4. Starting visualization...")
            robot_urdf = Path("c:/junkyard/ai/autobots/autobot10/urdf/dofbot.urdf")
            self.visualizer = Sim2RealVisualizer(str(robot_urdf))
            self.visualizer.start()
            print("   Visualization window opened")
        
        print("\n=== Initialization Complete ===\n")
    
    def run(self):
        """Execute main control loop."""
        print(f"Starting sim2real control loop for {self.num_steps} steps...")
        print("Press Ctrl+C to stop\n")
        
        # Reset velocity tracking
        self.inference.reset_velocities()
        
        # Storage for trajectory
        trajectory = []
        
        try:
            for step in range(self.num_steps):
                step_start_time = time.time()
                
                # === Step 1: Capture current state ===
                # Read camera feed
                frame = self.camera.read_frame()
                if frame is None:
                    print(f"  Failed to capture image")
                    return None
                
                # Get current joint angles (servo angles 0-180°)
                # Use commanded positions instead of hardware read (hardware read is unreliable)
                current_servo_angles = self.robot.get_current_positions(use_hardware_read=False)
                
                # Convert to radians for model
                current_joint_radians = self.inference.convert_servo_to_radians(
                    current_servo_angles
                )
                
                # === Step 2: Run segmentation (if using segmented model) ===
                detection = None
                seg_result_for_viz = None
                if self.use_segmented_model and self.segmenter is not None:
                    # Run segmentation on captured frame
                    seg_result = self.segmenter.analyze(frame, conf_threshold=0.2)
                    
                    # Convert to detection dict format expected by inference
                    if seg_result['detected']:
                        # Get bounding box from polygon
                        polygon = seg_result['polygon']
                        if polygon is not None and len(polygon) > 0:
                            x_min, y_min = polygon.min(axis=0)
                            x_max, y_max = polygon.max(axis=0)
                            bbox = (x_min, y_min, x_max - x_min, y_max - y_min)
                            bbox_viz = (int(x_min), int(y_min), int(x_max), int(y_max))  # For visualization
                        else:
                            bbox = None
                            bbox_viz = None
                        
                        detection = {
                            'bbox': bbox,
                            'confidence': seg_result['confidence'],
                            'coverage': seg_result['coverage'],
                            'edges_cut': seg_result['edges_cut'],
                            'chessboard_present': True
                        }
                        
                        # Prepare for visualization
                        seg_result_for_viz = {
                            'bbox': bbox_viz,
                            'confidence': seg_result['confidence'],
                            'coverage': seg_result['coverage'],
                            'mask': seg_result.get('mask'),
                            'chessboard_present': True
                        }
                    else:
                        detection = {
                            'bbox': None,
                            'confidence': 0.0,
                            'coverage': 0.0,
                            'edges_cut': [],
                            'chessboard_present': False
                        }
                
                # === Step 3: Run model inference ===
                if self.use_segmented_model:
                    result = self.inference.predict_next_joint_angles(
                        current_joint_radians,
                        detection=detection,
                        deterministic=True
                    )
                else:
                    result = self.inference.predict_next_joint_angles(
                        current_joint_radians,
                        deterministic=True
                    )
                
                next_joint_radians = result['next_joint_angles']
                
                # Convert back to servo angles
                next_servo_angles = self.inference.convert_radians_to_servo(
                    next_joint_radians
                )
                
                # === Step 4: Apply to physical robot ===
                # Apply action to robot
                self.robot.set_all_servos(next_servo_angles.tolist(), time_ms=self.servo_time_ms)
                
                # Update visualizations
                if self.visualizer:
                    self.visualizer.update_arm_pose(next_servo_angles.tolist())
                    self.visualizer.update_camera_feed(frame, segmentation_result=seg_result_for_viz)
                
                time.sleep(self.settling_delay_s)
                
                # === Step 5: Save data ===
                step_data = {
                    'step': step,
                    'current_servo': current_servo_angles,
                    'current_radians': current_joint_radians,
                    'next_servo': next_servo_angles,
                    'next_radians': next_joint_radians,
                    'action': result['action'],
                    'velocity_command': result['velocity_command']
                }
                
                # Add segmentation data if available
                if self.use_segmented_model:
                    step_data['detection'] = detection
                    if 'segmentation_features' in result:
                        step_data['segmentation_features'] = result['segmentation_features']
                trajectory.append(step_data)
                
                if self.save_images:
                    img_path = self.output_dir / f"step_{step:03d}.jpg"
                    cv2.imwrite(str(img_path), frame)
                
                # === Step 6: Print progress ===
                step_duration = time.time() - step_start_time
                print(f"Step {step+1}/{self.num_steps}:")
                print(f"  Current servo: {np.round(current_servo_angles, 1)}")
                print(f"  Next servo:    {np.round(next_servo_angles, 1)}")
                print(f"  Delta:         {np.round(next_servo_angles - current_servo_angles, 1)}")
                print(f"  Action:        {np.round(result['action'], 3)}")
                if self.use_segmented_model and detection:
                    print(f"  Board detected: {detection['chessboard_present']}")
                    if detection['chessboard_present']:
                        print(f"  Coverage:      {detection['coverage']:.1f}%")
                        print(f"  Confidence:    {detection['confidence']:.3f}")
                print(f"  Duration:      {step_duration:.3f}s")
                print()
            
            print("=== Control loop completed successfully ===")
            self.robot.set_rgb(0, 255, 0)  # Green = success
            self.robot.buzzer(duration_ms=200)
            
            # Save trajectory
            if self.save_images:
                self._save_trajectory(trajectory)
            
            # Keep visualizer window open for inspection
            if self.visualizer:
                print("\nVisualization window will remain open. Press ESC in the window to close.")
                self.visualizer.stop()  # Stop updating but keep window
                self.visualizer.wait_for_close()  # Wait for user to close
            
        except KeyboardInterrupt:
            print("\n=== Interrupted by user ===")
            self.robot.set_rgb(255, 255, 0)  # Yellow = stopped
        except Exception as e:
            print(f"\n=== Error occurred: {e} ===")
            self.robot.set_rgb(255, 0, 0)  # Red = error
            raise
        finally:
            self.cleanup()
    
    def _save_trajectory(self, trajectory):
        """Save trajectory data to file."""
        import json
        
        def convert_to_serializable(obj):
            """Recursively convert numpy types to Python native types."""
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, (np.float32, np.float64)):
                return float(obj)
            elif isinstance(obj, (np.int32, np.int64)):
                return int(obj)
            elif isinstance(obj, dict):
                return {k: convert_to_serializable(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [convert_to_serializable(item) for item in obj]
            else:
                return obj
        
        # Convert all data to JSON-serializable format
        trajectory_serializable = convert_to_serializable(trajectory)
        
        trajectory_path = self.output_dir / "trajectory.json"
        with open(trajectory_path, 'w') as f:
            json.dump(trajectory_serializable, f, indent=2)
        
        print(f"Trajectory saved to: {trajectory_path}")
    
    def cleanup(self):
        """Clean up resources."""
        print("\nCleaning up...")
        
        if self.visualizer:
            self.visualizer.stop()
            print("  Visualization closed")
        
        if hasattr(self, 'camera'):
            self.camera.close()
            print("  Camera closed")
        
        if hasattr(self, 'robot'):
            self.robot.close()
            print("  Robot connection closed")
        
        print("Cleanup complete")


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Sim2Real Chessboard Segmentation Control")
    parser.add_argument('--steps', type=int, default=30,
                        help='Number of control steps (default: 30)')
    parser.add_argument('--model', type=str, default=None,
                        help='Path to autobot10 model (default: use built-in path)')
    parser.add_argument('--no-save', action='store_true',
                        help='Disable saving images and trajectory')
    parser.add_argument('--servo-time', type=int, default=200,
                        help='Servo movement time in milliseconds (default: 200)')
    parser.add_argument('--settling-delay', type=float, default=0.25,
                        help='Settling delay in seconds after servo command (default: 0.25)')
    parser.add_argument('--pose-type', type=str, default='sim2real', 
                        choices=['sim2real', 'autobot8', 'v10'],
                        help='Starting pose type (default: sim2real)')
    parser.add_argument('--visualize', action='store_true',
                        help='Enable real-time visualization of arm pose and camera feed')
    parser.add_argument('--use-segmented', action='store_true',
                        help='Use v10-segmented model with 20D observation (includes segmentation features)')

    args = parser.parse_args()

    controller = Sim2RealController(
        num_steps=args.steps,
        model_path=args.model,
        save_images=not args.no_save,
        servo_time_ms=args.servo_time,
        settling_delay_s=args.settling_delay,
        use_default_start=True,  # Always start from default pose
        pose_type=args.pose_type,
        enable_visualization=args.visualize,
        use_segmented_model=args.use_segmented
    )

    controller.run()


if __name__ == '__main__':
    main()
