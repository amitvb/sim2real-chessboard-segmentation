"""
Real-time visualization for sim2real deployment.
Shows arm pose rendering and camera feed side-by-side.
"""

import numpy as np
import cv2
import pybullet as p
import pybullet_data
from pathlib import Path
import threading
import queue


class Sim2RealVisualizer:
    """Real-time visualizer for sim2real arm pose and camera feed."""
    
    def __init__(self, robot_urdf_path):
        """Initialize visualizer.
        
        Args:
            robot_urdf_path: Path to robot URDF file
        """
        self.robot_urdf_path = robot_urdf_path
        self.physics_client = None
        self.robot_id = None
        self.joint_indices = []
        
        # Visualization settings
        self.render_width = 640
        self.render_height = 480
        self.window_name = "Sim2Real Visualization"
        
        # Threading
        self.running = False
        self.display_thread = None
        self.render_queue = queue.Queue(maxsize=1)
        self.camera_queue = queue.Queue(maxsize=1)
        
    def start(self):
        """Start the visualizer."""
        if self.running:
            return
        
        # Initialize PyBullet in DIRECT mode
        self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane
        p.loadURDF("plane.urdf")
        
        # Load robot
        self.robot_id = p.loadURDF(
            self.robot_urdf_path,
            basePosition=[0, 0, 0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, -np.pi/2]),
            useFixedBase=True
        )
        
        # Get joint indices
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.joint_indices.append(i)
        
        # Create table and board for context
        self._create_scene()
        
        # Start display thread
        self.running = True
        self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self.display_thread.start()
        
    def _create_scene(self):
        """Create table and chessboard for visualization context."""
        # Table
        table_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.6, 0.4, 0.025])
        table_vis = p.createVisualShape(
            p.GEOM_BOX, 
            halfExtents=[0.6, 0.4, 0.025],
            rgbaColor=[0.6, 0.4, 0.2, 1]
        )
        p.createMultiBody(0, table_col, table_vis, [0.33, 0, 0.025])
        
        # Chessboard
        board_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.08, 0.08, 0.005])
        board_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.08, 0.08, 0.005],
            rgbaColor=[0.9, 0.9, 0.9, 1]
        )
        p.createMultiBody(0, board_col, board_vis, [0.33, 0, 0.055])
        
    def update_arm_pose(self, servo_angles):
        """Update arm pose visualization.
        
        Args:
            servo_angles: List of 6 servo angles in degrees
        """
        if not self.running:
            return
        
        # Convert servo angles to radians
        radians = [(s - 90) * np.pi / 180 for s in servo_angles[:5]]
        
        # Update joint positions
        for i, angle in enumerate(radians):
            if i < len(self.joint_indices):
                p.resetJointState(self.robot_id, self.joint_indices[i], angle)
        
        # Step simulation for physics update
        p.stepSimulation()
        
        # Render the scene
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[0.5, 0.5, 0.4],
            cameraTargetPosition=[0.2, 0, 0.2],
            cameraUpVector=[0, 0, 1]
        )
        
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=self.render_width / self.render_height,
            nearVal=0.1,
            farVal=3.0
        )
        
        # Capture rendered image
        img_data = p.getCameraImage(
            self.render_width,
            self.render_height,
            view_matrix,
            projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # Convert to OpenCV format
        rgb_array = np.array(img_data[2], dtype=np.uint8).reshape(
            (self.render_height, self.render_width, 4)
        )
        rgb_array = rgb_array[:, :, :3]  # Remove alpha
        
        # Update queue (non-blocking)
        try:
            self.render_queue.put_nowait(rgb_array)
        except queue.Full:
            pass
    
    def update_camera_feed(self, camera_frame):
        """Update camera feed visualization.
        
        Args:
            camera_frame: Camera image (numpy array)
        """
        if not self.running or camera_frame is None:
            return
        
        # Update queue (non-blocking)
        try:
            self.camera_queue.put_nowait(camera_frame.copy())
        except queue.Full:
            pass
    
    def _display_loop(self):
        """Display loop running in separate thread."""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 480)
        
        # Initialize with blank frames
        render_frame = np.zeros((self.render_height, self.render_width, 3), dtype=np.uint8)
        camera_frame = np.zeros((self.render_height, self.render_width, 3), dtype=np.uint8)
        
        while self.running:
            # Get latest render frame
            try:
                render_frame = self.render_queue.get_nowait()
            except queue.Empty:
                pass
            
            # Get latest camera frame
            try:
                camera_frame = self.camera_queue.get_nowait()
                # Resize camera frame to match render height
                camera_frame = cv2.resize(camera_frame, (self.render_width, self.render_height))
            except queue.Empty:
                pass
            
            # Add labels
            render_labeled = render_frame.copy()
            camera_labeled = camera_frame.copy()
            
            cv2.putText(
                render_labeled, "PyBullet Arm Pose", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )
            
            cv2.putText(
                camera_labeled, "Physical Camera Feed", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )
            
            # Combine frames side-by-side
            combined = np.hstack([render_labeled, camera_labeled])
            
            # Display
            cv2.imshow(self.window_name, combined)
            
            # Check for window close or ESC key
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                self.running = False
                break
        
        cv2.destroyWindow(self.window_name)
    
    def stop(self):
        """Stop the visualizer."""
        self.running = False
        
        if self.display_thread is not None:
            self.display_thread.join(timeout=2.0)
        
        if self.physics_client is not None:
            p.disconnect()
            self.physics_client = None
