# Sim2Real Chessboard Segmentation

Physical robot deployment system for trained RL models on DofBot SE arm.

## Overview

This project deploys trained models from autobot10 and autobot10-segmented to control the physical DofBot arm. Supports both:
- **V10**: Basic model with 10D observations (joint positions + velocities)
- **V10-Segmented**: Vision-guided model with 20D observations (joints + segmentation features)

## Architecture

1. **Camera Capture** - Uses `dofbot_lib.Camera` to capture real camera feed from physical arm
2. **Joint Reading** - Uses `dofbot_lib.DofbotRobot` to read current joint angles
3. **Model Inference** - Passes data to autobot10's trained SAC model for next action
4. **Joint Control** - Applies predicted actions as new joint positions to physical arm
5. **Loop** - Repeats for configurable number of steps (default: 30)

## Components

- `run_sim2real.py` - Main control loop for physical robot deployment
- `autobot10_inference.py` - Wrapper for V10 model inference
- `autobot10_segmented_inference.py` - Wrapper for V10-Segmented model with vision
- `sim2real_visualizer.py` - Real-time 3D visualization of arm movements
- `verify_pose.py` - Pose verification and testing utility
- `config.yaml` - Configuration (step count, hardware settings)

**Note:** Model inference logic resides in respective training projects for self-containment.

## Setup

```bash
# Install dependencies
pip install -r requirements.txt

# Ensure dofbot_lib is accessible
# Ensure autobot10 model is trained and available
```

## Usage

### Basic V10 Model
```bash
python run_sim2real.py --model ../autobot10/models/v10/board_view_v10_final_*.zip
```

### V10-Segmented Model (Vision-Guided)
```bash
python run_sim2real.py --use-segmented --model ../autobot10-segmented/models/v10-segmented/board_view_v10_final_*.zip
```

### With Visualization
```bash
python run_sim2real.py --model <path> --visualize
```

The system will:
1. Initialize physical robot and camera
2. Load specified trained model
3. Run model inference for specified steps (default: 30)
4. Apply predicted actions to physical arm
5. Save trajectory data and captured images to `outputs/`

## Requirements

- Physical DofBot SE arm connected via USB (default COM5)
- USB camera (default index 0)
- Trained model from autobot10 or autobot10-segmented
- For V10-segmented: YOLOv8-seg model at `../chess-model-evaluation/models/YOLO-amit-seg/`

## Safety

- Ensure robot has clear workspace
- Start from home position
- Monitor first runs carefully
- Emergency stop: Ctrl+C or disconnect power
