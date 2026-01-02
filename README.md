# Sim2Real Chessboard Segmentation

Physical robot control using autobot10's trained chessboard viewing model.

## Overview

This project deploys the autobot10 trained model (v10) to control the physical DofBot arm to find and view a chessboard in the real world.

## Architecture

1. **Camera Capture** - Uses `dofbot_lib.Camera` to capture real camera feed from physical arm
2. **Joint Reading** - Uses `dofbot_lib.DofbotRobot` to read current joint angles
3. **Model Inference** - Passes data to autobot10's trained SAC model for next action
4. **Joint Control** - Applies predicted actions as new joint positions to physical arm
5. **Loop** - Repeats for configurable number of steps (default: 30)

## Components

- `autobot10_inference.py` - Thin wrapper that imports V10 inference from autobot10 project
- `run_sim2real.py` - Main control loop for physical robot deployment
- `config.yaml` - Configuration (step count, model path, etc.)

**Note:** All V10 inference logic resides in `autobot10/inference.py` for self-containment.

## Setup

```bash
# Install dependencies
pip install -r requirements.txt

# Ensure dofbot_lib is accessible
# Ensure autobot10 model is trained and available
```

## Usage

```bash
python run_sim2real.py
```

This will:
1. Initialize physical robot and camera
2. Load autobot10 trained model
3. Run model inference for 30 steps
4. Apply predicted actions to physical arm
5. Save results and captured images

## Requirements

- Physical DofBot SE arm connected (COM5)
- USB camera on index 0
- Trained autobot10 model at: `c:/junkyard/ai/autobots/autobot10/models/v10/board_view_v10_final_20251225_164911.zip`

## Safety

- Ensure robot has clear workspace
- Start from home position
- Monitor first runs carefully
- Emergency stop: Ctrl+C or disconnect power
