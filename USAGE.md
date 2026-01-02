# Usage Guide

## Quick Start

### 1. Test Inference (No Hardware Required)

Test that the model loads and produces outputs:

```bash
python test_inference.py
```

This validates the inference API without requiring the physical robot.

### 2. Run on Physical Robot

**⚠️ SAFETY FIRST:**
- Ensure robot has clear workspace
- Keep emergency stop accessible
- Monitor first run carefully

**V10 Model (Basic):**
```bash
python run_sim2real.py --model ../autobot10/models/v10/board_view_v10_final_*.zip
```

**V10-Segmented Model (Vision-Guided):**
```bash
python run_sim2real.py --use-segmented --model ../autobot10-segmented/models/v10-segmented/board_view_v10_final_*.zip
```

Default: 30 steps, saves images and trajectory to `outputs/`

### 3. Custom Configuration

```bash
# Run with custom step count
python run_sim2real.py --model <path> --steps 50

# Enable visualization
python run_sim2real.py --model <path> --visualize

# Don't save images (faster)
python run_sim2real.py --model <path> --no-save

# Custom servo speed
python run_sim2real.py --model <path> --servo-time 300
```

## Expected Behavior

The robot will:
1. Start from current position
2. Incrementally adjust joints to find/view chessboard
3. Use camera feedback (via model) to guide movements
4. Execute 30 steps (configurable)
5. Save camera images and trajectory data

## Outputs

After running, check `outputs/YYYYMMDD_HHMMSS/`:
- `step_000.jpg` to `step_029.jpg` - Camera captures at each step
- `trajectory.json` - Complete trajectory data (joint angles, actions)

## Troubleshooting

### Model not found
```
Error: Model file not found
```
**Solution:** Provide valid model path via `--model` argument.
- V10 models: `../autobot10/models/v10/`
- V10-segmented models: `../autobot10-segmented/models/v10-segmented/`

### Robot connection failed
```
RuntimeError: Failed to connect to robot
```
**Solution:** 
- Check robot is powered on
- Verify USB connection
- Confirm COM port is COM5 (or update in code)

### Camera not found
```
RuntimeError: Failed to open camera
```
**Solution:**
- Check camera is connected
- Verify camera index in config.yaml (default: 0)
- Test with OpenCV: `python -c "import cv2; print(cv2.VideoCapture(0).isOpened())"`

## Advanced Usage

### Using Configuration File

Edit `config.yaml` to change hardware settings (COM port, camera index, etc.).
Model paths are specified via command-line arguments.

### Custom Control Loop

```python
import sys
sys.path.insert(0, 'c:/junkyard/ai/autobots/dofbot_lib')

from hardware import DofbotRobot, Camera
from autobot10_inference import V10Inference

# Initialize hardware and model
robot = DofbotRobot()
camera = Camera().open()
inference = V10Inference(model_path='path/to/model.zip')

# Custom control loop
for step in range(10):
    frame = camera.read_frame()
    current_servo = robot.get_current_positions()
    current_rad = inference.convert_servo_to_radians(current_servo)
    
    result = inference.predict_next_joint_angles(current_rad)
    next_servo = inference.convert_radians_to_servo(result['next_joint_angles'])
    
    robot.set_all_servos(next_servo.tolist(), time_ms=100)
    time.sleep(0.1)

# Cleanup
camera.close()
robot.close()
```

## Next Steps

After successful deployment:
1. Analyze trajectory data to understand model behavior
2. Review captured images to see what camera saw
3. Adjust step count for longer/shorter sequences
4. Experiment with different starting positions
5. Compare physical behavior to simulation
