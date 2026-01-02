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

```bash
python run_sim2real.py
```

Default: 30 steps, saves images and trajectory to `outputs/`

### 3. Custom Configuration

```bash
# Run with custom step count
python run_sim2real.py --steps 50

# Use different model
python run_sim2real.py --model path/to/model.zip

# Don't save images (faster)
python run_sim2real.py --no-save
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
**Solution:** Check that autobot10 model exists at:
`c:/junkyard/ai/autobots/autobot10/models/v10/board_view_v10_final_20251225_164911.zip`

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
- Verify camera index (default: 0)
- Try `python -c "from hardware import find_cameras; print(find_cameras())"`

## Advanced Usage

### Using Configuration File

Edit `config.yaml` to change defaults, then:

```python
import yaml
from run_sim2real import Sim2RealController

with open('config.yaml') as f:
    config = yaml.safe_load(f)

controller = Sim2RealController(
    num_steps=config['num_steps'],
    model_path=config['model_path'],
    save_images=config['save_images']
)
controller.run()
```

### Custom Control Loop

```python
import sys
sys.path.insert(0, 'c:/junkyard/ai/autobots/dofbot_lib')

from hardware import DofbotRobot, Camera
from autobot10_inference import V10Inference

# Initialize
robot = DofbotRobot()
camera = Camera().open()
inference = V10Inference()

# Custom loop
for i in range(10):
    img = camera.read_frame()
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
