"""
Autobot10 Inference API - Wrapper

This module imports the V10 inference functionality from the autobot10 project.
All V10-specific logic is maintained in autobot10/inference.py for self-containment.
"""

import sys
from pathlib import Path

# Add autobot10 to path
autobot10_path = Path(__file__).parent.parent / "autobot10"
sys.path.insert(0, str(autobot10_path))

# Import V10 inference from autobot10 project
from inference import V10Inference

# Create alias for backward compatibility
Autobot10Inference = V10Inference
