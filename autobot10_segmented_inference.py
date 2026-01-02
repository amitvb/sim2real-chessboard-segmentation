"""
Autobot10-Segmented Inference API - Wrapper

This module imports the V10-segmented inference functionality from the autobot10-segmented project.
All V10-segmented-specific logic is maintained in autobot10-segmented/inference.py for self-containment.
"""

import importlib.util
from pathlib import Path

# Get absolute path to autobot10-segmented inference module
autobot10_segmented_path = Path(__file__).parent.parent / "autobot10-segmented"
inference_module_path = autobot10_segmented_path / "inference.py"

# Load module directly from file path to avoid sys.path conflicts
spec = importlib.util.spec_from_file_location("autobot10_segmented_inference_module", inference_module_path)
inference_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(inference_module)

# Import V10-segmented inference from loaded module
V10SegmentedInference = inference_module.V10SegmentedInference

# Create alias for backward compatibility
Autobot10SegmentedInference = V10SegmentedInference
