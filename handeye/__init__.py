"""FR5 + ZED X eye-in-hand calibration from collected images and flange poses.

Modules: geometry (transforms, FR5 pose convention), board (Kalibr Aprilgrid),
dataset (session folder), observations (per-image PnP), solver (hand-eye,
refinement, selection, quality gate), diagnostics (coverage, uncertainty,
board scale and intrinsics), kinematics (FR5 model, pose/joint checks),
validation (independent touch-point check). Command line: python -m handeye.
"""

import os
import sys


def python_command() -> str:
    """This interpreter as a command to paste into the same shell (printed hints only).

    From the repository root on Windows this is .venv\\Scripts\\python.exe.
    """
    try:
        path = os.path.relpath(sys.executable)
    except ValueError:  # Windows: interpreter on another drive
        path = sys.executable
    return path if " " not in path else "python"
