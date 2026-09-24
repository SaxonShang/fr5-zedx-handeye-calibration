"""FR5 + ZED X eye-in-hand calibration from collected images and flange poses.

Modules: geometry (transforms, FR5 pose convention), board (Kalibr Aprilgrid),
dataset (session folder), observations (per-image PnP), solver (hand-eye,
refinement, selection, quality gate), diagnostics (coverage, uncertainty,
board scale and intrinsics), kinematics (FR5 model, pose/joint checks),
validation (independent touch-point check). Command line: python -m handeye.
"""
