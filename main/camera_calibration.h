#pragma once

// Camera intrinsic calibration, measured 2026-08-16 from 6 checkerboard frames
// (11x8 inner corners, 25.0 mm squares). RMS reprojection 0.308 px.
// Source: calibration_capture/camera_intrinsics.json (run
// tools/calibrate_camera_intrinsics.py to regenerate).
// Full sensor frame is 2560x1920; the on-device preview uses a centre crop.
#define CAMERA_FX_PX            2027.307623367193
#define CAMERA_FY_PX            2017.894435000454
#define CAMERA_CX_PX            1292.460773339633
#define CAMERA_CY_PX            934.299741097846
#define CAMERA_WIDTH_PX         2560
#define CAMERA_HEIGHT_PX        1920

// Distortion (k1 k2 p1 p2 k3), OpenCV radial-tangential model.
#define CAMERA_DIST_K1          -0.37796009106886685
#define CAMERA_DIST_K2          -0.463295975578497
#define CAMERA_DIST_P1          0.0008913772431690511
#define CAMERA_DIST_P2          -0.0012640675461081652
#define CAMERA_DIST_K3          2.677185861026511
