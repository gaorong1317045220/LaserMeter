#pragma once

// Device coordinate frame: +X right, +Y laser-forward, +Z up.
// BNO086 quaternion is assumed to rotate device vectors into world vectors.
#define FUSION_QUATERNION_CONJUGATE     0

// BNO086 sensor frame -> device body frame, calibrated 2026-07-14:
// body X = -sensor Z, body Y = -sensor Y, body Z = -sensor X.
#define FUSION_IMU_BODY_X_FROM_SENSOR_X  0.0f
#define FUSION_IMU_BODY_X_FROM_SENSOR_Y  0.0f
#define FUSION_IMU_BODY_X_FROM_SENSOR_Z -1.0f
#define FUSION_IMU_BODY_Y_FROM_SENSOR_X  0.0f
#define FUSION_IMU_BODY_Y_FROM_SENSOR_Y -1.0f
#define FUSION_IMU_BODY_Y_FROM_SENSOR_Z  0.0f
#define FUSION_IMU_BODY_Z_FROM_SENSOR_X -1.0f
#define FUSION_IMU_BODY_Z_FROM_SENSOR_Y  0.0f
#define FUSION_IMU_BODY_Z_FROM_SENSOR_Z  0.0f

// Quaternion rotating body vectors into the BNO086 sensor frame.
#define FUSION_IMU_BODY_TO_SENSOR_QI     0.70710678f
#define FUSION_IMU_BODY_TO_SENSOR_QJ     0.0f
#define FUSION_IMU_BODY_TO_SENSOR_QK    -0.70710678f
#define FUSION_IMU_BODY_TO_SENSOR_QR     0.0f

// Laser ray and emitter offset relative to the BNO086 origin.
// Measured 2026-08-16: laser is 14.5 mm right of IMU (+X), 48.6527 mm forward
// (+Y), 16.256 mm below IMU (-Z). Device frame: +X right, +Y forward, +Z up.
#define FUSION_LASER_DIR_X              0.0f
#define FUSION_LASER_DIR_Y              1.0f
#define FUSION_LASER_DIR_Z              0.0f
#define FUSION_LASER_OFFSET_X_M         0.0145f
#define FUSION_LASER_OFFSET_Y_M         0.0486527f
#define FUSION_LASER_OFFSET_Z_M        -0.016256f

// Tripod pivot axis relative to the laser emitter, measured 2026-08-17 with a
// caliper: pivot is centred on the laser axis (X = -7.83), 18.397 mm behind the
// emitter (-Y), 73.1802 mm below it (-Z). Device frame: +X right, +Y forward,
// +Z up.
#define FUSION_TRIPOD_AXIS_OFFSET_X_M  -0.00783f
#define FUSION_TRIPOD_AXIS_OFFSET_Y_M  -0.018397f
#define FUSION_TRIPOD_AXIS_OFFSET_Z_M  -0.0731802f

// Projection station (= rotation centre) used by P2P and room scan. This is
// the calibrated station vector used by the existing projection convention.
#define FUSION_PIVOT_OFFSET_X_M       (FUSION_LASER_OFFSET_X_M + FUSION_TRIPOD_AXIS_OFFSET_X_M)
#define FUSION_PIVOT_OFFSET_Y_M       (FUSION_LASER_OFFSET_Y_M + FUSION_TRIPOD_AXIS_OFFSET_Y_M)
#define FUSION_PIVOT_OFFSET_Z_M       (FUSION_LASER_OFFSET_Z_M + FUSION_TRIPOD_AXIS_OFFSET_Z_M)
