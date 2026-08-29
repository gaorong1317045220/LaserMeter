#pragma once

#include <stdint.h>

#include "esp_err.h"

struct FusionPose {
    int64_t t_us = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float qi = 0.0f;
    float qj = 0.0f;
    float qk = 0.0f;
    float qr = 1.0f;
    bool valid = false;
};

struct FusionState {
    int64_t t_us = 0;
    bool attitude_valid = false;
    bool stationary = true;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float confidence = 0.0f;
    uint8_t point_count = 0;
    float point1_x = 0.0f;
    float point1_y = 0.0f;
    float point1_z = 0.0f;
    float point2_x = 0.0f;
    float point2_y = 0.0f;
    float point2_z = 0.0f;
    float point_distance_m = -1.0f;
};

// Nine-axis IMU geometry engine. The device origin is fixed; BNO086 supplies
// an accelerometer/gyroscope/magnetometer rotation vector used to project the
// laser ray into the local coordinate frame.
class FusionEngine {
public:
    esp_err_t begin();
    void resetPose();
    void resetMeasurementPair();
    void updateGyroscope(int64_t t_us, float gx, float gy, float gz);
    void updateAttitude(int64_t t_us, float qi, float qj, float qk, float qr,
                        float ax, float ay, float az, float gx, float gy, float gz,
                        uint8_t accuracy_status);
    FusionPose capturePose() const;
    FusionPose bodyPoseFromSensorQuaternion(int64_t t_us, float qi, float qj,
                                            float qk, float qr) const;
    bool commitLaserPoint(const FusionPose &pose, int32_t distance_mm);
    bool projectLaserPoint(const FusionPose &pose, int32_t distance_mm,
                           float *x, float *y, float *z) const;
    bool solveRelativePair(const FusionPose &pose_a, int32_t distance_a_mm,
                           const FusionPose &pose_b, int32_t distance_b_mm,
                           float *space_m, float *horizontal_m, float *height_m,
                           float *relative_angle_deg) const;
    FusionState snapshot() const;

private:
    void *mutex_ = nullptr;
    FusionState state_;
    FusionPose pose_;
    float gyro_x_ = 0.0f;
    float gyro_y_ = 0.0f;
    float gyro_z_ = 0.0f;
};
