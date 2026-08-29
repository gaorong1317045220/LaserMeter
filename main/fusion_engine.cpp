#include "fusion_engine.h"

#include <algorithm>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "fusion_config.h"

namespace {

struct Vec3 { float x, y, z; };

void normalize_quaternion(float &x, float &y, float &z, float &w)
{
    const float n = std::sqrt(x * x + y * y + z * z + w * w);
    if (n < 1.0e-6f) { x = y = z = 0.0f; w = 1.0f; return; }
    x /= n; y /= n; z /= n; w /= n;
}

void multiply_quaternion(float ax, float ay, float az, float aw,
                         float bx, float by, float bz, float bw,
                         float &x, float &y, float &z, float &w)
{
    x = aw * bx + ax * bw + ay * bz - az * by;
    y = aw * by - ax * bz + ay * bw + az * bx;
    z = aw * bz + ax * by - ay * bx + az * bw;
    w = aw * bw - ax * bx - ay * by - az * bz;
}

Vec3 sensor_to_body(Vec3 v)
{
    return {
        FUSION_IMU_BODY_X_FROM_SENSOR_X * v.x + FUSION_IMU_BODY_X_FROM_SENSOR_Y * v.y + FUSION_IMU_BODY_X_FROM_SENSOR_Z * v.z,
        FUSION_IMU_BODY_Y_FROM_SENSOR_X * v.x + FUSION_IMU_BODY_Y_FROM_SENSOR_Y * v.y + FUSION_IMU_BODY_Y_FROM_SENSOR_Z * v.z,
        FUSION_IMU_BODY_Z_FROM_SENSOR_X * v.x + FUSION_IMU_BODY_Z_FROM_SENSOR_Y * v.y + FUSION_IMU_BODY_Z_FROM_SENSOR_Z * v.z,
    };
}

Vec3 rotate(float qx, float qy, float qz, float qw, Vec3 v)
{
    const Vec3 t = {2.0f * (qy * v.z - qz * v.y),
                    2.0f * (qz * v.x - qx * v.z),
                    2.0f * (qx * v.y - qy * v.x)};
    return {v.x + qw * t.x + (qy * t.z - qz * t.y),
            v.y + qw * t.y + (qz * t.x - qx * t.z),
            v.z + qw * t.z + (qx * t.y - qy * t.x)};
}

void quaternion_to_euler(float x, float y, float z, float w,
                         float &roll, float &pitch, float &yaw)
{
    // Body axes are +X right, +Y forward, +Z up. This Z-X-Y sequence makes
    // pitch correspond to raising/lowering the front of the instrument.
    pitch = std::asin(std::clamp(2.0f * (w * x + y * z), -1.0f, 1.0f));
    yaw = std::atan2(2.0f * (w * z - x * y), 1.0f - 2.0f * (x * x + z * z));
    roll = std::atan2(2.0f * (w * y - x * z), 1.0f - 2.0f * (x * x + y * y));
}

SemaphoreHandle_t mutex_handle(void *p) { return static_cast<SemaphoreHandle_t>(p); }

}  // namespace

esp_err_t FusionEngine::begin()
{
    if (!mutex_) mutex_ = xSemaphoreCreateMutex();
    return mutex_ ? ESP_OK : ESP_ERR_NO_MEM;
}

void FusionEngine::resetPose()
{
    if (!mutex_) return;
    xSemaphoreTake(mutex_handle(mutex_), portMAX_DELAY);
    state_.x = state_.y = state_.z = 0.0f;
    state_.vx = state_.vy = 0.0f;
    state_.stationary = true;
    pose_.x = pose_.y = pose_.z = 0.0f;
    xSemaphoreGive(mutex_handle(mutex_));
}

void FusionEngine::resetMeasurementPair()
{
    if (!mutex_) return;
    xSemaphoreTake(mutex_handle(mutex_), portMAX_DELAY);
    state_.point_count = 0;
    state_.point_distance_m = -1.0f;
    xSemaphoreGive(mutex_handle(mutex_));
}

void FusionEngine::updateGyroscope(int64_t, float gx, float gy, float gz)
{
    if (!mutex_) return;
    const Vec3 body = sensor_to_body({gx, gy, gz});
    xSemaphoreTake(mutex_handle(mutex_), portMAX_DELAY);
    gyro_x_ = body.x; gyro_y_ = body.y; gyro_z_ = body.z;
    xSemaphoreGive(mutex_handle(mutex_));
}

void FusionEngine::updateAttitude(int64_t t_us, float qi, float qj, float qk, float qr,
                                  float, float, float, float gx, float gy, float gz,
                                  uint8_t accuracy_status)
{
    if (!mutex_) return;
    const FusionPose body_pose = bodyPoseFromSensorQuaternion(t_us, qi, qj, qk, qr);
    const Vec3 body_gyro = sensor_to_body({gx, gy, gz});

    xSemaphoreTake(mutex_handle(mutex_), portMAX_DELAY);
    state_.t_us = t_us;
    state_.attitude_valid = true;
    state_.x = state_.y = state_.z = 0.0f;
    state_.vx = state_.vy = 0.0f;
    state_.stationary = true;
    state_.confidence = std::min<uint8_t>(accuracy_status, 3) / 3.0f;
    quaternion_to_euler(body_pose.qi, body_pose.qj, body_pose.qk, body_pose.qr,
                        state_.roll, state_.pitch, state_.yaw);
    pose_.t_us = t_us;
    pose_.x = pose_.y = pose_.z = 0.0f;
    pose_.qi = body_pose.qi; pose_.qj = body_pose.qj;
    pose_.qk = body_pose.qk; pose_.qr = body_pose.qr;
    pose_.valid = true;
    gyro_x_ = body_gyro.x; gyro_y_ = body_gyro.y; gyro_z_ = body_gyro.z;
    xSemaphoreGive(mutex_handle(mutex_));
}

FusionPose FusionEngine::bodyPoseFromSensorQuaternion(int64_t t_us, float qi, float qj,
                                                       float qk, float qr) const
{
    normalize_quaternion(qi, qj, qk, qr);
#if FUSION_QUATERNION_CONJUGATE
    qi = -qi; qj = -qj; qk = -qk;
#endif
    FusionPose pose;
    multiply_quaternion(qi, qj, qk, qr,
                        FUSION_IMU_BODY_TO_SENSOR_QI, FUSION_IMU_BODY_TO_SENSOR_QJ,
                        FUSION_IMU_BODY_TO_SENSOR_QK, FUSION_IMU_BODY_TO_SENSOR_QR,
                        pose.qi, pose.qj, pose.qk, pose.qr);
    normalize_quaternion(pose.qi, pose.qj, pose.qk, pose.qr);
    pose.t_us = t_us;
    pose.valid = t_us > 0;
    return pose;
}

FusionPose FusionEngine::capturePose() const
{
    FusionPose out;
    if (!mutex_) return out;
    xSemaphoreTake(mutex_handle(mutex_), portMAX_DELAY);
    out = pose_;
    xSemaphoreGive(mutex_handle(mutex_));
    return out;
}

bool FusionEngine::commitLaserPoint(const FusionPose &pose, int32_t distance_mm)
{
    float point_x = 0.0f, point_y = 0.0f, point_z = 0.0f;
    if (!projectLaserPoint(pose, distance_mm, &point_x, &point_y, &point_z)) return false;
    const Vec3 point = {point_x, point_y, point_z};

    xSemaphoreTake(mutex_handle(mutex_), portMAX_DELAY);
    if (state_.point_count == 0 || state_.point_count >= 2) {
        state_.point1_x = point.x; state_.point1_y = point.y; state_.point1_z = point.z;
        state_.point_count = 1;
        state_.point_distance_m = -1.0f;
    } else {
        state_.point2_x = point.x; state_.point2_y = point.y; state_.point2_z = point.z;
        const float dx = state_.point2_x - state_.point1_x;
        const float dy = state_.point2_y - state_.point1_y;
        const float dz = state_.point2_z - state_.point1_z;
        state_.point_distance_m = std::sqrt(dx * dx + dy * dy + dz * dz);
        state_.point_count = 2;
    }
    xSemaphoreGive(mutex_handle(mutex_));
    return true;
}

bool FusionEngine::projectLaserPoint(const FusionPose &pose, int32_t distance_mm,
                                     float *x, float *y, float *z) const
{
    if (!pose.valid || distance_mm <= 0 || !x || !y || !z) return false;
    Vec3 dir = {FUSION_LASER_DIR_X, FUSION_LASER_DIR_Y, FUSION_LASER_DIR_Z};
    const float n = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    if (n < 1.0e-6f) return false;
    dir.x /= n; dir.y /= n; dir.z /= n;
    const Vec3 world_dir = rotate(pose.qi, pose.qj, pose.qk, pose.qr, dir);
    // Keep the established project convention: P2P and fixed-station scans
    // rotate the calibrated station vector FUSION_PIVOT_OFFSET.  The previous
    // experiment substituted (laser - pivot), which changes the effective
    // rotation radius and made the real-device P2P error larger.
    const Vec3 station_offset = rotate(pose.qi, pose.qj, pose.qk, pose.qr,
                                       {FUSION_PIVOT_OFFSET_X_M,
                                        FUSION_PIVOT_OFFSET_Y_M,
                                        FUSION_PIVOT_OFFSET_Z_M});
    const float range_m = static_cast<float>(distance_mm) * 0.001f;
    *x = pose.x + station_offset.x + world_dir.x * range_m;
    *y = pose.y + station_offset.y + world_dir.y * range_m;
    *z = pose.z + station_offset.z + world_dir.z * range_m;
    return true;
}

bool FusionEngine::solveRelativePair(const FusionPose &pose_a, int32_t distance_a_mm,
                                     const FusionPose &pose_b, int32_t distance_b_mm,
                                     float *space_m, float *horizontal_m, float *height_m,
                                     float *relative_angle_deg) const
{
    if (!space_m || !horizontal_m || !height_m || !relative_angle_deg) return false;
    float ax, ay, az, bx, by, bz;
    if (!projectLaserPoint(pose_a, distance_a_mm, &ax, &ay, &az) ||
        !projectLaserPoint(pose_b, distance_b_mm, &bx, &by, &bz)) return false;
    const float dx = bx - ax;
    const float dy = by - ay;
    const float dz = bz - az;
    *space_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    *horizontal_m = std::sqrt(dx * dx + dy * dy);
    *height_m = dz;

    // q_rel = inverse(q_a) * q_b.  Applying it to the body-frame laser ray
    // expresses ray B in A's frame, so the angle is independent of the
    // arbitrary world yaw used by Game Rotation Vector.
    float rqi, rqj, rqk, rqr;
    multiply_quaternion(-pose_a.qi, -pose_a.qj, -pose_a.qk, pose_a.qr,
                        pose_b.qi, pose_b.qj, pose_b.qk, pose_b.qr,
                        rqi, rqj, rqk, rqr);
    normalize_quaternion(rqi, rqj, rqk, rqr);
    Vec3 dir = {FUSION_LASER_DIR_X, FUSION_LASER_DIR_Y, FUSION_LASER_DIR_Z};
    const float n = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    if (n < 1.0e-6f) return false;
    dir.x /= n; dir.y /= n; dir.z /= n;
    const Vec3 dir_b_in_a = rotate(rqi, rqj, rqk, rqr, dir);
    const float cosine = std::clamp(dir.x * dir_b_in_a.x +
                                    dir.y * dir_b_in_a.y +
                                    dir.z * dir_b_in_a.z, -1.0f, 1.0f);
    *relative_angle_deg = std::acos(cosine) * 57.2957795f;
    return std::isfinite(*space_m) && std::isfinite(*relative_angle_deg);
}

FusionState FusionEngine::snapshot() const
{
    FusionState out;
    if (!mutex_) return out;
    xSemaphoreTake(mutex_handle(mutex_), portMAX_DELAY);
    out = state_;
    xSemaphoreGive(mutex_handle(mutex_));
    return out;
}
