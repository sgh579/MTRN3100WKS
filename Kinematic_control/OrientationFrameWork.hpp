//
// Created by Naveen Thayyil on 13/7/2025.
//

//#ifndef ORIENTATIONFRAMEWORK_HPP
//#define ORIENTATIONFRAMEWORK_HPP
//
//#endif //ORIENTATIONFRAMEWORK_HPP

struct SensorOffsets {
    float frontOffset;
    float leftOffset;
    float rightOffset;
};

struct LidarReadings {
    float front;
    float left;
    float right;
};

struct OrientationState {
    float yaw;            // degrees or radians
    float angularVelocity;
};

OrientationState readIMU();
LidarReadings readLidars();

class OrientationEstimator {
public:
    OrientationEstimator(SensorOffsets offsets, float alpha = 0.98f)
        : offsets(offsets), alpha(alpha), estimatedYaw(0.0f) {}

    void update(float dt) {
        OrientationState imuData = readIMU();
        LidarReadings lidarData = readLidars();

        // Use IMU gyro to integrate yaw
        float imuYawDelta = imuData.angularVelocity * dt;
        estimatedYaw += imuYawDelta;

        // Correct using LiDARs (wall following assumption)
        float lidarYawCorrection = estimateYawFromLidars(lidarData);

        // Complementary Filter
        estimatedYaw = alpha * estimatedYaw + (1 - alpha) * lidarYawCorrection;
    }

    float getYaw() const { return estimatedYaw; }

private:
    float estimateYawFromLidars(const LidarReadings& lidar) {
        // Adjust for sensor placement
        float leftCorrected = lidar.left + offsets.leftOffset;
        float rightCorrected = lidar.right + offsets.rightOffset;

        // If walls are parallel, yaw can be estimated as arctangent of difference
        float yawEstimate = atan2(rightCorrected - leftCorrected, offsets.leftOffset + offsets.rightOffset);

        return yawEstimate * (180.0f / M_PI); // convert to degrees if necessary
    }

    SensorOffsets offsets;
    float alpha;         // complementary filter coefficient
    float estimatedYaw;
};


