//
// Created by Akshay Gillett on 7/16/26.
//

#include "Helpers.h"
#include <cmath>
#include "flight_control.h"

#define GYRO_CUTOFF_HZ 90.0f //11.11 ms
#define LOOP_DT 0.001f //1 ms

//helper functions for state estimation

static float angles[2] = {0.0f, 0.0f};

void estimateAngles() { //calculate the pitch and roll angles from the accelerometer axes using trig
    //pich-accel = arctan2(-ax, sqrt(ay**2+az**2)*(180/pi)
    //roll-accel = arctan2(ay, az)*(180/pi)

    const float aX = drone.accelRaw[0], aY = drone.accelRaw[1], aZ = drone.accelRaw[2];
    const float accel_pitch = std::atan2(-aX, std::sqrt(aY * aY + aZ * aZ)) * 57.2957795f; //constant is an estimation of 180/pi
    const float accel_roll = std::atan2(aY, aZ) * 57.2957795f;

    angles[0] = accel_pitch;
    angles[1] = accel_roll;

    drone.estimatedPitch = accel_pitch;
    drone.estimatedRoll = accel_roll;
    drone.estimatedYaw = 0.0f;
}

static float filteredGyro[3] = {0.0f, 0.0f, 0.0f};   //one per axis

void pt1Filter() { //clean the gyroscope noise before it reaches complementary filter
    static bool alphaComputed = false;
    static float alpha = 0.0f;

    if (!alphaComputed) {
        const float rc = 1.0f / (2.0f * 3.14159265f * GYRO_CUTOFF_HZ);
        alpha = LOOP_DT / (rc + LOOP_DT);
        alphaComputed = true;
    }

    for (int axis = 0; axis < 3; axis++) {
        filteredGyro[axis] = filteredGyro[axis] + alpha * (drone.gyroCalibrated[axis] - filteredGyro[axis]);
        drone.gyroFiltered[axis] = filteredGyro[axis];
    }
}

void complementaryFilter() { //fuses the gyroscope and the accelerometer tilt
    //ensure IMU is pointing in the correct direction

    //angle mode
    // attitudeRoll = 0.98f * (attitudeRoll + drone.gyroFiltered[0] * 0.001f) + 0.02f * drone.estimatedRoll;
    // attitudePitch = 0.98f * (attitudePitch + drone.gyroFiltered[1] * 0.001f) + 0.02f * drone.estimatedPitch;

    //acro mode
    drone.attitudeRoll  = drone.gyroFiltered[0]; // Roll Rate (X-axis)
    drone.attitudePitch = drone.gyroFiltered[1]; // Pitch Rate (Y-axis)
    drone.attitudeYaw   = drone.gyroFiltered[2]; // Yaw Rate (Z-axis)
}

uint32_t dshotBuffer1[17];
uint32_t dshotBuffer2[17];
uint32_t dshotBuffer3[17];
uint32_t dshotBuffer4[17];

//other helpers
DShotFrame generateDShotFrame(float throttleInput) {
    DShotFrame frame;
    uint16_t throttleValue = 0;

    if (throttleInput > 0.0f) { throttleValue = 48 + static_cast<uint16_t>(throttleInput * (2047.0f - 48.0f)); } //convert throttle
    if (throttleValue > 2047) { throttleValue = 2047; }

    uint16_t packet = (throttleValue << 5);

    uint16_t checksum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F; //ensures no data loss or corruption
    packet |= checksum;

    //converts into dshot with bitwise logic
    uint16_t mask = 0x8000;
    for (int i = 0; i < 16; i++) {
        if (packet & mask) {
            frame.bits[i] = 270; // High timing value for 1 (75% duty)
        } else {
            frame.bits[i] = 135; // Low timing value for 0 (37.5% duty)
        }
        mask >>= 1;
    }

    frame.bits[16] = 0;

    return frame;
}