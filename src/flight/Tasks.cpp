//
// Created by Akshay Gillett on 7/12/26.
//

#include "./flight_control.h"
#include "../rtos/rtos.h"
#include "./BufferPopulation.h"
#include "stm32f7xx.h"

static float gyroBias[3] = {0.0f, 0.0f, 0.0f};

void readIMUDataRegisters() {
    //function to populate the 14 registers starting at the start address
    IMURawPacket rawIMUBuffer = populateIMUBuffer();

    __disable_irq();
    //reassemble registers into signed 16-bit integers
    const int16_t rawAccelX = (rawIMUBuffer.bytes[1] << 8) | rawIMUBuffer.bytes[0];
    const int16_t rawAccelY = (rawIMUBuffer.bytes[3] << 8) | rawIMUBuffer.bytes[2];
    const int16_t rawAccelZ = (rawIMUBuffer.bytes[5] << 8) | rawIMUBuffer.bytes[4];

    //reassemble Gyroscope
    const int16_t rawGyroX = (rawIMUBuffer.bytes[7] << 8) | rawIMUBuffer.bytes[6];
    const int16_t rawGyroY = (rawIMUBuffer.bytes[9] << 8) | rawIMUBuffer.bytes[8];
    const int16_t rawGyroZ = (rawIMUBuffer.bytes[11] << 8) | rawIMUBuffer.bytes[10];
    __enable_irq();

    __disable_irq();
    //scale using floats for precice rotation
    drone.accelRaw[0] = static_cast<float>(rawAccelX) / 2048.0f;
    drone.accelRaw[1] = static_cast<float>(rawAccelY) / 2048.0f;
    drone.accelRaw[2] = static_cast<float>(rawAccelZ) / 2048.0f;

    drone.gyroRaw[0] = static_cast<float>(rawGyroX) / 16.4f;
    drone.gyroRaw[1] = static_cast<float>(rawGyroY) / 16.4f;
    drone.gyroRaw[2] = static_cast<float>(rawGyroZ) / 16.4f;
    __enable_irq();

    __disable_irq();
    //raw data minus the static bias
    if (drone.calibrated) {
        drone.gyroCalibrated[0] = drone.gyroRaw[0] - gyroBias[0];
        drone.gyroCalibrated[1] = drone.gyroRaw[1] - gyroBias[1];
        drone.gyroCalibrated[2] = drone.gyroRaw[2] - gyroBias[2];
    }
    __enable_irq();
}


void sensorCalibration() { //performs a mean over the gyro samples and subtracts that from the final gyro output
    print_str("Calibrating IMU Starting. Keep the drone still\n");
    float sumX = 0, sumY = 0, sumZ = 0;
    constexpr int samples = 2000;

    for (int i = 0; i < 2000; i++) {
        //populate gyro data with read imu function;
        readIMUDataRegisters();
        sumX += drone.gyroRaw[0];
        sumY += drone.gyroRaw[1];
        sumZ += drone.gyroRaw[2];
    }

    gyroBias[0] = sumX / samples;
    gyroBias[1] = sumY / samples;
    gyroBias[2] = sumZ / samples;

    drone.calibrated = true;
    print_str("Calibration complete.\n");
}

void crsfParsing() {
    CRSFPacket crsfBuffer = populateCRSFBuffer(); //populate buffers

    if (crsfBuffer.bytes[0] != 0xC8 || crsfBuffer.bytes[2] != 0x16) {return;}

    //use bitwise operators to traverse the array and extract the correct values
    const uint16_t ch1 = (crsfBuffer.bytes[3] | crsfBuffer.bytes[4] << 8) & 0x07FF;
    const uint16_t ch2 = (crsfBuffer.bytes[4] >> 3 | crsfBuffer.bytes[5] << 5) & 0x07FF;
    const uint16_t ch3 = (crsfBuffer.bytes[5] >> 6 | crsfBuffer.bytes[6] << 2 | crsfBuffer.bytes[7] << 10) & 0x07FF;
    const uint16_t ch4 = (crsfBuffer.bytes[7] >> 1 | crsfBuffer.bytes[8] << 7) & 0x07FF;
    const uint16_t ch5 = (crsfBuffer.bytes[8] >> 4 | crsfBuffer.bytes[9] << 4) & 0x07FF; //check this aux pos

    drone.rollStick = (static_cast<float>(ch1) - 992.0f) / 820.0f;
    drone.pitchStick = (static_cast<float>(ch2) - 992.0f) / 820.0f;
    drone.yawStick = (static_cast<float>(ch3) - 992.0f) / 820.0f;
    drone.throttleStick = (static_cast<float>(ch4) - 172.0f) / 1639.0f;

    drone.armSwitch = ch5 > 1000;

    drone.lastValidRCFrameTime = globalSystemTicks;
}

void radioLinkFailSafe() {
    if (globalSystemTicks - drone.lastValidRCFrameTime > 200) {
        drone.armSwitch = false;
        drone.currentSystemState = FlightState::FAILSAFE;
        print_str("CRSF RADIO LINK LOST\nFORCE DISARM\n");
    }
}

[[noreturn]] void lowLevelFailSafe(){

    //assumes that the battery voltage is being updated by the state machine

    static uint32_t lowVoltageStartTime = 0;
    static bool lowVoltageStarted = false;

    while (true) {
        if (constexpr float LOW_VOLTAGE_THRESHOLD = 19.2f; drone.batteryVoltage < LOW_VOLTAGE_THRESHOLD) {
            if (!lowVoltageStarted) {
                lowVoltageStarted = true;
                lowVoltageStartTime = globalSystemTicks;
            } else {
                if (globalSystemTicks - lowVoltageStartTime > 5000) {
                    if (drone.currentSystemState == FlightState::ARMED) {
                        drone.currentSystemState = FlightState::WARN;
                        print_str("LOW BATTERY DETECTED\nFORCE DISARM\n");
                    }
                }
            }
        } else {
            lowVoltageStarted = false;
            lowVoltageStartTime = 0;
        }

        yieldCurrentTask(); //if no voltage drop was detected ignore and yield the task
    }
}