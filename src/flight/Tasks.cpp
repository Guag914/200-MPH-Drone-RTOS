//
// Created by Akshay Gillett on 7/12/26.
//

#include <algorithm>
#include <cmath>

#include "./flight_control.h"
#include "../rtos/rtos.h"
#include "./BufferPopulation.h"
#include "stm32f7xx.h"
#include "Helpers.h"
#include "../rtos/Logger.h"
#include "../sim/SimInjector.h"

DroneState drone; //define drone struct

static float gyroBias[3] = {0.0f, 0.0f, 0.0f};

void readIMUDataRegisters() {
    //function to populate the 14 registers starting at the start address
    #ifdef SIMULATION
    IMURawPacket rawIMUBuffer = populateIMUMockBuffer();
    #else
    IMURawPacket rawIMUBuffer = populateIMUBuffer();
    #endif

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

    printToUSART("Reading IMU Registers\n");

    printToUSART(" [IMU] Accel vals: \n");
    printToUSART("  "); printToUSART(drone.accelRaw[0]); printToUSART("\n");
    printToUSART("  "); printToUSART(drone.accelRaw[1]); printToUSART("\n");
    printToUSART("  "); printToUSART(drone.accelRaw[2]); printToUSART("\n\n");

    printToUSART(" [IMU] Gyro vals: \n");
    printToUSART("  "); printToUSART(drone.gyroRaw[0]); printToUSART("\n");
    printToUSART("  "); printToUSART(drone.gyroRaw[1]); printToUSART("\n");
    printToUSART("  "); printToUSART(drone.gyroRaw[2]); printToUSART("\n\n");
}

void stateEstimation() {
    estimateAngles(); //calculate the pitch and roll angles from the accelerometer axes using trig
    pt1Filter(); //clean the gyroscope noise before it reaches complementary filter
    complementaryFilter(); //fuses the gyroscope and the accelerometer tilt

    printToUSART("Running State Estimation:\n");

    printToUSART(" [SE-EA] Estimated Angles: \n");
    printToUSART("  Estimated Pitch: "); printToUSART(drone.estimatedPitch); printToUSART("\n");
    printToUSART("  Estimated Roll: "); printToUSART(drone.estimatedRoll); printToUSART("\n");
    printToUSART("  Estimated Yaw: "); printToUSART(drone.estimatedYaw); printToUSART("\n\n");

    printToUSART(" [SE-PT1F] Filtered Gyro: \n");
    printToUSART("  Filtered X Gyro: "); printToUSART(drone.gyroFiltered[0]); printToUSART("\n");
    printToUSART("  Filtered Y Gyro: "); printToUSART(drone.gyroFiltered[1]); printToUSART("\n");
    printToUSART("  Filtered Z Gyro: "); printToUSART(drone.gyroFiltered[2]); printToUSART("\n\n");

    printToUSART(" [SE-CF] Attitude: \n");
    printToUSART("  Attitude Roll: "); printToUSART(drone.attitudeRoll); printToUSART("\n");
    printToUSART("  Attitude Pitch: "); printToUSART(drone.attitudePitch); printToUSART("\n");
    printToUSART("  Attitude Yaw: "); printToUSART(drone.attitudeYaw); printToUSART("\n\n\n");
}

//PID values
constexpr float Kp_roll = 1.2f, Ki_roll = 0.05f, Kd_roll = 0.3f; //adjust these numbers when bench testing (change to rates on betaflight)
constexpr float Kp_pitch = 1.2f, Ki_pitch = 0.05f, Kd_pitch = 0.3f;
constexpr float Kp_yaw = 1.2f, Ki_yaw = 0.05f, Kd_yaw = 0.3f;

//these get overriden do NOT adjust
static float rollErrorSum = 0.0f, lastRollError = 0.0f;
static float pitchErrorSum = 0.0f, lastPitchError = 0.0f;
static float yawErrorSum = 0.0f, lastYawError = 0.0f;

void flightLoop() {
    //P - reacts to present error (proportional)
    //I - reacts to past accumulated error (Integral)
    //D - preducts future error (Derivative)

    //prevents the drone from sending dshot commands if not armed
    if (
        drone.currentSystemState == FlightState::DISARMED ||
        drone.currentSystemState == FlightState::FAILSAFE ||
        drone.currentSystemState == FlightState::ERROR
       )
    {
        drone.motorOutput[0] = 0; //front left
        drone.motorOutput[1] = 0; //front right
        drone.motorOutput[2] = 0; //back left
        drone.motorOutput[3] = 0; //back right
        return;
    }

    const float targetRoll = drone.rollStick;
    const float targetPitch = drone.pitchStick;
    const float targetYaw = drone.yawStick;
    const float targetThrottle = drone.throttleStick;

    const float rollError = targetRoll - drone.attitudeRoll;
    const float pitchError = targetPitch - drone.attitudePitch;
    const float yawError = targetYaw - drone.attitudeYaw;

    //PID for roll
    rollErrorSum += rollError;
    const float rollOutput = (Kp_roll * rollError) + (Ki_roll * rollErrorSum) + (Kd_roll * (rollError - lastRollError));
    lastRollError = rollError;

    //PID for pitch
    pitchErrorSum += pitchError;
    const float pitchOutput = (Kp_pitch * pitchError) + (Ki_pitch * pitchErrorSum) + (Kd_pitch * (pitchError - lastPitchError));
    lastPitchError = pitchError;

    //PID for yaw
    yawErrorSum += yawError;
    const float yawOutput = (Kp_yaw * yawError) + (Ki_yaw * yawErrorSum) + (Kd_yaw * (yawError - lastYawError));
    lastYawError = yawError;

    //clamp values
    rollErrorSum = std::clamp(rollErrorSum, -100.0f, 100.0f);
    pitchErrorSum = std::clamp(pitchErrorSum, -100.0f, 100.0f);
    yawErrorSum = std::clamp(yawErrorSum, -100.0f, 100.0f);

    //mixer for dshot600 commands - adjust direction based upon which motors spin ccw vs cw
    drone.motorOutput[0] =  + targetThrottle + pitchOutput + rollOutput - yawOutput; //front left
    drone.motorOutput[1] =  + targetThrottle + pitchOutput - rollOutput + yawOutput; //front right
    drone.motorOutput[2] =  + targetThrottle - pitchOutput + rollOutput + yawOutput; //back left
    drone.motorOutput[3] =  + targetThrottle - pitchOutput - rollOutput - yawOutput; //back right

    //clamp values
    drone.motorOutput[0] = std::clamp(drone.motorOutput[0], 0.0f, 2000.0f);
    drone.motorOutput[1] = std::clamp(drone.motorOutput[1], 0.0f, 2000.0f);
    drone.motorOutput[2] = std::clamp(drone.motorOutput[2], 0.0f, 2000.0f);
    drone.motorOutput[3] = std::clamp(drone.motorOutput[3], 0.0f, 2000.0f);

    printToUSART("Running Flight Loop: \n");

    printToUSART(" [FL-PID] Error Values: \n");
    printToUSART("  Roll Error: "); printToUSART(rollError); printToUSART("\n");
    printToUSART("  Pitch Error: "); printToUSART(pitchError); printToUSART("\n");
    printToUSART("  Yaw Error: "); printToUSART(yawError); printToUSART("\n\n");

    printToUSART(" [FL-PID] Error Sums: \n");
    printToUSART("  Roll Error Sum: "); printToUSART(rollErrorSum); printToUSART("\n");
    printToUSART("  Pitch Error Sum: "); printToUSART(pitchErrorSum); printToUSART("\n");
    printToUSART("  Yaw Error Sum: "); printToUSART(yawErrorSum); printToUSART("\n\n");

    printToUSART(" [FL-MIXER] Motor Ouputs (Clamped): \n");
    printToUSART("  M1: "); printToUSART(drone.motorOutput[0]); printToUSART("\n");
    printToUSART("  M2: "); printToUSART(drone.motorOutput[1]); printToUSART("\n");
    printToUSART("  M3: "); printToUSART(drone.motorOutput[2]); printToUSART("\n");
    printToUSART("  m4: "); printToUSART(drone.motorOutput[3]); printToUSART("\n\n\n");
}

void crsfParsing() {
    while (true) {
        #ifdef SIMULATION
        CRSFPacket crsfBuffer =  populateCRSFMockBuffer();
        #else
        CRSFPacket crsfBuffer =  populateCRSFBuffer();
        #endif

        if (crsfBuffer.bytes[0] == 0x00 && crsfBuffer.bytes[1] == 0x00) { yieldCurrentTask(); continue; }

        if (crsfBuffer.bytes[0] != 0xC8) {//checks for corrupted data

            drone.currentSystemState = FlightState::ERROR;
            drone.errorMSG = "[CRSF] Corrupted data input stream.\n";
            yieldCurrentTask();
            continue;
        }

        //use bitwise operators to traverse the array and extract the correct values
        const uint16_t ch1 = (crsfBuffer.bytes[3] | crsfBuffer.bytes[4] << 8) & 0x07FF;
        const uint16_t ch2 = (crsfBuffer.bytes[4] >> 3 | crsfBuffer.bytes[5] << 5) & 0x07FF;
        const uint16_t ch3 = (crsfBuffer.bytes[5] >> 6 | crsfBuffer.bytes[6] << 2 | crsfBuffer.bytes[7] << 10) & 0x07FF;
        const uint16_t ch4 = (crsfBuffer.bytes[7] >> 1 | crsfBuffer.bytes[8] << 7) & 0x07FF;
        const uint16_t ch5 = (crsfBuffer.bytes[8] >> 4 | crsfBuffer.bytes[9] << 4) & 0x07FF; //check this aux pos

        //convert stick positions into proper format (e.g. bound to -1.0f to 1.0f)
        drone.rollStick = (static_cast<float>(ch1) - 992.0f) / 820.0f;
        drone.pitchStick = (static_cast<float>(ch2) - 992.0f) / 820.0f;
        drone.yawStick = (static_cast<float>(ch3) - 992.0f) / 820.0f;
        drone.throttleStick = (static_cast<float>(ch4) - 172.0f) / 1639.0f;

        drone.armed = ch5 > 1000;

        printToUSART("Reading CRSF Data Registers\n");

        printToUSART(" [CRSF] Drone Channel Values:\n");
        printToUSART("  Ch1: "); printToUSART(ch1); printToUSART("\n");
        printToUSART("  Ch2: "); printToUSART(ch2); printToUSART("\n");
        printToUSART("  Ch3: "); printToUSART(ch3); printToUSART("\n");
        printToUSART("  Ch4: "); printToUSART(ch4); printToUSART("\n");
        printToUSART("  Ch5: "); printToUSART(ch5); printToUSART("\n\n");

        printToUSART(" [CRSF] Drone Stick Values: \n");
        printToUSART("  Roll Stick: "); printToUSART(drone.rollStick); printToUSART("\n");
        printToUSART("  Pitch Stick: "); printToUSART(drone.pitchStick); printToUSART("\n");
        printToUSART("  Yaw Stick: "); printToUSART(drone.yawStick); printToUSART("\n");
        printToUSART("  Throttle Stick: "); printToUSART(drone.throttleStick); printToUSART("\n\n\n");

        drone.lastValidRCFrameTime = globalSystemTicks;
        yieldCurrentTask();
    }
}

void radioLinkFailSafe() {
    while (true) {
        if (globalSystemTicks - drone.lastValidRCFrameTime > 500) {
            drone.armed = false;
            drone.currentSystemState = FlightState::FAILSAFE;
            drone.failsafeMSG = FAILSAFE::CRSF_LOST;
        }

        printToUSART("Radio Link Failsafe\n");
        printToUSART(" [RLFS] Drone CRSF Link Quality: \n");
        printToUSART("  Armed: "); printToUSART(drone.armed); printToUSART("\n");
        printToUSART("  Last Frame: "); printToUSART(globalSystemTicks - drone.lastValidRCFrameTime); printToUSART("\n\n\n");

        yieldCurrentTask();
    }
}

void sensorCalibration() { //performs a mean over the gyro samples and subtracts that from the final gyro output
    printToUSART("Calibrating IMU Starting. Keep the drone still\n");
    float sumX = 0, sumY = 0, sumZ = 0;
    constexpr int samples = 2000;

    for (int i = 0; i < 2000; i++) {
        //populate gyro data with read imu function;
        readIMUDataRegisters();
        sumX += drone.gyroRaw[0];
        sumY += drone.gyroRaw[1];
        sumZ += drone.gyroRaw[2];

        printToUSART(" [SC] Reporting Sum Values:\n");
        printToUSART("  SumX: "); printToUSART(sumX); printToUSART("\n");
        printToUSART("  SumY: "); printToUSART(sumY); printToUSART("\n");
        printToUSART("  SumZ: "); printToUSART(sumZ); printToUSART("\n\n\n");
    }

    gyroBias[0] = sumX / samples;
    gyroBias[1] = sumY / samples;
    gyroBias[2] = sumZ / samples;

    drone.calibrated = true;
    printToUSART("Calibration complete.\n");
}

void lowLevelFailSafe() {

    static uint32_t lowVoltageStartTime = 0;
    static bool lowVoltageStarted = false;

    while (true) {
        readBatteryVoltage(); //reads once per pass

        if (constexpr float LOW_VOLTAGE_THRESHOLD = 19.2f; drone.batteryVoltage < LOW_VOLTAGE_THRESHOLD) {
            if (!lowVoltageStarted) {
                lowVoltageStarted = true;
                lowVoltageStartTime = globalSystemTicks;
            } else {
                if (globalSystemTicks - lowVoltageStartTime > 5000) {
                    if (drone.currentSystemState == FlightState::ARMED) {
                        drone.currentSystemState = FlightState::WARN;
                        drone.warnMSG = WARN::LOW_BATTERY;
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

[[noreturn]] void dShotGeneration() {
    // Enable the Timer update DMA requests once at boot
    currentBoardConfig.motor_timer->DIER |= TIM_DIER_UDE;

    while (true) {
        //populate local buffers via helper and trigger the transfer
        DShotFrame m1 = generateDShotFrame(drone.motorOutput[0]);
        DShotFrame m2 = generateDShotFrame(drone.motorOutput[1]);
        DShotFrame m3 = generateDShotFrame(drone.motorOutput[2]);
        DShotFrame m4 = generateDShotFrame(drone.motorOutput[3]);

        for (int i = 0; i < 17; i++) {
            dshotBuffer1[i] = m1.bits[i];
            dshotBuffer2[i] = m2.bits[i];
            dshotBuffer3[i] = m3.bits[i];
            dshotBuffer4[i] = m4.bits[i];
        }

        startMotor1_DMATransfer();
        startMotor2_DMATransfer();
        startMotor3_DMATransfer();
        startMotor4_DMATransfer();

        //yield the task
        yieldCurrentTask();
    }
}

void flightStateMachine() {
    while (true) {
        bool armed = drone.armed;
        //switch statement to go through all the possible states
        switch (drone.currentSystemState) {
            case FlightState::DISARMED:
                if (armed) {
                    if (drone.throttleStick < 0.05f) { //check throttle pos
                        if (std::abs(drone.attitudePitch) < 15.0f && std::abs(drone.attitudeRoll) < 15.0f) { //check tilt within a few degrees
                            drone.currentSystemState = FlightState::ARMED;
                        } else {
                            drone.currentSystemState = FlightState::WARN;
                            drone.warnMSG = WARN::ARMED_ANGLE;
                        }
                    } else {
                        drone.currentSystemState = FlightState::WARN;
                        drone.warnMSG = WARN::ARMED_THROTTLE;
                    }
                }
                break;
            case FlightState::ARMED:
                if (!armed) {
                    drone.currentSystemState = FlightState::DISARMED; //instant disarm from switch bool
                }
                break;
            case FlightState::WARN:
                printToUSART(drone.warnMSG.data());
                //print message to OSD
                break;
            case FlightState::FAILSAFE:
                printToUSART(drone.failsafeMSG.data());
                //print message to OSD
                // drone.currentSystemState = FlightState::DISARMED;
                break;
            case FlightState::ERROR:
                printToUSART(drone.errorMSG.data()); //convert to actual log function
                //print message to OSD
                // drone.currentSystemState = FlightState::DISARMED;
                break;
        }

        yieldCurrentTask();
    }
}

//master task for imu
void imuControlLoop() { // one interrupt-zone task, triggered by real IMU DRDY eventually
    while (true) {
        readIMUDataRegisters();
        stateEstimation();
        flightLoop();
        yieldCurrentTask();
    }
}