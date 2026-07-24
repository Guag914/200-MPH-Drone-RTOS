//
// Created by Akshay Gillett on 7/12/26.
//

#include <algorithm>
#include <cmath>
#include <cstring>

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

#ifdef DEBUG
    printToUSART("Reading IMU Registers\n");

    printToUSART(" [IMU] Accel vals: \n");
    printToUSART("  "); printToUSART(drone.accelRaw[0]); printToUSART("\n");
    printToUSART("  "); printToUSART(drone.accelRaw[1]); printToUSART("\n");
    printToUSART("  "); printToUSART(drone.accelRaw[2]); printToUSART("\n\n");

    printToUSART(" [IMU] Gyro vals: \n");
    printToUSART("  "); printToUSART(drone.gyroRaw[0]); printToUSART("\n");
    printToUSART("  "); printToUSART(drone.gyroRaw[1]); printToUSART("\n");
    printToUSART("  "); printToUSART(drone.gyroRaw[2]); printToUSART("\n\n");
#endif
}

void stateEstimation() {
    estimateAngles(); //calculate the pitch and roll angles from the accelerometer axes using trig
    pt1Filter(); //clean the gyroscope noise before it reaches complementary filter
    complementaryFilter(); //fuses the gyroscope and the accelerometer tilt

#ifdef DEBUG
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
    printToUSART("  Attitude Yaw: "); printToUSART(drone.attitudeYaw); printToUSART("\n\n");
#endif
}

//PID values
volatile float Kp_roll = 1.2f, Ki_roll = 0.05f, Kd_roll = 0.3f; //adjust these numbers when bench testing (change to rates on betaflight)
volatile float Kp_pitch = 1.2f, Ki_pitch = 0.05f, Kd_pitch = 0.3f;
volatile float Kp_yaw = 1.2f, Ki_yaw = 0.05f, Kd_yaw = 0.3f;

//these get overriden do NOT adjust
static float rollErrorSum = 0.0f, lastRollError = 0.0f;
static float pitchErrorSum = 0.0f, lastPitchError = 0.0f;
static float yawErrorSum = 0.0f, lastYawError = 0.0f;

void flightLoop() { //make sure to forward values by usart
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

#ifdef DEBUG
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
    printToUSART("  m4: "); printToUSART(drone.motorOutput[3]); printToUSART("\n\n");
#endif
}

void crsfParsing() {
    yieldCurrentTask();

    while (true) {
        #ifdef SIMULATION
        CRSFPacket crsfBuffer =  populateCRSFMockBuffer();
        #else
        CRSFPacket crsfBuffer =  populateCRSFBuffer();
        #endif

        if (crsfBuffer.bytes[0] == 0x00 && crsfBuffer.bytes[1] == 0x00) { yieldCurrentTask(); continue; }

        if (crsfBuffer.bytes[0] != 0xC8) {//checks for corrupted data

            drone.currentSystemState = FlightState::ERROR;
            drone.errorMSG = "[CRSF] Corrupted data input stream!\n";
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

#ifdef DEBUG
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
        printToUSART("  Throttle Stick: "); printToUSART(drone.throttleStick); printToUSART("\n\n");
#endif

        drone.lastValidRCFrameTime = globalSystemTicks;
        yieldCurrentTask();
    }
}

void radioLinkFailSafe() {
    yieldCurrentTask();

    while (true) {
        if (globalSystemTicks - drone.lastValidRCFrameTime > 500) {
            drone.armed = false;
            drone.currentSystemState = FlightState::FAILSAFE;
            drone.failsafeMSG = FAILSAFE::CRSF_LOST;
        }
#ifdef DEBUG
        printToUSART("Radio Link Failsafe: \n");
        printToUSART(" [RLFS] Drone CRSF Link Quality: \n");
        printToUSART("  Armed: "); printToUSART(drone.armed); printToUSART("\n");
        printToUSART("  Last Frame: "); printToUSART(globalSystemTicks - drone.lastValidRCFrameTime); printToUSART("\n\n");
#endif
        yieldCurrentTask();
    }
}

static uint32_t lowVoltageStartTime = 0; //outside the while true loop so it doesn't get reset
static bool lowVoltageStarted = false;
constexpr float LOW_VOLTAGE_THRESHOLD = 19.2f;

void lowLevelFailSafe() {
    yieldCurrentTask();

    while (true) {
        //assumes that battery info has been populated by power management
        if (drone.batteryVoltage < LOW_VOLTAGE_THRESHOLD) {           
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

#ifdef DEBUG
        printToUSART("Low Level Fail Safe: \n");
        printToUSART(" [LLFS] Drone Battery Status: \n");
        printToUSART("  Armed: "); printToUSART(drone.armed); printToUSART("\n");
        printToUSART("  Dist To Threshold: "); printToUSART(drone.batteryCapacityUsed - LOW_VOLTAGE_THRESHOLD); printToUSART("\n\n");
#endif

        yieldCurrentTask(); //if no voltage drop was detected ignore and yield the task
    }
}

void sensorCalibration() { //performs a mean over the gyro samples and subtracts that from the final gyro output
#ifdef DEBUG
    printToUSART("Calibrating IMU Starting. Keep the drone still\n");
#endif

    float sumX = 0, sumY = 0, sumZ = 0;
    constexpr int samples = 2000;

    for (int i = 0; i < 2000; i++) {
        //populate gyro data with read imu function;
        readIMUDataRegisters();
        sumX += drone.gyroRaw[0];
        sumY += drone.gyroRaw[1];
        sumZ += drone.gyroRaw[2];
    }
#ifdef DEBUG
    printToUSART(" [SC] Reporting Sum Values:\n");
    printToUSART("  SumX: "); printToUSART(sumX); printToUSART("\n");
    printToUSART("  SumY: "); printToUSART(sumY); printToUSART("\n");
    printToUSART("  SumZ: "); printToUSART(sumZ); printToUSART("\n\n");
#endif

    gyroBias[0] = sumX / samples;
    gyroBias[1] = sumY / samples;
    gyroBias[2] = sumZ / samples;

    drone.calibrated = true;
#ifdef DEBUG
    printToUSART("Calibration complete.\n");
#endif
}


volatile uint16_t batteryADCBuffer[2] = {0, 0}; //used for circular buffer
void powerManagement() { //becomes its own task as it will be used in multiple places
    static uint32_t lastRunTime = 0;

    constexpr float VOLTAGE_DIVIDER_RATIO = 11.0f; //adjust based on schematic
    constexpr float TOTAL_BATTERY_CAPACITY_MAH = 1500.0f;
    constexpr float CURRENT_SENSOR_SCALE = 0.25f;

    yieldCurrentTask();

    while (true) {
        #ifdef SIMULATION
            float voltage = (1670.0f + ((rand() % 40) - 20)) * (3.3f / 4095.0f) * VOLTAGE_DIVIDER_RATIO;
            float currentVolts = (3100.0f + ((rand() % 60) - 30)) * (3.3f / 4095.0f);
            float currentAmps = currentVolts / CURRENT_SENSOR_SCALE;
        #else
            const uint16_t rawVoltageAdcValue = batteryADCBuffer[0]; //get latest values from circular dma buffer
            const uint16_t rawCurrentAdcValue = batteryADCBuffer[1];

            // Convert raw values to actual Volts and Amps
            float voltage = static_cast<float>(rawVoltageAdcValue) * (3.3f / 4095.0f) * VOLTAGE_DIVIDER_RATIO;
            float currentVolts = static_cast<float>(rawCurrentAdcValue) * (3.3f / 4095.0f);
            float currentAmps = currentVolts / CURRENT_SENSOR_SCALE;
        #endif

        //total capacity
        //used to compute hours since last measure
        uint32_t currentTime  = globalSystemTicks;
        uint32_t dtMs = (lastRunTime > 0) ? (currentTime - lastRunTime) : 0;
        lastRunTime = currentTime;

        static float accumulatedUsedMah = 0.0f;
        //mah = time elapsed (in hours) * amphours
        if (dtMs > 0) { accumulatedUsedMah += currentAmps * (static_cast<float>(dtMs) / 3600.0f); }

        float percentRemaining = 100.0f - ((accumulatedUsedMah / TOTAL_BATTERY_CAPACITY_MAH) * 100.0f);
        if (percentRemaining < 0.0f) { percentRemaining = 0.0f; }
        if (percentRemaining > 100.0f) { percentRemaining = 100.0f; }

        __disable_irq();
        drone.batteryVoltage = voltage;
        drone.batteryCurrent = currentAmps;
        drone.batteryCapacityUsed = accumulatedUsedMah;
        drone.batteryPerecent = static_cast<uint8_t>(percentRemaining);
        __enable_irq();

#ifdef DEBUG
        printToUSART("Power Management: \n");
        printToUSART(" [PM] Ouput Values: \n");
        printToUSART("  Voltage: "); printToUSART(drone.batteryVoltage); printToUSART("\n");
        printToUSART("  Current: "); printToUSART(currentAmps); printToUSART("\n");
        printToUSART("  Capacity: "); printToUSART(drone.batteryCapacityUsed); printToUSART("\n");
        printToUSART("  Percent: "); printToUSART(percentRemaining); printToUSART("\n\n");
#endif
        yieldCurrentTask();
    }
}

void dShotGeneration() {
    //enable the timer update DMA requests once at boot
    currentBoardConfig.motor_timer->DIER |= TIM_DIER_UDE;

    yieldCurrentTask();

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
    yieldCurrentTask();

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
                printToUSART(drone.warnMSG.data()); //printing to osd is automatic
                break;
            case FlightState::FAILSAFE:
                printToUSART(drone.failsafeMSG.data()); //printing to osd is automatic
                // drone.currentSystemState = FlightState::DISARMED;
                break;
            case FlightState::ERROR:
                printToUSART(drone.errorMSG.data()); //printing to osd is automatic
                // drone.currentSystemState = FlightState::DISARMED;
                break;
        }

        printToUSART("Flight State Machine: \n");
        printToUSART(" [FSM] System State: \n");
        printToUSART("  State: "); printToUSART(to_string(drone.currentSystemState).c_str()); printToUSART("\n\n");

        printToUSART(" [FSM] Errors: \n");
        printToUSART("  Warn msg: "); printToUSART(drone.warnMSG.data()); printToUSART("\n");
        printToUSART("  Error msg: "); printToUSART(drone.errorMSG.data()); printToUSART("\n");
        printToUSART("  Failsafe msg: "); printToUSART(drone.failsafeMSG.data()); printToUSART("\n\n");

        yieldCurrentTask();
    }
}

//master task for imu
void imuControlLoop() { // one interrupt-zone task, triggered by real IMU DRDY eventually
    yieldCurrentTask();

    while (true) {
        printToUSART("[IMU-CL] Start: \n");
        readIMUDataRegisters();
        stateEstimation();
        flightLoop();
        printToUSART("[IMU-CL] End \n");
        yieldCurrentTask();
    }
}

//main scheduled tasks
void iwdgTask() {
    IWDG->KR  = 0x5555; //enable register by writing
    IWDG->PR  = IWDG_PR_PR_2; //32khz = 1ms clock tick (same as systick
    IWDG->RLR = 1000;
    IWDG->KR  = 0xAAAA; //reload counter
    IWDG->KR  = 0xCCCC; //start watchdog

#ifdef DEBUG
    printToUSART("[IWDG] Timer Started \n\n");
#endif

    yieldCurrentTask();

    while (true) {
        IWDG->KR = 0xAAAA; //reset the counter

#ifdef DEBUG
        printToUSART("[IWDG] Timer Reset! \n\n");
#endif

        yieldCurrentTask();
    }
}

void updatePeripherals() {
    yieldCurrentTask();

    while (true) {

        //LED logic
        if (drone.currentSystemState == FlightState::DISARMED) {
            currentBoardConfig.status_led_port->BSRR = static_cast<uint32_t>(currentBoardConfig.status_led_pin) << 16; //bit shit 16 to the left to set low

        } else if (drone.currentSystemState == FlightState::ARMED) {
            currentBoardConfig.status_led_port->BSRR = currentBoardConfig.status_led_pin;

        } else if (drone.currentSystemState == FlightState::WARN ||
            drone.currentSystemState == FlightState::FAILSAFE ||
            drone.currentSystemState == FlightState::ERROR
           )
        {
            constexpr uint32_t BLINK_WINDOW_MS = 166;
            constexpr uint32_t BLINK_PULSE_MS  = 20;

            if (globalSystemTicks % BLINK_WINDOW_MS < BLINK_PULSE_MS) { currentBoardConfig.status_led_port->BSRR = static_cast<uint32_t>(currentBoardConfig.status_led_pin) << 16; }
            else { currentBoardConfig.status_led_port->BSRR = currentBoardConfig.status_led_pin; }

        } else {
            currentBoardConfig.status_led_port->BSRR = static_cast<uint32_t>(currentBoardConfig.status_led_pin) << 16; //default state low
        }

        //buzzer logic
        if (drone.currentSystemState == FlightState::FAILSAFE ||
            drone.currentSystemState == FlightState::ERROR ||
            drone.currentSystemState == FlightState::WARN
           )
        {
            currentBoardConfig.buzzer_port->BSRR = currentBoardConfig.buzzer_pin; //on

        } else {
            currentBoardConfig.buzzer_port->BSRR = static_cast<uint32_t>(currentBoardConfig.buzzer_pin) << 16; //off
        }

#ifdef DEBUG
        printToUSART("Update Peripherals: \n");
        printToUSART(" [UP] LEDS: \n");

        currentBoardConfig.status_led_port->BSRR == currentBoardConfig.status_led_pin ? printToUSART("  State: ON\n\n") : printToUSART("  State: OFF\n\n");

        printToUSART(" [UP] BUZZER: \n");
        currentBoardConfig.buzzer_port->BSRR == currentBoardConfig.buzzer_pin ? printToUSART("  State: ON\n\n") : printToUSART("  State: OFF\n\n");

        printToUSART(" [UP] System: \n");
        printToUSART("  State: "); printToUSART(to_string(drone.currentSystemState).c_str()); printToUSART("\n\n");
#endif

        yieldCurrentTask(); //yield
    }
}

//3 buffers
uint8_t batteryTX[8];
uint8_t flightModeTX[1];
uint8_t gpsTX[15];

void telemetryTX() {
    yieldCurrentTask();

    while (true) {
        //handle battery transmitting
        const auto voltage = static_cast<uint16_t>(drone.batteryVoltage * 10.0f);
        const auto current = static_cast<uint16_t>(drone.batteryCurrent * 10.0f);
        const auto capacityUsed = static_cast<uint32_t>(drone.batteryCapacityUsed * 10.0f);
        const auto perecentage = static_cast<uint8_t>(drone.batteryPerecent);

        batteryTX[0] = (voltage >> 8) & 0xFF;
        batteryTX[1] = (voltage & 0xFF);

        batteryTX[2] = (current >> 8) & 0xFF;
        batteryTX[3] = (current & 0xFF);

        batteryTX[4] = (capacityUsed >> 16) & 0xFF;
        batteryTX[5] = (capacityUsed >> 8) & 0xFF;
        batteryTX[6] = (capacityUsed & 0xFF);

        batteryTX[7] = (perecentage) & 0xFF;

        //handle flight mode handling
        flightModeTX[0] = static_cast<uint8_t>(drone.currentSystemState) & 0xFF; //its already uint8_t

        //handle gps values
        const auto latitude = static_cast<int32_t>(drone.latitude * std::pow(10, 7));
        const auto longitude = static_cast<int32_t>(drone.longitude * std::pow(10, 7));
        const auto speed = static_cast<uint16_t>(drone.groundSpeed * 10);
        const auto heading = static_cast<uint16_t>(drone.heading * 100);
        const auto altitude = static_cast<uint16_t>(drone.altitude * 100);
        const auto satellites = static_cast<uint8_t>(drone.satellites);

        gpsTX[0] = (latitude >> 24) & 0xFF;
        gpsTX[1] = (latitude >> 16) & 0xFF;
        gpsTX[2] = (latitude >> 8) & 0xFF;
        gpsTX[3] = (latitude & 0xFF);

        gpsTX[4] = (longitude >> 24) & 0xFF;
        gpsTX[5] = (longitude >> 16) & 0xFF;
        gpsTX[6] = (longitude >> 8) & 0xFF;
        gpsTX[7] = (longitude & 0xFF);

        gpsTX[8] = (speed >> 8) & 0xFF;
        gpsTX[9] = (speed & 0xFF);

        gpsTX[10] = (heading >> 8) & 0xFF;
        gpsTX[11] = (heading & 0xFF);

        gpsTX[12] = (altitude >> 8) & 0xFF;
        gpsTX[13] = (altitude & 0xFF);

        gpsTX[14] = (satellites) & 0xFF;

        //format the buffers with the checksums
        uint8_t formattedBatteryTX[12];
        uint8_t formattedFlightModeTX[5];
        uint8_t formattedGPSTX[19];

        const uint8_t formattedBatteryTXSize = formatCRSFFrame(0x08, batteryTX, 8, formattedBatteryTX);
        const uint8_t formattedFlightModeTXSize = formatCRSFFrame(0x21, flightModeTX, 1, formattedFlightModeTX);
        const uint8_t formattedGPSSize = formatCRSFFrame(0x02, gpsTX, 15, formattedGPSTX);

#ifdef DEBUG
        printToUSART("Telemetry TX: \n");
        printToUSART(" [TXT-BAT] Battery: \n");
        printToUSART("  Voltage: "); printToUSART(drone.batteryVoltage); printToUSART("\n");
        printToUSART("  Current: "); printToUSART(drone.batteryCurrent); printToUSART("\n");
        printToUSART("  Capacity: "); printToUSART(drone.batteryCapacityUsed); printToUSART("\n");
        printToUSART("  Percent: "); printToUSART(drone.batteryPerecent); printToUSART("\n\n");

        printToUSART(" [TXT-FLM] Flight Status: \n");
        printToUSART("  Flight Mode: "); printToUSART(to_string(drone.currentSystemState).c_str()); printToUSART("\n");
        printToUSART("  Warn msg: "); printToUSART(drone.warnMSG.data()); printToUSART("\n");
        printToUSART("  Error msg: "); printToUSART(drone.errorMSG.data()); printToUSART("\n");
        printToUSART("  Failsafe msg: "); printToUSART(drone.failsafeMSG.data()); printToUSART("\n\n");

        printToUSART(" [TXT-GPS] Location: \n");
        printToUSART("  Latitude: "); printToUSART(drone.latitude); printToUSART("\n");
        printToUSART("  Longitude: "); printToUSART(drone.longitude); printToUSART("\n");
        printToUSART("  Altitude: "); printToUSART(drone.altitude); printToUSART("\n\n");

        printToUSART(" [TXT-GPS] Directionality: \n");
        printToUSART("  Ground Speed (kph): "); printToUSART(drone.groundSpeed); printToUSART("\n");
        printToUSART("  Heading: "); printToUSART(drone.heading); printToUSART("\n");
        printToUSART("  Satellites: "); printToUSART(drone.satellites); printToUSART("\n\n\n");
#endif

        for (int i = 0; i < formattedBatteryTXSize; i++) {
             while (!(currentBoardConfig.crsf_uart->ISR & USART_ISR_TXE)) {}
             currentBoardConfig.crsf_uart->TDR = formattedBatteryTX[i];
        }

        for (int i = 0; i < formattedFlightModeTXSize; i++) {
             while (!(currentBoardConfig.crsf_uart->ISR & USART_ISR_TXE)) {}
             currentBoardConfig.crsf_uart->TDR = formattedFlightModeTX[i];
        }

        for (int i = 0; i < formattedGPSSize; i++) {
             while (!(currentBoardConfig.crsf_uart->ISR & USART_ISR_TXE)) {}
             currentBoardConfig.crsf_uart->TDR = formattedGPSTX[i];
        }

        //yield
        yieldCurrentTask();
    }
}

void osdUpdate() {
    yieldCurrentTask();

    while (true) {

        //yield
        yieldCurrentTask();
    }
}

void gpsParser() {
    static char sentenceBuffer[96];
    static uint8_t bufferIdx = 0;

    yieldCurrentTask();

    while (true) {

        while (currentBoardConfig.gps_uart->ISR & USART_ISR_RXNE) { //while recieving new bytes

            char c = static_cast<char>(currentBoardConfig.gps_uart->RDR); //gets a single byte from read

            if (c == '$') { //reset sequence and start new transmit
                bufferIdx = 0;
                sentenceBuffer[bufferIdx++] = c;
            }

            else if (c == '\r' || c == '\n') {
                if (bufferIdx > 0) {
                    sentenceBuffer[bufferIdx] = '\0';
                    if (std::strncmp(sentenceBuffer, "$GNGGA", 6) == 0 || std::strncmp(sentenceBuffer, "$GPGGA", 6) == 0) { //ensure gnss or gps signal via string compare
                        int32_t  localLat = 0; //reset locals
                        int32_t  localLon = 0;
                        uint8_t  localSats = 0;
                        uint16_t localAlt = 0;
                        uint8_t  fixQuality = 0;

                        char* token = strtok(sentenceBuffer, ","); //split sentence by commas
                        uint8_t field = 0; //used to track position oin sentence

                        char latStr[16] = {0};
                        char latDir = 'N';
                        char lonStr[16] = {0};
                        char lonDir = 'E';

                        while (token != nullptr) {
                            field++;
                            switch (field) {
                                case 3: strncpy(latStr, token, sizeof(latStr) - 1); break;
                                case 4: latDir = token[0]; break;
                                case 5: strncpy(lonStr, token, sizeof(lonStr) - 1); break;
                                case 6: lonDir = token[0]; break;
                                case 7: fixQuality = static_cast<uint8_t>(atoi(token)); break; //convert to int using atoi
                                case 8: localSats = static_cast<uint8_t>(atoi(token)); break;
                                case 10: localAlt = static_cast<uint16_t>(atof(token)); break;
                                default: break;
                            }
                            token = strtok(nullptr, ",");
                        }

                        //only commit values to global drone struct if we have a valid 2D/3D fix (e.g. good satellite connections etc.)
                        if (fixQuality > 0) {
                            localLat = parseNmeaCoord(latStr, latDir);
                            localLon = parseNmeaCoord(lonStr, lonDir);

                            __disable_irq();
                            drone.latitude   = localLat;
                            drone.longitude  = localLon;
                            drone.satellites = localSats;
                            drone.altitude   = localAlt;
                            __enable_irq();
                        }
                    } else if (std::strncmp(sentenceBuffer, "$GNRMC", 6) == 0 || std::strncmp(sentenceBuffer, "$GPRMC", 6) == 0) {
                        char* token = strtok(sentenceBuffer, ","); //split sentence by commas
                        uint8_t field = 0;
                        bool statusValid = false;

                        float rawKnots = 0.0f;
                        float rawHeading = 0.0f;

                        while (token != nullptr) {
                            field++;
                            switch (field) {
                                case 3: statusValid = (token[0] == 'A'); break;
                                case 8: rawKnots = static_cast<float>(atof(token)); break; //convert to float using atof
                                case 9: rawHeading = atof(token); break;
                                default: statusValid = false; break;
                            }

                            token = strtok(nullptr, ",");
                        }

                        if (statusValid) {
                            auto localSpeed = static_cast<uint16_t>(rawKnots * 1.852f); //convert to kmh
                            auto localHeading = static_cast<uint16_t>(rawHeading);

                            __disable_irq();
                            drone.groundSpeed = localSpeed;
                            drone.heading = localHeading;
                            __enable_irq();
                        }
                    }

                    bufferIdx = 0; //reset for next idx
                }
            }

            else if (bufferIdx < sizeof(sentenceBuffer) - 1) {
                sentenceBuffer[bufferIdx++] = c;
            }

        }

#ifdef DEBUG
        printToUSART("GPS Parser: \n");
        printToUSART(" [GPSP] Location: \n");
        printToUSART("  Latitude: "); printToUSART(drone.latitude); printToUSART("\n");
        printToUSART("  Longitude: "); printToUSART(drone.longitude); printToUSART("\n");
        printToUSART("  Altitude: "); printToUSART(drone.altitude); printToUSART("\n\n");

        printToUSART(" [GPSP] Directionality: \n");
        printToUSART("  Ground Speed (kph): "); printToUSART(drone.groundSpeed); printToUSART("\n");
        printToUSART("  Heading: "); printToUSART(drone.heading); printToUSART("\n");
        printToUSART("  Satellites: "); printToUSART(drone.satellites); printToUSART("\n\n");
#endif

        yieldCurrentTask(); //yield
    }
}

void usbCLI() {
    static char commandBuffer[64];
    static uint8_t cmdIdx = 0;

    printToUSART("\r\n=================================\r\n");
    printToUSART("   DRONE RTOS FLIGHT TERMINAL   \r\n");
    printToUSART("=================================\r\n# ");

    yieldCurrentTask();

    while (true) {
        printToUSART("[USB-CLI] Command Line Active!\n\n");
        while (currentBoardConfig.cli_uart->ISR & USART_ISR_RXNE) { //wait for bytes
            char c = static_cast<char>(currentBoardConfig.cli_uart->RDR);

            //handle Backspace
            if (c == '\b' || c == 127) {
                if (cmdIdx > 0) {
                    cmdIdx--;
                    //erase character on user's terminal
                    while (!(currentBoardConfig.cli_uart->ISR & USART_ISR_TXE)) {} //wait for registers to be free
                    currentBoardConfig.cli_uart->TDR = '\b'; //transmit back bytes like backspace, space etc.
                    while (!(currentBoardConfig.cli_uart->ISR & USART_ISR_TXE)) {}
                    currentBoardConfig.cli_uart->TDR = ' ';
                    while (!(currentBoardConfig.cli_uart->ISR & USART_ISR_TXE)) {}
                    currentBoardConfig.cli_uart->TDR = '\b';
                }
                continue;
            }

            //echo char back
            while (!(currentBoardConfig.cli_uart->ISR & USART_ISR_TXE)) {} //wait
            currentBoardConfig.cli_uart->TDR = c;

            //handle enter key
            if (c == '\r' || c == '\n') {
                commandBuffer[cmdIdx] = '\0';

                if (cmdIdx > 0) {
                    printToUSART("\r\n");

                    //points to the next character after the last space
                    const char* numPtr = std::strrchr(commandBuffer, ' ');
                    float parsedVal = numPtr ? static_cast<float>(std::atof(numPtr + 1)) : 0.0f;

                    //system status
                    if (std::strncmp(commandBuffer, "status", 6) == 0) {
                        printToUSART("[SYSTEM STATUS]\r\n");
                        printToUSART("  Battery Voltage: "); printToUSART(drone.batteryVoltage); printToUSART(" V\r\n");
                        printToUSART("  Flight State   : "); printToUSART(static_cast<uint32_t>(drone.currentSystemState)); printToUSART("\r\n");
                        printToUSART("  Armed          : "); printToUSART(drone.armed); printToUSART("\r\n");
                        printToUSART("  Sats Locked    : "); printToUSART(static_cast<uint32_t>(drone.satellites)); printToUSART("\r\n");
                    }

                    //tune kp
                    else if (std::strncmp(commandBuffer, "set kp --roll", 13) == 0 || std::strncmp(commandBuffer, "set kp -r", 9) == 0) {
                        Kp_roll = parsedVal;
                        printToUSART("Kp_roll updated to: "); printToUSART(Kp_roll); printToUSART("\r\n");
                    }
                    else if (std::strncmp(commandBuffer, "set kp --pitch", 14) == 0 || std::strncmp(commandBuffer, "set kp -p", 9) == 0) {
                        Kp_pitch = parsedVal;
                        printToUSART("Kp_pitch updated to: "); printToUSART(Kp_pitch); printToUSART("\r\n");
                    }
                    else if (std::strncmp(commandBuffer, "set kp --yaw", 12) == 0 || std::strncmp(commandBuffer, "set kp -y", 9) == 0) {
                        Kp_yaw = parsedVal;
                        printToUSART("Kp_yaw updated to: "); printToUSART(Kp_yaw); printToUSART("\r\n");
                    }

                    //tune ki
                    else if (std::strncmp(commandBuffer, "set ki --roll", 13) == 0 || std::strncmp(commandBuffer, "set ki -r", 9) == 0) {
                        Ki_roll = parsedVal;
                        printToUSART("Ki_roll updated to: "); printToUSART(Ki_roll); printToUSART("\r\n");
                    }

                    else if (std::strncmp(commandBuffer, "set ki --pitch", 14) == 0 || std::strncmp(commandBuffer, "set ki -p", 9) == 0) {
                        Ki_pitch = parsedVal;
                        printToUSART("Ki_pitch updated to: "); printToUSART(Ki_pitch); printToUSART("\r\n");
                    }
                    else if (std::strncmp(commandBuffer, "set ki --yaw", 12) == 0 || std::strncmp(commandBuffer, "set ki -y", 9) == 0) {
                        Ki_yaw = parsedVal;
                        printToUSART("Ki_yaw updated to: "); printToUSART(Ki_yaw); printToUSART("\r\n");
                    }

                    //tune kd
                    else if (std::strncmp(commandBuffer, "set kd --roll", 13) == 0 || std::strncmp(commandBuffer, "set kd -r", 9) == 0) {
                        Kd_roll = parsedVal;
                        printToUSART("Kd updated to: "); printToUSART(Kd_roll); printToUSART("\r\n");
                    }
                    else if (std::strncmp(commandBuffer, "set kd --pitch", 14) == 0 || std::strncmp(commandBuffer, "set kd -p", 9) == 0) {
                        Kd_pitch = parsedVal;
                        printToUSART("Kd_pitch updated to: "); printToUSART(Kd_pitch); printToUSART("\r\n");
                    }
                    else if (std::strncmp(commandBuffer, "set kd --yaw", 12) == 0 || std::strncmp(commandBuffer, "set kd -y", 9) == 0) {
                        Kd_yaw = parsedVal;
                        printToUSART("Kd_yaw updated to: "); printToUSART(Kd_yaw); printToUSART("\r\n");
                    }

                    //inspect registers
                    else if (std::strncmp(commandBuffer, "get --pid", 9) == 0 || std::strncmp(commandBuffer, "get -p", 6) == 0) {
                        printToUSART("[PID GAINS]\r\n");
                        printToUSART("  Kp: "); printToUSART(Kp_roll); printToUSART("\r\n");
                        printToUSART("  Ki: "); printToUSART(Ki_roll); printToUSART("\r\n");
                        printToUSART("  Kd: "); printToUSART(Kd_roll); printToUSART("\r\n");
                    }
                    else if (std::strncmp(commandBuffer, "get --imu", 9) == 0 || std::strncmp(commandBuffer, "get -i", 6) == 0) {
                        printToUSART("[IMU ATTITUDE]\r\n");
                        printToUSART("  Roll : "); printToUSART(drone.attitudeRoll); printToUSART("\r\n");
                        printToUSART("  Pitch: "); printToUSART(drone.attitudePitch); printToUSART("\r\n");
                        printToUSART("  Yaw  : "); printToUSART(drone.attitudeYaw); printToUSART("\r\n");
                    }
                    else if (std::strncmp(commandBuffer, "get --gps", 9) == 0 || std::strncmp(commandBuffer, "get -g", 6) == 0) {
                        printToUSART("[GPS DATA]\r\n");
                        printToUSART("  Lat : "); printToUSART(drone.latitude); printToUSART("\r\n");
                        printToUSART("  Lon : "); printToUSART(drone.longitude); printToUSART("\r\n");
                        printToUSART("  Alt : "); printToUSART(static_cast<uint32_t>(drone.altitude)); printToUSART(" m\r\n");
                        printToUSART("  Sats: "); printToUSART(static_cast<uint32_t>(drone.satellites)); printToUSART("\r\n");
                    }
                    else if (std::strncmp(commandBuffer, "get --motors", 12) == 0 || std::strncmp(commandBuffer, "get -m", 6) == 0) {
                        printToUSART("[DSHOT MOTOR OUTPUTS]\r\n");
                        printToUSART("  M1: "); printToUSART(drone.motorOutput[0]); printToUSART("\r\n");
                        printToUSART("  M2: "); printToUSART(drone.motorOutput[1]); printToUSART("\r\n");
                        printToUSART("  M3: "); printToUSART(drone.motorOutput[2]); printToUSART("\r\n");
                        printToUSART("  M4: "); printToUSART(drone.motorOutput[3]); printToUSART("\r\n");
                    }

                    //system reboot
                    else if (std::strncmp(commandBuffer, "reboot", 6) == 0) {
                        printToUSART("Rebooting MCU...\r\n");
                        NVIC_SystemReset();
                    }

                    //help menu
                    else if (std::strncmp(commandBuffer, "help", 4) == 0) {
                        printToUSART("Available Commands:\r\n");
                        printToUSART("  status                      - System telemetry & state\r\n");
                        printToUSART("  get -p / get --pid          - Print all PID gains\r\n");
                        printToUSART("  set kp -r / --roll <val>    - Update Roll Kp\r\n");
                        printToUSART("  set kp -p / --pitch <val>   - Update Pitch Kp\r\n");
                        printToUSART("  set kp -y / --yaw <val>     - Update Yaw Kp\r\n");
                        printToUSART("  get -i / get --imu          - Print orientation\r\n");
                        printToUSART("  get -g / get --gps          - Print GPS coordinates\r\n");
                        printToUSART("  get -m / get --motors       - Print DShot outputs\r\n");
                        printToUSART("  reboot                      - Software reset\r\n");
                    }
                    else {
                        printToUSART("Unknown command. Type 'help' for options.\r\n");
                    }
                }

                //reset buffer and output fresh prompt
                cmdIdx = 0;
                printToUSART("# ");
            }
            //store ASCII characters
            else if (cmdIdx < sizeof(commandBuffer) - 1 && c >= 32 && c <= 126) {
                commandBuffer[cmdIdx++] = c;
            }
        }

        yieldCurrentTask();
    }
}