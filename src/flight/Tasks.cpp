    //
    // Tasks.cpp by Akshay Gillett on 7/12/26.
    //

    #include <algorithm>
    #include <cinttypes>
    #include <cmath>
    #include <cstring>
    #include <iostream>
    #include <random>

    #include "../sim/SimInjector.h"
    #include "./flight_control.h"
    #include "../rtos/rtos.h"
    #include "./BufferPopulation.h"
    #include "stm32f7xx.h"
    #include "Helpers.h"
    #include "../rtos/logging/Logger.h"

    DroneState drone; //define drone struct

    static float gyroBias[3] = {0.0f, 0.0f, 0.0f};

    //imu orientation cli values
    //these values are caclulated in a roation matrix and are multiplied against the actual
    //gryo values to offset the calculation and therofre roatate the imu to any orientation

    float imu_cos_theta = 1;
    float imu_sin_theta = 0;

    void readIMUDataRegisters() {
        //function to populate the 14 registers starting at the start address
        IMURawPacket rawIMUBuffer = populateIMUBuffer(); //don't raw inject in order to test populateIMUBuffer

        //reassemble registers into signed 16-bit integers
        const int16_t rawAccelX = (rawIMUBuffer.bytes[1] << 8) | rawIMUBuffer.bytes[0];
        const int16_t rawAccelY = (rawIMUBuffer.bytes[3] << 8) | rawIMUBuffer.bytes[2];
        const int16_t rawAccelZ = (rawIMUBuffer.bytes[5] << 8) | rawIMUBuffer.bytes[4];

        //reassemble Gyroscope
        const int16_t rawGyroX = (rawIMUBuffer.bytes[7] << 8) | rawIMUBuffer.bytes[6];
        const int16_t rawGyroY = (rawIMUBuffer.bytes[9] << 8) | rawIMUBuffer.bytes[8];
        const int16_t rawGyroZ = (rawIMUBuffer.bytes[11] << 8) | rawIMUBuffer.bytes[10];

        //do not multiple after (you need to use volatile)


        //scale using floats for precice rotation
        float scaledAx = static_cast<float>(rawAccelX) / 2048.0f;
        float scaledAy = static_cast<float>(rawAccelY) / 2048.0f;
        float scaledAz = static_cast<float>(rawAccelZ) / 2048.0f;

        float scaledGx = static_cast<float>(rawGyroX) / 16.4f;
        float scaledGy = static_cast<float>(rawGyroY) / 16.4f;
        float scaledGz = static_cast<float>(rawGyroZ) / 16.4f;

        //scaled x and y with rotation matix (z axis stays unrotated as the pcb is flat - only rotating in 2d space)
        float rotAx = (scaledAx * imu_cos_theta) - (scaledAy * imu_sin_theta);
        float rotAy = (scaledAx * imu_sin_theta) + (scaledAy * imu_cos_theta);

        float rotGx = (scaledGx * imu_cos_theta) - (scaledGy * imu_sin_theta);
        float rotGy = (scaledGx * imu_sin_theta) + (scaledGy * imu_cos_theta);


        __disable_irq(); //only disable when actually writing to the drone struct
        drone.accelRaw[0] = rotAx;
        drone.accelRaw[1] = rotAy;
        drone.accelRaw[2] = scaledAz;

        drone.gyroRaw[0] = rotGx;
        drone.gyroRaw[1] = rotGy;
        drone.gyroRaw[2] = scaledGz;

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

    static ActualRateConfig rollRateCfg  = { .centerRate = 200.0f, .maxRate = 670.0f, .expo = 0.5f };
    static ActualRateConfig pitchRateCfg = { .centerRate = 200.0f, .maxRate = 670.0f, .expo = 0.5f };
    static ActualRateConfig yawRateCfg   = { .centerRate = 200.0f, .maxRate = 670.0f, .expo = 0.5f };

    void flightLoop() {
        //P - reacts to present error (proportional)
        //I - reacts to past accumulated error (Integral)
        //D - preducts future error (Derivative)

        //declare variables outside if-scope
        //apply betaflight like rates
        const float targetRoll = applyActualRates(drone.rollStick, rollRateCfg);
        const float targetPitch = applyActualRates(drone.pitchStick, pitchRateCfg);
        const float targetYaw = applyActualRates(drone.yawStick, yawRateCfg);
        const float targetThrottle = drone.throttleStick;

        const float rollError = targetRoll - drone.attitudeRoll;
        const float pitchError = targetPitch - drone.attitudePitch;
        const float yawError = targetYaw - drone.attitudeYaw;

        auto handleErrorState = []() {
    #ifdef DEBUG
            printToUSART("[FL-ARM-FLAG] Zeroing Motor Outputs!\n\n");
    #endif
            __disable_irq();
            drone.motorOutput[0] = 0; //front left
            drone.motorOutput[1] = 0; //front right
            drone.motorOutput[2] = 0; //back left
            drone.motorOutput[3] = 0; //back right
            __enable_irq();
        };

        auto handleArmedState = [rollError, pitchError, yawError, targetThrottle]() {
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

            const float baseThrottle = targetThrottle * 2000.0f;
            //mixer for dshot600 commands - adjust direction based upon which motors spin ccw vs cw
            drone.motorOutput[0] = std::clamp(baseThrottle + pitchOutput + rollOutput - yawOutput, 0.0f, 2000.0f);
            drone.motorOutput[1] = std::clamp(baseThrottle + pitchOutput - rollOutput + yawOutput, 0.0f, 2000.0f);
            drone.motorOutput[2] = std::clamp(baseThrottle - pitchOutput + rollOutput + yawOutput, 0.0f, 2000.0f);
            drone.motorOutput[3] = std::clamp(baseThrottle - pitchOutput - rollOutput - yawOutput, 0.0f, 2000.0f);
        };

        //prevents the drone from sending dshot commands if not armed
        switch (drone.currentSystemState) {
            case FlightState::DISARMED: handleErrorState(); break;
            case FlightState::FAILSAFE: handleErrorState(); break;
            case FlightState::ERROR: handleErrorState(); break;
            case FlightState::WARN:
                if (drone.warnMSG == WARN::ARMED_ANGLE ||
                    drone.warnMSG == WARN::ARMED_THROTTLE
                   )
                {
                    handleErrorState();
                } else {
                    handleArmedState();
                }

                break;

            case FlightState::ARMED: handleArmedState(); break;
        }


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

    //cli mapping constant channels
    volatile uint16_t
    ch1 = 0, ch2 = 0, ch3 = 0,
    ch4 = 0, ch5 = 0, ch6 = 0,
    ch7 = 0, ch8 = 0, ch9 = 0,
    ch10 = 0, ch11 = 0;

    volatile uint16_t* ROLL_CH = &ch1;
    volatile uint16_t* PITCH_CH = &ch2;
    volatile uint16_t* YAW_CH = &ch3;
    volatile uint16_t* THROTTLE_CH = &ch4;
    volatile uint16_t* ARMED_CH = &ch5;

    void crsfParsing() { //do not return anymore as it is a helper for the crsfControlLoop
            CRSFPacket crsfBuffer =  populateCRSFBuffer(); //don't raw inject in order to test populateCRSFBuffer

            if (crsfBuffer.bytes[0] == 0x00 && crsfBuffer.bytes[1] == 0x00) { return; }

            if (crsfBuffer.bytes[0] != 0xC8) { //checks for corrupted data

                drone.currentSystemState = FlightState::ERROR;
                setEventMessage(drone.errorMSG, ERROR::CRSF_CORRUPT_STREAM);
                return;
            }

            //use bitwise operators to traverse the array and extract the correct values
            ch1 = (crsfBuffer.bytes[3] | crsfBuffer.bytes[4] << 8) & 0x07FF;
            ch2 = (crsfBuffer.bytes[4] >> 3 | crsfBuffer.bytes[5] << 5) & 0x07FF;

            ch3 = (crsfBuffer.bytes[5] >> 6 | crsfBuffer.bytes[6] << 2 | crsfBuffer.bytes[7] << 10) & 0x07FF;
                ch4 = (crsfBuffer.bytes[7] >> 1 | crsfBuffer.bytes[8] << 7) & 0x07FF;

            ch5 = (crsfBuffer.bytes[8] >> 4 | crsfBuffer.bytes[9] << 4) & 0x07FF;
            ch6 = (crsfBuffer.bytes[9] >> 7 | crsfBuffer.bytes[10] << 1 | crsfBuffer.bytes[11] << 9) & 0x07FF;

                ch7 = (crsfBuffer.bytes[11] >> 2 | crsfBuffer.bytes[12] << 6) & 0x07FF;
                ch8 = (crsfBuffer.bytes[12] >> 5 | crsfBuffer.bytes[13] << 3) & 0x07FF;
                ch9 = (crsfBuffer.bytes[14] | crsfBuffer.bytes[15] << 8) & 0x07FF;
                ch10 = (crsfBuffer.bytes[15] >> 3 | crsfBuffer.bytes[16] << 5) & 0x07FF;
                ch11 = (crsfBuffer.bytes[16] >> 6 | crsfBuffer.bytes[17] << 2 | crsfBuffer.bytes[18] << 10) & 0x07FF;


            //convert stick positions into proper format (e.g. bound to -1.0f to 1.0f)
            const float normalizedRollStick = (static_cast<float>(*ROLL_CH) - 992.0f) / 820.0f;
            const float normalizedPitchStick = (static_cast<float>(*PITCH_CH) - 992.0f) / 820.0f;
            const float normalizedYawStick = (static_cast<float>(*YAW_CH) - 992.0f) / 820.0f;
            const float noralizedThrottleStick = (static_cast<float>(*THROTTLE_CH) - 172.0f) / 1639.0f;

            drone.rollStick = normalizedRollStick;
            drone.pitchStick = normalizedPitchStick;
            drone.yawStick = normalizedYawStick;
            drone.throttleStick = noralizedThrottleStick;

            drone.armed = *ARMED_CH > 1000;

    #ifdef DEBUG
            printToUSART("Reading CRSF Data Registers\n");

            printToUSART(" [CRSF] Drone Channel Values:\n");
            printToUSART("  Ch1: "); printToUSART(ch1); printToUSART("\n");
            printToUSART("  Ch2: "); printToUSART(ch2); printToUSART("\n");
            printToUSART("  Ch3: "); printToUSART(ch3); printToUSART("\n");
            printToUSART("  Ch4: "); printToUSART(ch4); printToUSART("\n");
            printToUSART("  Ch5: "); printToUSART(ch5); printToUSART("\n");
            printToUSART("  Ch6: "); printToUSART(ch6); printToUSART("\n");
            printToUSART("  Ch7: "); printToUSART(ch7); printToUSART("\n");
            printToUSART("  Ch8: "); printToUSART(ch8); printToUSART("\n");
            printToUSART("  Ch9: "); printToUSART(ch9); printToUSART("\n");
            printToUSART("  Ch10: "); printToUSART(ch10); printToUSART("\n");
            printToUSART("  Ch11: "); printToUSART(ch11); printToUSART("\n\n");

            printToUSART(" [CRSF] Drone Stick Values: \n");
            printToUSART("  Roll Stick: "); printToUSART(drone.rollStick); printToUSART("\n");
            printToUSART("  Pitch Stick: "); printToUSART(drone.pitchStick); printToUSART("\n");
            printToUSART("  Yaw Stick: "); printToUSART(drone.yawStick); printToUSART("\n");
            printToUSART("  Throttle Stick: "); printToUSART(drone.throttleStick); printToUSART("\n\n");
    #endif

            drone.lastValidRCFrameTime = globalSystemTicks;
    }

    void radioLinkFailSafe() {
        yieldCurrentTask();

        while (true) {

            volatile uint32_t lastFrame = globalSystemTicks - drone.lastValidRCFrameTime;

            if (lastFrame > 300) {
                drone.armed = false;
                drone.currentSystemState = FlightState::FAILSAFE;
                setEventMessage(drone.failsafeMSG, FAILSAFE::CRSF_LOST);
            }
    #ifdef DEBUG
            printToUSART("Radio Link Failsafe: \n");
            printToUSART(" [RLFS] Drone CRSF Link Quality: \n");
            printToUSART("  Armed: "); printToUSART(to_string(drone.currentSystemState)); printToUSART("\n");
            printToUSART("  Last Frame: "); printToUSART(lastFrame); printToUSART("\n");
            switch (lastFrame) {
                case 0 ... 99: printToUSART("  Quality: GOOD\n\n"); break;
                case 100 ... 199: printToUSART("  Quality: MEDIUM\n\n"); break;
                case 200 ... 299: printToUSART("  Quality: CRITICAL\n\n"); break;
                case 300 ... 500: printToUSART("  Quality: FAILSAFE\n\n"); break;
            }
    #endif
            yieldCurrentTask();
        }
    }

    constexpr float LOW_VOLTAGE_THRESHOLD = 19.2f; //outside the task stack so that cli can access it

    void lowLevelFailSafe() {
        uint32_t lowVoltageStartTime = 0; //outside the while true loop so it doesn't get reset
        bool lowVoltageStarted = false;

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
                            setEventMessage(drone.warnMSG, WARN::LOW_BATTERY);
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
            printToUSART("  Dist To Threshold: "); printToUSART(drone.batteryCapacityUsed - LOW_VOLTAGE_THRESHOLD); printToUSART("\n\n"); //consider fixing debug print
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

    void powerManagement() { //becomes its own task as it will be used in multiple places
        static uint32_t lastRunTime = 0;

        constexpr float VOLTAGE_DIVIDER_RATIO = 11.0f; //adjust based on schematic
        constexpr float TOTAL_BATTERY_CAPACITY_MAH = 1500.0f;
        constexpr float CURRENT_SENSOR_SCALE = 0.25f;

        yieldCurrentTask();

        while (true) {
            ADCPacket packet = populateBatteryADCBuffer();

            const float voltage = packet.bytes[0] * (3.3f / 4095.0f) * VOLTAGE_DIVIDER_RATIO;
            const float currentVolts = packet.bytes[1] * (3.3f / 4095.0f);
            const float currentAmps = currentVolts / CURRENT_SENSOR_SCALE;

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

    void dShotGeneration() { //merge with imu control loop
        //populate local buffers via helper and trigger the transfer
        //divide by 2000 as dshot frames are expected to be between 0 and 1 not 0 and 2000
        DShotFrame m1 = generateDShotFrame(drone.motorOutput[0] / 2000.0f);
        DShotFrame m2 = generateDShotFrame(drone.motorOutput[1] / 2000.0f);
        DShotFrame m3 = generateDShotFrame(drone.motorOutput[2] / 2000.0f);
        DShotFrame m4 = generateDShotFrame(drone.motorOutput[3] / 2000.0f);

        for (int i = 0; i < 17; i++) {
            dshotBuffer1[i] = m1.bits[i]/2; //tim4 is twice as slow
            dshotBuffer3[i] = m3.bits[i]/2;

            dshotBuffer2[i] = m2.bits[i];
            dshotBuffer4[i] = m4.bits[i];
        }

#ifdef DEBUG
        printToUSART("DShot Generation: \n");
        printToUSART(" [DSHOT] Mx Values: \n");
        //figure out a way to print out the motor outputs without it lagging out everything
        printToUSART("  m1: "); /*printToUSART(m1);*/ printToUSART("\n");
        printToUSART("  m2: "); /*printToUSART(m2);*/ printToUSART("\n");
        printToUSART("  m3: "); /*printToUSART(m3);*/ printToUSART("\n");
        printToUSART("  m4: "); /*printToUSART(m4);*/ printToUSART("\n\n");
#endif

        startMotor1_DMATransfer();
        startMotor2_DMATransfer();
        startMotor3_DMATransfer();
        startMotor4_DMATransfer();
    }

    void flightStateMachine() { //add keybinds for actions where applicable and FIX the drone throttle placement for arming
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
                            setEventMessage(drone.warnMSG, WARN::ARMED_ANGLE);
                        }
                    } else {
                        drone.currentSystemState = FlightState::WARN;
                        setEventMessage(drone.warnMSG, WARN::ARMED_THROTTLE);
                    }
                }
                break;
            case FlightState::ARMED:
                if (!armed) {
                    drone.currentSystemState = FlightState::DISARMED; //instant disarm from switch bool
                }
                break;
            case FlightState::WARN:
                printToUSART(drone.warnMSG); //printing to osd is automatic
                //need to add blackboxlogging
                break;
            case FlightState::FAILSAFE:
                printToUSART(drone.failsafeMSG); //printing to osd is automatic
                //need to add blackboxlogging
                break;
            case FlightState::ERROR:
                printToUSART(drone.errorMSG); //printing to osd is automatic
                //need to add blackboxlogging
                break;
        }

        printToUSART("Flight State Machine: \n");
        printToUSART(" [FSM] System State: \n");
        printToUSART("  State: "); printToUSART(to_string(drone.currentSystemState).c_str()); printToUSART("\n\n");

        printToUSART(" [FSM] Errors: \n");
        printToUSART("  Warn msg: "); printToUSART(drone.warnMSG); printToUSART("\n");
        printToUSART("  Error msg: "); printToUSART(drone.errorMSG); printToUSART("\n");
        printToUSART("  Failsafe msg: "); printToUSART(drone.failsafeMSG); printToUSART("\n\n");
    }

    //master task for imu
    void imuControlLoop() { // one interrupt-zone task, triggered by real IMU DRDY eventually
        //enable the timer update DMA requests once at boot
        currentBoardConfig.motor_timer->DIER |= TIM_DIER_UDE; //merge with dshot

        yieldCurrentTask();

        while (true) {
            printToUSART("[IMU-CL] Start: \n");
            readIMUDataRegisters();
            stateEstimation();
            flightLoop();
            dShotGeneration();
            printToUSART("[IMU-CL] End \n\n");

            yieldCurrentTask();
        }
    }

    void crsfControlLoop() {
        yieldCurrentTask();

        while (true) {
            printToUSART("[CRSF-CL] Start: \n");
            crsfParsing();
            flightStateMachine();
            printToUSART("[CRSF-CL] End \n\n");

            yieldCurrentTask();
        }
    }

    //main scheduled tasks
    void iwdgTask() { //add a stopping sequence to reboot the fc
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

        constexpr uint32_t BLINK_WINDOW_MS = 166; //declare consts inside of task stack instead of redeclaring during runtime
        constexpr uint32_t BLINK_PULSE_MS  = 20;

        yieldCurrentTask();

        while (true) {

            //LED logic:
            //ARM = ON
            //DISARM = OFF
            //ISSUE = FLASH
            if (drone.currentSystemState == FlightState::DISARMED) {
                currentBoardConfig.status_led_port->BSRR = static_cast<uint32_t>(currentBoardConfig.status_led_pin) << 16; //bit shit 16 to the left to set low

            } else if (drone.currentSystemState == FlightState::ARMED) {
                currentBoardConfig.status_led_port->BSRR = currentBoardConfig.status_led_pin;

            } else if (drone.currentSystemState == FlightState::WARN ||
                drone.currentSystemState == FlightState::FAILSAFE ||
                drone.currentSystemState == FlightState::ERROR
               )
            {
                if (globalSystemTicks % BLINK_WINDOW_MS < BLINK_PULSE_MS) { currentBoardConfig.status_led_port->BSRR = static_cast<uint32_t>(currentBoardConfig.status_led_pin) << 16; }
                else { currentBoardConfig.status_led_port->BSRR = currentBoardConfig.status_led_pin; }

            } else {
                currentBoardConfig.status_led_port->BSRR = static_cast<uint32_t>(currentBoardConfig.status_led_pin) << 16; //default state low
            }

            //buzzer logic
            //ARM = OFF
            //ISSUE = ON

            if (drone.currentSystemState == FlightState::FAILSAFE ||
                drone.currentSystemState == FlightState::ERROR //do not do warn here as it is not fatal
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

    void gpsParser() { //needs injector to be fixed
        uint8_t bufferIdx = 0;

        yieldCurrentTask();

        while (true) {

            GPSPacket sentenceBuffer = populateGPSBuffer();

            while (currentBoardConfig.gps_uart->ISR & USART_ISR_RXNE) { //while recieving new bytes

                char c = static_cast<char>(currentBoardConfig.gps_uart->RDR); //gets a single byte from read

                if (c == '$') { //reset sequence and start new transmit
                    bufferIdx = 0;
                    sentenceBuffer.bytes[bufferIdx++] = c;
                }

                else if (c == '\r' || c == '\n') {
                    if (bufferIdx > 0) {
                        sentenceBuffer.bytes[bufferIdx] = '\0';
                        if (std::strncmp(sentenceBuffer.bytes, "$GNGGA", 6) == 0 || std::strncmp(sentenceBuffer.bytes, "$GPGGA", 6) == 0) { //ensure gnss or gps signal via string compare
                            int32_t  localLat = 0; //reset locals
                            int32_t  localLon = 0;
                            uint8_t  localSats = 0;
                            uint16_t localAlt = 0;
                            uint8_t  fixQuality = 0;

                            char* token = strtok(sentenceBuffer.bytes, ","); //split sentence by commas
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
                        } else if (std::strncmp(sentenceBuffer.bytes, "$GNRMC", 6) == 0 || std::strncmp(sentenceBuffer.bytes, "$GPRMC", 6) == 0) {
                            char* token = strtok(sentenceBuffer.bytes, ","); //split sentence by commas
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

                else if (bufferIdx < sizeof(sentenceBuffer.bytes) - 1) {
                    sentenceBuffer.bytes[bufferIdx++] = c;
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

    void usbCLI() { //cli is allowed to be a blocking function because it is a super low priority task only used during drone setup
        static char commandBuffer[64];
        static uint8_t cmdIdx = 0;

        //enable reciever, transmitter, and peripheral
        currentBoardConfig.cli_uart->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);

        printToCLI("\r\n=================================\r\n");
        printToCLI("   DRONE RTOS FLIGHT TERMINAL   \r\n");
        printToCLI("=================================\r\n# ");

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
                        printToCLI("\r\n");

                        //points to the next character after the last space
                        const char* numPtr = std::strrchr(commandBuffer, ' ');
                        float parsedVal = numPtr ? static_cast<float>(std::atof(numPtr + 1)) : 0.0f;

                        //system status
                        if (std::strncmp(commandBuffer, "status", 6) == 0) {
                            printToCLI("[SYSTEM STATUS]\r\n");
                            printToCLI("  Battery Voltage: "); printToCLI(drone.batteryVoltage); printToCLI(" V\r\n");
                            printToCLI("  Flight State: "); printToCLI(static_cast<uint32_t>(drone.currentSystemState)); printToCLI("\r\n");
                            printToCLI("  Armed: "); printToCLI(drone.armed); printToCLI("\r\n");
                            printToCLI("  Sats Locked: "); printToCLI(static_cast<uint32_t>(drone.satellites)); printToCLI("\r\n");
                        }

                        //tune kp
                        else if (std::strncmp(commandBuffer, "set kp --roll", 13) == 0 || std::strncmp(commandBuffer, "set kp -r", 9) == 0) {
                            Kp_roll = parsedVal;
                            printToCLI("Kp_roll updated to: "); printToCLI(Kp_roll); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set kp --pitch", 14) == 0 || std::strncmp(commandBuffer, "set kp -p", 9) == 0) {
                            Kp_pitch = parsedVal;
                            printToCLI("Kp_pitch updated to: "); printToCLI(Kp_pitch); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set kp --yaw", 12) == 0 || std::strncmp(commandBuffer, "set kp -y", 9) == 0) {
                            Kp_yaw = parsedVal;
                            printToCLI("Kp_yaw updated to: "); printToCLI(Kp_yaw); printToCLI("\r\n");
                        }

                        //tune ki
                        else if (std::strncmp(commandBuffer, "set ki --roll", 13) == 0 || std::strncmp(commandBuffer, "set ki -r", 9) == 0) {
                            Ki_roll = parsedVal;
                            printToCLI("Ki_roll updated to: "); printToCLI(Ki_roll); printToCLI("\r\n");
                        }

                        else if (std::strncmp(commandBuffer, "set ki --pitch", 14) == 0 || std::strncmp(commandBuffer, "set ki -p", 9) == 0) {
                            Ki_pitch = parsedVal;
                            printToCLI("Ki_pitch updated to: "); printToCLI(Ki_pitch); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set ki --yaw", 12) == 0 || std::strncmp(commandBuffer, "set ki -y", 9) == 0) {
                            Ki_yaw = parsedVal;
                            printToCLI("Ki_yaw updated to: "); printToCLI(Ki_yaw); printToCLI("\r\n");
                        }

                        //tune kd
                        else if (std::strncmp(commandBuffer, "set kd --roll", 13) == 0 || std::strncmp(commandBuffer, "set kd -r", 9) == 0) {
                            Kd_roll = parsedVal;
                            printToCLI("Kd updated to: "); printToCLI(Kd_roll); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set kd --pitch", 14) == 0 || std::strncmp(commandBuffer, "set kd -p", 9) == 0) {
                            Kd_pitch = parsedVal;
                            printToCLI("Kd_pitch updated to: "); printToCLI(Kd_pitch); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set kd --yaw", 12) == 0 || std::strncmp(commandBuffer, "set kd -y", 9) == 0) {
                            Kd_yaw = parsedVal;
                            printToCLI("Kd_yaw updated to: "); printToCLI(Kd_yaw); printToCLI("\r\n");
                        }

                        //inspect registers
                        else if (std::strncmp(commandBuffer, "get --pid", 9) == 0 || std::strncmp(commandBuffer, "get -p", 6) == 0) {
                            printToCLI("[PID GAINS]\r\n");
                            printToCLI("  Kp: "); printToCLI(Kp_roll); printToCLI("\r\n");
                            printToCLI("  Ki: "); printToCLI(Ki_roll); printToCLI("\r\n");
                            printToCLI("  Kd: "); printToCLI(Kd_roll); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "get --imu", 9) == 0 || std::strncmp(commandBuffer, "get -i", 6) == 0) {
                            printToCLI("[IMU ATTITUDE]\r\n");
                            printToCLI("  Roll: "); printToCLI(drone.attitudeRoll); printToCLI("\r\n");
                            printToCLI("  Pitch: "); printToCLI(drone.attitudePitch); printToCLI("\r\n");
                            printToCLI("  Yaw: "); printToCLI(drone.attitudeYaw); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "get --gps", 9) == 0 || std::strncmp(commandBuffer, "get -g", 6) == 0) {
                            printToCLI("[GPS DATA]\r\n");
                            printToCLI("  Lat: "); printToCLI(drone.latitude); printToCLI("\r\n");
                            printToCLI("  Lon: "); printToCLI(drone.longitude); printToCLI("\r\n");
                            printToCLI("  Alt: "); printToCLI(static_cast<uint32_t>(drone.altitude)); printToCLI(" m\r\n");
                            printToCLI("  Sats: "); printToCLI(static_cast<uint32_t>(drone.satellites)); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "get --motors", 12) == 0 || std::strncmp(commandBuffer, "get -m", 6) == 0) {
                            printToCLI("[DSHOT MOTOR OUTPUTS]\r\n");
                            printToCLI("  M1: "); printToCLI(drone.motorOutput[0]); printToCLI("\r\n");
                            printToCLI("  M2: "); printToCLI(drone.motorOutput[1]); printToCLI("\r\n");
                            printToCLI("  M3: "); printToCLI(drone.motorOutput[2]); printToCLI("\r\n");
                            printToCLI("  M4: "); printToCLI(drone.motorOutput[3]); printToCLI("\r\n");
                        }

                        //system reboot
                        else if (std::strncmp(commandBuffer, "reboot", 6) == 0) {
                            printToCLI("Rebooting MCU...\r\n");
                            NVIC_SystemReset();
                        }

                        //help menu
                        else if (std::strncmp(commandBuffer, "help", 4) == 0) {
                            printToCLI("Available Commands:\r\n");
                            printToCLI("  status                                                          - System telemetry & state\r\n");
                            printToCLI("  set kp <--roll, --pitch, --yaw, -r, -p, -y> <val>               - Update Kp Values\r\n");
                            printToCLI("  set ki <--roll, --pitch, --yaw, -r, -p, -y> <val>               - Update Ki Values\r\n");
                            printToCLI("  set kd <--roll, --pitch, --yaw, -r, -p, -y> <val>               - Update Kd Values\r\n");
                            printToCLI("  get <--pid, -p>                                                 - Print all PID gains\r\n");
                            printToCLI("  get <--imu, -i>                                                 - Print orientation\r\n");
                            printToCLI("  get <--gps, -g>                                                 - Print GPS coordinates\r\n");
                            printToCLI("  get <--motors, -m>                                              - Print DShot outputs\r\n");
                            printToCLI("  reboot                                                          - Software reset\r\n");
                            printToCLI("  set rate --roll <--max, --center, --expo, -m, -c, -e> <val>     - Update Roll Ratesr\r\n");
                            printToCLI("  set rate --pitch <--max, --center, --expo, -m, -c, -e> <val>    - Update Pitch Ratesr\r\n");
                            printToCLI("  set rate --yaw <--max, --center, --expo, -m, -c, -e> <val>      - Update Yaw Ratesr\r\n");
                            printToCLI("  get <--imu-raw, -ir>                                            - Get IMU Values\r\n");
                            printToCLI("  get <--imu-cal, -ic>                                            - Get IMU Calibration Values\r\n");
                            printToCLI("  get <--baro, -b>                                                - Get Baro Values\r\n");
                            printToCLI("  get <--crsf, -c>                                                - Get CRSF Values\r\n");
                            printToCLI("  get <--batt, -bat>                                              - Get Battery Values\r\n");
                            printToCLI("  get <--sys, -s>                                                 - Get System State Values\r\n");
                        }

                        //set advanced max rates
                        else if (std::strncmp(commandBuffer, "set rate --roll --max", 21) == 0 || std::strncmp(commandBuffer, "set rate -r -m", 14) == 0) {
                            rollRateCfg.maxRate = parsedVal;
                            printToCLI("Roll maxRate updated to: "); printToCLI(rollRateCfg.maxRate); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set rate --pitch --max", 22) == 0 || std::strncmp(commandBuffer, "set rate -p -m", 14) == 0) {
                            pitchRateCfg.maxRate = parsedVal;
                            printToCLI("Pitch maxRate updated to: "); printToCLI(pitchRateCfg.maxRate); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set rate --yaw --max", 20) == 0 || std::strncmp(commandBuffer, "set rate -y -m", 14) == 0) {
                            yawRateCfg.maxRate = parsedVal;
                            printToCLI("Yaw maxRate updated to: "); printToCLI(yawRateCfg.maxRate); printToCLI("\r\n");
                        }

                        //set advanced center rates
                        else if (std::strncmp(commandBuffer, "set rate --roll --center", 24) == 0 || std::strncmp(commandBuffer, "set rate -r -c", 14) == 0) {
                            rollRateCfg.centerRate = parsedVal;
                            printToCLI("Roll centerRate updated to: "); printToCLI(rollRateCfg.centerRate); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set rate --pitch --center", 25) == 0 || std::strncmp(commandBuffer, "set rate -p -c", 14) == 0) {
                            pitchRateCfg.centerRate = parsedVal;
                            printToCLI("Pitch centerRate updated to: "); printToCLI(pitchRateCfg.centerRate); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set rate --yaw --center", 23) == 0 || std::strncmp(commandBuffer, "set rate -y -c", 14) == 0) {
                            yawRateCfg.centerRate = parsedVal;
                            printToCLI("Yaw centerRate updated to: "); printToCLI(yawRateCfg.centerRate); printToCLI("\r\n");
                        }

                        //set advanced expo rates
                        else if (std::strncmp(commandBuffer, "set rate --roll --expo", 22) == 0 || std::strncmp(commandBuffer, "set rate -r -e", 14) == 0) {
                            rollRateCfg.expo = parsedVal;
                            printToCLI("Roll expo updated to: "); printToCLI(rollRateCfg.expo); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set rate --pitch --expo", 23) == 0 || std::strncmp(commandBuffer, "set rate -p -e", 14) == 0) {
                            pitchRateCfg.expo = parsedVal;
                            printToCLI("Pitch expo updated to: "); printToCLI(pitchRateCfg.expo); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set rate --yaw --expo", 21) == 0 || std::strncmp(commandBuffer, "set rate -y -e", 14) == 0) {
                            yawRateCfg.expo = parsedVal;
                            printToCLI("Yaw expo updated to: "); printToCLI(yawRateCfg.expo); printToCLI("\r\n");
                        }

                        //update crsf channels
                        else if (std::strncmp(commandBuffer, "set channel --roll", 18) == 0 || std::strncmp(commandBuffer, "set channel -r", 14) == 0) {
                            assignChannel(ROLL_CH, parsedVal);
                            printToCLI("Roll channel updated to: "); printToCLI(*ROLL_CH); printToCLI("\r\n");
                        }

                        else if (std::strncmp(commandBuffer, "set channel --pitch", 19) == 0 || std::strncmp(commandBuffer, "set channel -p", 14) == 0) {
                            assignChannel(PITCH_CH, parsedVal);
                            printToCLI("Pitch channel updated to: "); printToCLI(*PITCH_CH); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set channel --yaw", 17) == 0 || std::strncmp(commandBuffer, "set channel -y", 14) == 0) {
                            assignChannel(YAW_CH, parsedVal);
                            printToCLI("Yaw channel updated to: "); printToCLI(*YAW_CH); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set channel --throttle", 22) == 0 || std::strncmp(commandBuffer, "set channel -t", 14) == 0) {
                            assignChannel(THROTTLE_CH, parsedVal);
                            printToCLI("Throttle channel updated to: "); printToCLI(*THROTTLE_CH); printToCLI("\r\n");
                        }
                        else if (std::strncmp(commandBuffer, "set channel --armed", 19) == 0 || std::strncmp(commandBuffer, "set channel -a", 14) == 0) {
                            assignChannel(ARMED_CH, parsedVal);
                            printToCLI("Armed channel updated to: "); printToCLI(*ARMED_CH); printToCLI("\r\n");
                        }

                        //offset imu calculations
                        else if (std::strncmp(commandBuffer, "set imu --orientation", 19) == 0 || std::strncmp(commandBuffer, "set imu -o", 10) == 0) {
                            float rad = parsedVal * (std::acos(-1)/ 180.0f); //acos(-1) is pi
                            imu_cos_theta = std::cos(rad);
                            imu_sin_theta = std::sin(rad);
                        }

                        //raw imu sensor state
                        else if (std::strncmp(commandBuffer, "get --imu-raw", 13) == 0 || std::strncmp(commandBuffer, "get -ir", 7) == 0) {
                            printToCLI("[RAW IMU DATA]\r\n");
                            printToCLI("  Accel X: "); printToCLI(drone.accelRaw[0]); printToCLI(" g\r\n");
                            printToCLI("  Accel Y: "); printToCLI(drone.accelRaw[1]); printToCLI(" g\r\n");
                            printToCLI("  Accel Z: "); printToCLI(drone.accelRaw[2]); printToCLI(" g\r\n");
                            printToCLI("  Gyro X: "); printToCLI(drone.gyroRaw[0]); printToCLI(" deg/s\r\n");
                            printToCLI("  Gyro Y: "); printToCLI(drone.gyroRaw[1]); printToCLI(" deg/s\r\n");
                            printToCLI("  Gyro Z: "); printToCLI(drone.gyroRaw[2]); printToCLI(" deg/s\r\n");
                        }

                        //calibrated imu & attitude state
                        else if (std::strncmp(commandBuffer, "get --imu-cal", 13) == 0 || std::strncmp(commandBuffer, "get -ic", 7) == 0) {
                            printToCLI("[CALIBRATED IMU & ATTITUDE]\r\n");
                            printToCLI("  Calibrated Status: "); printToCLI(drone.calibrated ? "YES" : "NO"); printToCLI("\r\n");
                            printToCLI("  Gyro Cal X: "); printToCLI(drone.gyroCalibrated[0]); printToCLI(" deg/s\r\n");
                            printToCLI("  Gyro Cal Y: "); printToCLI(drone.gyroCalibrated[1]); printToCLI(" deg/s\r\n");
                            printToCLI("  Gyro Cal Z: "); printToCLI(drone.gyroCalibrated[2]); printToCLI(" deg/s\r\n");
                            printToCLI("  Estimated Roll "); printToCLI(drone.attitudeRoll); printToCLI(" deg\r\n");
                            printToCLI("  Estimated Pitch: "); printToCLI(drone.attitudePitch); printToCLI(" deg\r\n");
                            printToCLI("  Estimated Yaw: "); printToCLI(drone.attitudeYaw); printToCLI(" deg\r\n");
                        }

                        //barometer sensor state
                        else if (std::strncmp(commandBuffer, "get --baro", 10) == 0 || std::strncmp(commandBuffer, "get -b", 6) == 0) {
                            printToCLI("[BAROMETER SENSOR]\r\n");
                            printToCLI("  Altitude: "); printToCLI(drone.altitude); printToCLI(" m\r\n");
                        }

                        //crsf radio receiver state
                        else if (std::strncmp(commandBuffer, "get --crsf", 10) == 0 || std::strncmp(commandBuffer, "get -c", 6) == 0) {
                            printToCLI("[CRSF LINK STATUS]\r\n");
                            printToCLI("  Armed State: "); printToCLI(drone.armed ? "ARMED" : "DISARMED"); printToCLI("\r\n");
                            printToCLI("  Roll Stick: "); printToCLI(drone.rollStick); printToCLI("\r\n");
                            printToCLI("  Pitch Stick: "); printToCLI(drone.pitchStick); printToCLI("\r\n");
                            printToCLI("  Yaw Stick: "); printToCLI(drone.yawStick); printToCLI("\r\n");
                            printToCLI("  Throttle Stick: "); printToCLI(drone.throttleStick); printToCLI("\r\n");
                            printToCLI("  Last Frame Age: "); printToCLI(globalSystemTicks - drone.lastValidRCFrameTime); printToCLI(" ms ago\r\n");
                        }

                        //battery & power monitoring
                        else if (std::strncmp(commandBuffer, "get --batt", 10) == 0 || std::strncmp(commandBuffer, "get -bat", 8) == 0) {
                            printToCLI("[BATTERY & POWER MONITOR]\r\n");
                            printToCLI("  Voltage: "); printToCLI(drone.batteryVoltage); printToCLI(" V\r\n");
                            printToCLI("  Current: "); printToCLI(drone.batteryCurrent); printToCLI(" A\r\n");
                            printToCLI("  Capacity Used: "); printToCLI(drone.batteryCapacityUsed); printToCLI(" mAh\r\n");
                            printToCLI("  Battery Level: "); printToCLI(static_cast<uint32_t>(drone.batteryPerecent)); printToCLI(" %\r\n");
                        }

                        //system & stack health summary
                        else if (std::strncmp(commandBuffer, "get --sys", 9) == 0 || std::strncmp(commandBuffer, "get -s", 6) == 0) {
                            printToCLI("[RTOS SYSTEM HEALTH]\r\n");
                            printToCLI("  System State: "); printToCLI(to_string(drone.currentSystemState).c_str()); printToCLI("\r\n");
                            printToCLI("  System Ticks: "); printToCLI(globalSystemTicks); printToCLI(" ms\r\n");
                            printToCLI("  Active Tasks: "); printToCLI(static_cast<uint32_t>(activeTasks)); printToCLI("\r\n");

                            printToCLI("  Task Stack High-Water Marks:\r\n");
                            for (int i = 0; i < activeTasks; i++) {
                                const uint32_t freeWords = getUnusedStackWords(i);
                                const uint32_t totalSpace = taskControlBlocks[i].stackSizeWords;
                                const uint32_t usedPercentage = 100 - ((freeWords * 100) / totalSpace);

                                printToCLI("    - "); printToCLI(taskControlBlocks[i].taskName); printToCLI(": ");
                                printToCLI(usedPercentage); printToCLI("% used\r\n");
                            }
                        }

                        else {
                            printToCLI("Unknown command. Type 'help' for options.\r\n");
                        }
                    }

                    //reset buffer and output fresh prompt
                    cmdIdx = 0;
                    printToCLI("# ");
                }
                //store ASCII characters
                else if (cmdIdx < sizeof(commandBuffer) - 1 && c >= 32 && c <= 126) {
                    commandBuffer[cmdIdx++] = c;
                }
            }

            yieldCurrentTask();
        }
    }

    //flags
    // MSP_DP_RELEASE: 0x00;
    // MSP_DP_WRITE: 0x01;
    // MSP_DP_DRAW: 0x04;

    //position buffer data (for cli config)
    //row, col
    uint8_t batVolPos[2] = {15, 22};
    uint8_t batCurPos[2] = {16, 22};
    uint8_t batMahPos[2] = {17, 22};

    uint8_t gpsLatPos[2] = {18, 22};
    uint8_t gpsLonPos[2] = {19, 22};
    uint8_t gpsAltPos[2] = {20, 22};
    uint8_t gpsSpdPos[2] = {21, 22};
    uint8_t gpsSatPos[2] = {22, 22};

    uint8_t drnMsgPos[2] = {23, 22};

    void osdUpdate() {

        char osdBuffer[32];
        yieldCurrentTask(); //yield after init setup

        while (true) {
            //things to log:
            //1. battery stats (voltage, mah left, and current)
            //2. Lon/lat, direction to home (optional), speed, # of sattelites
            //3. Warnings, errors, and failsafe messages, and flight state

            uint8_t releaseCmd = MSP_DP_RELEASE; //convert to uint
            sendMSPFrame(182, &releaseCmd, 1);

            //battery
            //special format for buffer size (make sure to not exceed 32 chars)
            //snprintf(buf, size(buf), "TEXT: %FLAG", contents);
            snprintf(osdBuffer, sizeof(osdBuffer), "BAT: %.1fV", drone.batteryVoltage);  //voltage
            writeOSDString(batVolPos[0], batVolPos[1], osdBuffer);

            snprintf(osdBuffer, sizeof(osdBuffer), "BAT: %.1fA", drone.batteryCurrent); //current (amps)
            writeOSDString(batCurPos[0], batCurPos[1], osdBuffer);

            float mahLeft = TOTAL_BATTERY_MAH - drone.batteryCapacityUsed;
            snprintf(osdBuffer, sizeof(osdBuffer), "BAT: %.0fmAh", mahLeft);
            writeOSDString(batMahPos[0], batMahPos[1], osdBuffer);

            //gps
            snprintf(osdBuffer, sizeof(osdBuffer), "LAT: %ld", drone.latitude);  //latitude
            writeOSDString(gpsLatPos[0], gpsLatPos[1], osdBuffer);

            snprintf(osdBuffer, sizeof(osdBuffer), "LON: %ld", drone.longitude);  //longitude
            writeOSDString(gpsLonPos[0], gpsLonPos[1], osdBuffer);

            snprintf(osdBuffer, sizeof(osdBuffer), "ALT: %d", drone.altitude);  //speed
            writeOSDString(gpsAltPos[0], gpsAltPos[1], osdBuffer);

            snprintf(osdBuffer, sizeof(osdBuffer), "SPD (KPH): %d", drone.groundSpeed);  //speed
            writeOSDString(gpsSpdPos[0], gpsSpdPos[1], osdBuffer);

            snprintf(osdBuffer, sizeof(osdBuffer), "SAT#: %d", drone.satellites);  //satellites
            writeOSDString(gpsSatPos[0], gpsSatPos[1], osdBuffer);

            if (drone.currentSystemState == FlightState::ERROR) { writeOSDString(drnMsgPos[0], drnMsgPos[1], drone.errorMSG); }
            else if (drone.currentSystemState == FlightState::WARN) { writeOSDString(drnMsgPos[0], drnMsgPos[1], drone.warnMSG); }
            else if (drone.currentSystemState == FlightState::FAILSAFE) { writeOSDString(drnMsgPos[0], drnMsgPos[1], drone.failsafeMSG); }

            uint8_t drawCmd = MSP_DP_DRAW; //convert to uint
            sendMSPFrame(182, &drawCmd, 1);


    #if DEBUG && SIMULATION
            printToUSART("Osd Update: \n");

            // --- Battery ---
            printToUSART(" [OU] Battery Values:\n");

            printToUSART("  Voltage, R"); printToUSART(batVolPos[0]);
            printToUSART(", C"); printToUSART(batVolPos[1]);
            printToUSART(", "); printToUSART(drone.batteryVoltage); printToUSART("\n");

            printToUSART("  Current, R"); printToUSART(batCurPos[0]);
            printToUSART(", C"); printToUSART(batCurPos[1]);
            printToUSART(", "); printToUSART(drone.batteryCurrent); printToUSART("\n");

            printToUSART("  mAh Left, R"); printToUSART(batMahPos[0]);
            printToUSART(", C"); printToUSART(batMahPos[1]);
            printToUSART(", "); printToUSART(mahLeft); printToUSART("\n\n");

            // --- GPS ---
            printToUSART(" [OU] GPS Values:\n");

            printToUSART("  Latitude, R"); printToUSART(gpsLatPos[0]);
            printToUSART(", C"); printToUSART(gpsLatPos[1]);
            printToUSART(", "); printToUSART(drone.latitude); printToUSART("\n");

            printToUSART("  Longitude, R"); printToUSART(gpsLonPos[0]);
            printToUSART(", C"); printToUSART(gpsLonPos[1]);
            printToUSART(", "); printToUSART(drone.longitude); printToUSART("\n");

            printToUSART("  Altitude, R"); printToUSART(gpsAltPos[0]);
            printToUSART(", C"); printToUSART(gpsAltPos[1]);
            printToUSART(", "); printToUSART(drone.altitude); printToUSART("\n");

            printToUSART("  Speed, R"); printToUSART(gpsSpdPos[0]);
            printToUSART(", C"); printToUSART(gpsSpdPos[1]);
            printToUSART(", "); printToUSART(drone.groundSpeed); printToUSART("\n");

            printToUSART("  Sats, R"); printToUSART(gpsSatPos[0]);
            printToUSART(", C"); printToUSART(gpsSatPos[1]);
            printToUSART(", "); printToUSART(drone.satellites); printToUSART("\n\n");

            // --- Message ---
            printToUSART(" [OU] Message Values:\n");

            const char* activeMsg = "NONE";
            if (drone.currentSystemState == FlightState::ERROR) { activeMsg = drone.errorMSG; }
            else if (drone.currentSystemState == FlightState::WARN) { activeMsg = drone.warnMSG; }
            else if (drone.currentSystemState == FlightState::FAILSAFE) { activeMsg = drone.failsafeMSG; }

            printToUSART("  System Msg, R"); printToUSART(drnMsgPos[0]);
            printToUSART(", C"); printToUSART(drnMsgPos[1]);
            printToUSART(", "); printToUSART(activeMsg); printToUSART("\n\n");
    #endif

            yieldCurrentTask();
        }
    }

    void respondToVTX() {
        uint8_t state = 0;
        uint8_t size = static_cast<uint8_t>(-1);
        uint8_t command = static_cast<uint8_t>(-1);
        uint8_t checksum = 0;
        uint8_t payload[32];
        uint8_t payloadIndex = static_cast<uint8_t>(-1);

        yieldCurrentTask();

        while (true) {
            //create checksum in payload
            OSDPacket mspPacket = populateOSDBuffer();

            #ifdef DEBUG
                printToUSART("Respond to VTX: \n");
            #endif

            for (int i = 0; i < mspPacket.length; i++) {

                uint8_t byte = mspPacket.bytes[i];

                switch (state) {
                    case 0:
                        #ifdef DEBUG
                            printToUSART(" [RVTX] Header\n");
                        #endif
                        if (byte == '$') { state = 1; }
                        else { state = 0;}
                        break;
                    case 1:
                        if (byte == 'M') {
                            state = 2;

                            #ifdef DEBUG
                                printToUSART("  Found '$'\n");
                            #endif
                        }
                        else { state = 0;}
                        break;

                    case 2:
                        if (byte == '<') {
                            #ifdef DEBUG
                                printToUSART("  Found 'M'\n");
                            #endif

                            state = 3;
                            checksum = 0;
                        }
                        else { state = 0;}
                        break;
                    case 3:
                        #ifdef DEBUG
                            printToUSART("  Found '<'\n");
                        #endif
                        size = byte;
                        checksum ^= byte;
                        state = 4;
                        break;
                    case 4:
                        command = byte;
                        checksum ^= byte;
                        payloadIndex = 0;
                        if (size > 0) {
                            state = 5;
                        } else {
                            state = 6;
                        }
                        break;
                    case 5:
                        payload[payloadIndex] = byte;
                        checksum ^= byte;
                        payloadIndex++;
                        if (payloadIndex == size) { state = 6; }
                        break;
                    case 6:
                        if (checksum == byte) {
                            //start command state machine
                            switch (command) {
                                case 2: {
                                    //MSP_FC_VARIANT
                                    uint8_t variantArr[4] = {'B', 'T', 'F', 'L'};
                                    sendMSPResponse(2, variantArr, 4);

                                    #ifdef DEBUG
                                        printToUSART(" [RVTX] Command 2 (FC Variant): \n");
                                        printToUSART("  Responding: BTFL\n\n");
                                    #endif
                                    break;
                                }
                                case 30: {
                                    //MSP_STATUS
                                    uint8_t statusArr[11] = {0};
                                    for (int i = 0; i < 11; i++) { statusArr[i] = 0; }

                                    statusArr[6] = drone.armed ? 1 : 0;
                                    sendMSPResponse(30, statusArr, 11);
                                    #ifdef DEBUG
                                        printToUSART(" [RVTX] Command 30 (Status): \n");
                                        printToUSART("  Armed: ");
                                        printToUSART(drone.armed ? "1" : "0");
                                        printToUSART("\n\n");
                                    #endif

                                    break;
                                }
                                case 105: {
                                    //MSP_RC
                                    uint8_t rcArr[16] = {0};

                                    auto rollPWM     = static_cast<uint16_t>((drone.rollStick * 500.0f) + 1500.0f);
                                    auto pitchPWM    = static_cast<uint16_t>((drone.pitchStick * 500.0f) + 1500.0f);
                                    auto yawPWM      = static_cast<uint16_t>((drone.yawStick * 500.0f) + 1500.0f);
                                    auto throttlePWM = static_cast<uint16_t>((drone.throttleStick * 1000.0f) + 1000.0f);

                                    //bit shift into uint8
                                    //& 0xFF limits to 8 bits max
                                    rcArr[0] = rollPWM & 0xFF; rcArr[1] = (rollPWM >> 8) & 0xFF;
                                    rcArr[2] = pitchPWM & 0xFF; rcArr[3] = (pitchPWM >> 8) & 0xFF;
                                    rcArr[4] = yawPWM & 0xFF; rcArr[5] = (yawPWM >> 8) & 0xFF;
                                    rcArr[6] = throttlePWM & 0xFF; rcArr[7] = (throttlePWM >> 8) & 0xFF;

                                    sendMSPResponse(105, rcArr, 16);

                                    #ifdef DEBUG
                                        printToUSART(" [RVTX] Command 105 (RC Channels): \n");
                                        printToUSART("  Roll PWM: "); printToUSART(rollPWM); printToUSART("\n");
                                        printToUSART("  Pitch PWM: "); printToUSART(pitchPWM); printToUSART("\n");
                                        printToUSART("  Yaw PWM: "); printToUSART(yawPWM); printToUSART("\n");
                                        printToUSART("  Throttle PWM: "); printToUSART(throttlePWM); printToUSART("\n\n");
                                    #endif

                                    break;
                                }
                            }

                            state = 0; //packet is done whether or not checksum failed
                        } else { state = 0; }
                        break;

                    default: break;
                    //will break automatically once packet is done reading
                }
            }

            yieldCurrentTask();
        }
    }

    volatile float p0 = 101325.0f; //adjust in cli

    void readBarometerRegisters() { //needs injectors to be fixed

    #ifdef SIMULATION
        BaroTrim trim = initMockBarometer();
    #else
        BaroTrim trim = initBarometer();
    #endif

        //trim scaling:
        static const float parT1 = static_cast<float>(trim.T1) * 256.0f; //trim.T1 / 2^-8
        static const float parT2 = static_cast<float>(trim.T2) / 1073741824.0f; //trim.T2 / 2^30
        static const float parT3 = static_cast<float>(trim.T3) / 281474976710656.0f; //trim.T3 / 2^48

        static const float parP1 = (trim.P1-std::pow(2.0f, 14.0f))/std::pow(2.0f, 20.0f);
        static const float parP2 = (trim.P2-std::pow(2.0f, 14.0f))/std::pow(2.0f, 29.0f);
        static const float parP3 = trim.P3/std::pow(2.0f, 32.0f);
        static const float parP4 = trim.P4/std::pow(2.0f, 37.0f);
        static const float parP5 = trim.P5/std::pow(2.0f, -3.0f);
        static const float parP6 = trim.P6/std::pow(2.0f, 6.0f);

        static const float parP7 = static_cast<float>(trim.P7) / 256.0f; // trim.P7 / 2^8

        static const float parP8 = trim.P8/std::pow(2.0f, 15.0f);
        static const float parP9 = trim.P9/std::pow(2.0f, 48.0f);
        static const float parP10 = trim.P10/std::pow(2.0f, 48.0f);
        static const float parP11 = trim.P11/std::pow(2.0f, 65.0f);

        yieldCurrentTask();

        while (true) {

            burstBaro_DMATransfer();

            BaroRawPacket baroRawBuffer = populateBarometerBuffer();
            //uses dma to do everything
            const uint32_t rawPressure = (baroRawBuffer.bytes[0] << 16) | (baroRawBuffer.bytes[1] << 8) | (baroRawBuffer.bytes[2]); //compile the 8 bit buffer into 24 bit
            const uint32_t rawTemperature = (baroRawBuffer.bytes[3] << 16) | (baroRawBuffer.bytes[4] << 8) | (baroRawBuffer.bytes[5]);

            //calculations
            //compensate temperature:
            const float partial1 = rawTemperature - parT1;
            const float partial2 = partial1*parT2;
            const float comp_temp = partial2 + (partial1*partial1)*parT3;

            //compensate pressure:
            const float partialT1 = parP6*comp_temp;
            const float partialT2 = parP7*(comp_temp*comp_temp);
            const float partialT3 = parP8*(comp_temp*comp_temp*comp_temp);
            const float partialOut1 = parP5 + partialT1 + partialT2 + partialT3;

            const float partialP1 = parP2*comp_temp;
            const float partialP2 = parP3*(comp_temp*comp_temp);
            const float partialP3 = parP4*(comp_temp*comp_temp*comp_temp);
            const float partialOut2 = rawPressure*(parP1+partialP1 + partialP2 + partialP3);

            const float partialD1 = (rawPressure*rawPressure)*(parP9+parP10*comp_temp);
            const float partialD2 = partialD1 + (rawPressure*rawPressure*rawPressure)*parP11;

            const float comp_press = partialOut1 + partialOut2 + partialD2;

            const float altitude = 44330.0f*(1.0f-std::pow((comp_press/p0), 0.190295));

            __disable_irq();
            drone.altitude = altitude;
            __enable_irq();

    #ifdef DEBUG
            printToUSART("Read Barometer Registers\n");
            printToUSART(" [RBR] Raw Values: \n");
            printToUSART("  Raw Pressure: "); printToUSART(rawPressure); printToUSART("\n");
            printToUSART("  Raw Temperature: "); printToUSART(rawTemperature); printToUSART("\n\n");

            printToUSART(" [RBR] Calculated Values:\n");
            printToUSART("  Partial_Out_1: "); printToUSART(partialOut1); printToUSART("\n");
            printToUSART("  Partial_Out_2: "); printToUSART(partialOut2); printToUSART("\n");
            printToUSART("  Comp_temp: "); printToUSART(comp_temp); printToUSART("\n");
            printToUSART("  Comp_press: "); printToUSART(comp_press); printToUSART("\n\n");

            printToUSART(" [RBR] Final Values: \n");
            printToUSART("  Altitude: "); printToUSART(altitude); printToUSART("\n\n");
    #endif

            yieldCurrentTask();
        }
    }

    void checkStackHealth() {
        yieldCurrentTask();
        while (true) {
            printToUSART("[CSH] Task Health Stats (% Used):\n");
            for (int i = 0; i < activeTasks; i++) {
                const uint32_t freeWords = getUnusedStackWords(i);
                const uint32_t totalSpace = taskControlBlocks[i].stackSizeWords;
                const uint32_t usedPercentage = 100 - (freeWords*100)/totalSpace;

                printToUSART("  "); printToUSART(taskControlBlocks[i].taskName); printToUSART(": ");
                printToUSART(usedPercentage); printToUSART("%\n");
            }

            printToUSART("\n");
            yieldCurrentTask();
        }
    }

    void logToBlackBox() { //MUST INCLUDE FOR LOGGING ERRORS AND EVERYTHING
        //design it so that its an interrupt at one of the lowest possible levels (even lower than some of the scheduled tasks)
        //have the code call the interrupt from individual methods, similar to printtousart/printtocli
        //have a scheduled version which runs at a high speed, but is only enabled with the debug flag enabled
        yieldCurrentTask();
        while (true) {



            yieldCurrentTask();
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
            printToUSART("  Warn msg: "); printToUSART(drone.warnMSG); printToUSART("\n");
            printToUSART("  Error msg: "); printToUSART(drone.errorMSG); printToUSART("\n");
            printToUSART("  Failsafe msg: "); printToUSART(drone.failsafeMSG); printToUSART("\n\n");

            printToUSART(" [TXT-GPS] Location: \n");
            printToUSART("  Latitude: "); printToUSART(drone.latitude); printToUSART("\n");
            printToUSART("  Longitude: "); printToUSART(drone.longitude); printToUSART("\n");
            printToUSART("  Altitude: "); printToUSART(drone.altitude); printToUSART("\n\n");

            printToUSART(" [TXT-GPS] Directionality: \n");
            printToUSART("  Ground Speed (kph): "); printToUSART(drone.groundSpeed); printToUSART("\n");
            printToUSART("  Heading: "); printToUSART(drone.heading); printToUSART("\n");
            printToUSART("  Satellites: "); printToUSART(drone.satellites); printToUSART("\n\n\n");
    #endif

            //internally now used blocking dma
            sendCRSFTelemetry_DMA(formattedBatteryTX, formattedBatteryTXSize);
            sendCRSFTelemetry_DMA(formattedFlightModeTX, formattedFlightModeTXSize);
            sendCRSFTelemetry_DMA(formattedGPSTX, formattedGPSSize);

            //old usart blocking transfer
            // for (int i = 0; i < formattedBatteryTXSize; i++) {
            //     while (!(currentBoardConfig.crsf_uart->ISR & USART_ISR_TXE)) {}
            //     currentBoardConfig.crsf_uart->TDR = formattedBatteryTX[i];
            // }
            //
            // for (int i = 0; i < formattedFlightModeTXSize; i++) {
            //     while (!(currentBoardConfig.crsf_uart->ISR & USART_ISR_TXE)) {}
            //     currentBoardConfig.crsf_uart->TDR = formattedFlightModeTX[i];
            // }
            //
            // for (int i = 0; i < formattedGPSSize; i++) {
            //     while (!(currentBoardConfig.crsf_uart->ISR & USART_ISR_TXE)) {}
            //     currentBoardConfig.crsf_uart->TDR = formattedGPSTX[i];
            // }

            //yield
            yieldCurrentTask();
        }
    }