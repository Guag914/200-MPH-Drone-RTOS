//
// Created by Akshay Gillett on 7/16/26.
//

#include "Helpers.h"

#include <algorithm>
#include <cmath>
#include <cstring>
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

//dshot helpers
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

//flight loop helpers
// applyActualRates(drone.rollStick, rollRateCfg);

float applyActualRates(float stick, const ActualRateConfig& config) {
    stick = std::clamp(stick, -1.0f, 1.0f);
    const float absStick = std::abs(stick);

    const float expoFactor = (absStick * absStick * absStick * config.expo) + (absStick * (1.0f - config.expo));
    const float rateDelta = std::max(0.0f, config.maxRate - config.centerRate); //calc delta
    const float outputRate = (stick * config.centerRate) + (std::copysign(expoFactor, stick) * rateDelta);

    return outputRate;
}

//gps helpers

uint8_t crc8_dvb_s2(const uint8_t* data, uint8_t len) { //used to add a checksum at the end
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint8_t formatCRSFFrame(uint8_t frameType, const uint8_t* payload, uint8_t payloadLen, uint8_t* outBuffer) {
    if (!outBuffer || (!payload && payloadLen > 0)) {
        return 0;
    }

    outBuffer[0] = 0xC8; //sync byte
    outBuffer[1] = payloadLen + 2; // frame length (add 2 for type byte and crc byte)
    outBuffer[2] = frameType; //type id byte

    //copy payload
    for (uint8_t i = 0; i < payloadLen; i++) {
        outBuffer[3 + i] = payload[i];
    }

    //compute crc8 over frame type & payload
    uint8_t crc = crc8_dvb_s2(&outBuffer[2], payloadLen + 1);
    outBuffer[3 + payloadLen] = crc;

    return payloadLen + 4;
}

//gps helpers
int32_t parseNmeaCoord(const char* str, char dir) {
    if (!str || strlen(str) < 4) return 0;

    double rawVal = atof(str);
    int degrees = static_cast<int>(rawVal / 100.0);
    double minutes = rawVal - (degrees * 100.0);
    double decimalDegrees = degrees + (minutes / 60.0);

    if (dir == 'S' || dir == 'W') {
        decimalDegrees = -decimalDegrees;
    }

    return static_cast<int32_t>(decimalDegrees * 1e7);
}

void writeOSDString(uint8_t row, uint8_t col, char* str) {
    //create payload buffer (64)
    //idx 0 = flag (write)
    //1 = row
    //2 = col
    //3 = 0 (normal 'attrib')

    //cast uint8 for each item
    //sendMSPFrame

    uint8_t payload[64];
    uint8_t len = std::strlen(str); //gets the length of the string
    uint8_t idx = 0;

    payload[idx++] = MSP_DP_WRITE;
    payload[idx++] = row;
    payload[idx++] = col;
    payload[idx++] = 0;

    for (uint8_t i = 0; i < len; i++) { payload[i + 4] = static_cast<uint8_t>(str[i]); } //compile the char into uint_8 char by char

    sendMSPFrame(182, payload, len + 4); //offset accounts for the headers
}

void sendMSPFrame(uint8_t cmd, const uint8_t* payload, uint8_t payloadLen) {
    //create 64 frame buffer
    //add $, M, <, payloadSize, and cmd starting at idx 0
    //create the checksum by doing ^= payload[i]
    //append checksum

    //broadcast

    uint8_t frame[64];
    uint8_t idx = 0;

    //header is always $M< - tells vt an incoming frame is starting followed by specifics
    frame[idx++] = '$';
    frame[idx++] = 'M';
    frame[idx++] = '<';
    frame[idx++] = payloadLen;
    frame[idx++] = cmd;

    uint8_t checksum = payloadLen ^ cmd;

    for (uint8_t i = 0; i < payloadLen; i++) {
        frame[idx++] = payload[i]; //appends payload to frameidx
        checksum ^= payload[i]; //calculates checksum
    }

    //broadcast
    for (uint8_t i = 0; i < idx; i++) {
        while (!(currentBoardConfig.osd_uart->ISR & USART_ISR_TXE)) {}
        currentBoardConfig.osd_uart->TDR = frame[i];
    }
}

//baro helpers
static uint8_t spi_transfer(SPI_TypeDef *SPIx, uint8_t data) {
    while (!(SPIx->SR & SPI_SR_TXE)) {} //wait until transmit buffer is empty (ONLY USE FOR INIT because normal tasks use DMA)
    SPIx->DR = data;
    while (!(SPIx->SR & SPI_FLAG_RXNE)) {} //wait
    return SPIx->DR;
}

static uint8_t baro_read_reg(SPI_TypeDef *SPIx, const uint8_t reg_addr) {
    HAL_GPIO_WritePin(currentBoardConfig.baro_cs_port, currentBoardConfig.baro_cs_pin, GPIO_PIN_RESET); //cs low
    spi_transfer(SPIx, reg_addr | 0x80); //send register address
    const uint8_t val = spi_transfer(SPIx, 0x00); //send dummy byte
    HAL_GPIO_WritePin(currentBoardConfig.baro_cs_port, currentBoardConfig.baro_cs_pin, GPIO_PIN_SET); //cs high

    return val;
}

static uint16_t baro_read_reg16(SPI_TypeDef *SPIx, const uint8_t lsb_reg, const uint8_t msb_reg) {
    const uint8_t lsb = baro_read_reg(SPIx, lsb_reg);
    const uint8_t msb = baro_read_reg(SPIx, msb_reg);

    return static_cast<uint16_t>((msb << 8) | lsb);
}

static void baro_write_reg(SPI_TypeDef *SPIx, uint8_t reg_addr, uint8_t data) {
    HAL_GPIO_WritePin(currentBoardConfig.baro_cs_port, currentBoardConfig.baro_cs_pin, GPIO_PIN_RESET);
    spi_transfer(SPIx, reg_addr & ~0x80); //clear bit 7 for WRITE mode
    spi_transfer(SPIx, data);
    HAL_GPIO_WritePin(currentBoardConfig.baro_cs_port, currentBoardConfig.baro_cs_pin, GPIO_PIN_SET);
}

BaroTrim initBarometer() {
    uint8_t chipID = baro_read_reg(currentBoardConfig.baro_spi, 0x00);
    if (chipID != 0x60) {  //verfy hardware communication
        drone.currentSystemState = FlightState::WARN;
        setEventMessage(drone.warnMSG, WARN::BARO_NO_RESPONSE);
    }

    //set up sock config vars in the baro
    BaroTrim trim;
    SPI_TypeDef *spi = currentBoardConfig.baro_spi;

    //use external function with auto combined
    trim.T1 = baro_read_reg16(spi, 0x31, 0x32);
    trim.T2 = baro_read_reg16(spi, 0x33, 0x34);
    trim.T3 = baro_read_reg(spi, 0x35);

    trim.P1 = baro_read_reg16(spi, 0x36, 0x37);
    trim.P2 = baro_read_reg16(spi, 0x38, 0x39);
    trim.P3 = baro_read_reg(spi, 0x3A);
    trim.P4 = baro_read_reg(spi, 0x3B);
    trim.P5 = baro_read_reg16(spi, 0x3C, 0x3D);
    trim.P6 = baro_read_reg16(spi, 0x3E, 0x3F);
    trim.P7 = baro_read_reg(spi, 0x40);
    trim.P8 = baro_read_reg(spi, 0x41);
    trim.P9 = baro_read_reg16(spi, 0x42, 0x43);
    trim.P10 = baro_read_reg(spi, 0x44);
    trim.P11 = baro_read_reg(spi, 0x45);

    baro_write_reg(spi, 0x1B, 0x33); //enable normal modes
    baro_write_reg(spi, 0x1C, 0x02); //write OSR reg

    return trim;
}

//task health task helper
uint32_t getUnusedStackWords(const uint8_t taskIdx) {

    const uint32_t* stack = taskControlBlocks[taskIdx].taskStack;
    uint32_t unusedWords = 0;

    for (int i = 8; i < taskControlBlocks[taskIdx].stackSizeWords; i++) {
        if (stack[i] == 0xA5A5A5A5) {
            unusedWords++;
        } else {
            break; //first overriten word found
        }
    }

    return unusedWords;
}