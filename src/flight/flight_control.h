#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdint.h>
#include "stm32f7xx.h"

#ifdef __cplusplus //this section is completely HIDDEN from the C compiler (main.c)

enum class FlightState : uint8_t {
    DISARMED, //calculations running, DShot motor output forced to 0/Idle-disabled
    ARMED, //full control active, PID outputs mixed into DShot commands
    WARN, //active warning blocked from arming, displaying OSD text
    FAILSAFE //critical lock, motors killed instantly, requires manual reset
};

struct DroneState {
    //rc inputs (Mapped from CRSF)
    float rollStick; //-1.0 to 1.0
    float pitchStick; //-1.0 to 1.0
    float yawStick; //-1.0 to 1.0
    float throttleStick; //0.0 to 1.0
    bool  armSwitch;

    //sensor inputs (Raw & Calibrated)
    float gyroRaw[3]; //xyz
    float gyroCalibrated[3]; //after bias subtraction
    float accelRaw[3]; //xyz

    bool calibrated;

    //orientation estimates
    float estimatedRoll;
    float estimatedPitch;
    float estimatedYaw;

    //system health
    float batteryVoltage;
    uint32_t lastValidRCFrameTime;
    FlightState currentSystemState;
};

struct HardwarePinMap {
    USART_TypeDef* crsf_uart;
    DMA_Stream_TypeDef* crsf_dma_stream;

    SPI_TypeDef* imu_spi;
    GPIO_TypeDef* imu_cs_port;
    uint16_t imu_cs_pin;

    ADC_TypeDef* battery_adc;
    uint32_t battery_channel;
};

inline HardwarePinMap currentBoardConfig = {
    .crsf_uart       = USART3,
    .crsf_dma_stream = DMA1_Stream1,

    //update these when geting hardware config map
    .imu_spi         = SPI1,
    .imu_cs_port     = GPIOA,
    .imu_cs_pin      = GPIO_PIN_4,

    .battery_adc     = ADC1,
    .battery_channel = 0 //represents the specific analog channel pin
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
//this section is visible to BOTH core/src/main.c (C) and src/Main.cpp (C++)

extern DroneState drone;

#ifdef __cplusplus
}
#endif

#endif // FLIGHT_CONTROL_H