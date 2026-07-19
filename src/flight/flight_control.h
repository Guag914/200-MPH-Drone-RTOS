#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdint.h>
#include <string>

#include "stm32f7xx.h"

#ifdef __cplusplus //this section is completely HIDDEN from the C compiler (main.c)

enum class FlightState : uint8_t {
    DISARMED, //calculations running, DShot motor output forced to 0/Idle-disabled
    ARMED, //full control active, PID outputs mixed into DShot commands
    WARN, //active warning blocked from arming, displaying OSD text
    FAILSAFE, //critical lock, motors killed instantly, requires manual reset
    ERROR //error thrown by code, logs + instantly kills motors
};

struct WARN { //add warnings and their signal to the pilot via the OSD
    static constexpr std::string_view LOW_BATTERY = "LOW BATTERY DETECTED: LAND NOW\n";
    static constexpr std::string_view ARMED_ANGLE = "PLACE DRONE ON A FLAT SURFACE TO ARM\n";
    static constexpr std::string_view ARMED_THROTTLE = "PUSH THROTTLE TO 0 TO ARM\n";
};

struct FAILSAFE { //add failsafe messages
    static constexpr std::string_view CRSF_LOST = "CRSF RADIO LINK LOST: FORCE DISARM\n";
};

struct ERROR {
    static constexpr std::string_view STACK_OVERFLOW = "STACK OVERFLOW DETECTED: FORCE DISARMED\n";
};

struct DroneState {
    //rc inputs (Mapped from CRSF)
    float rollStick; //-1.0 to 1.0
    float pitchStick; //-1.0 to 1.0
    float yawStick; //-1.0 to 1.0
    float throttleStick; //0.0 to 1.0
    bool  armed;

    //sensor inputs (Raw & Calibrated)
    float gyroRaw[3]; //xyz
    float gyroCalibrated[3]; //after bias subtraction
    float gyroFiltered[3]; //after pt1 filter

    float motorOutput[4];

    float accelRaw[3]; //xyz

    bool calibrated;

    //orientation estimates
    float estimatedRoll;
    float estimatedPitch;
    float estimatedYaw;

    float attitudeRoll;
    float attitudePitch;
    float attitudeYaw;

    //protection safes
    std::string warnMSG; //used for warning the pilot about drone stats - needs attention - uses same log and osd message
    std::string failsafeMSG; //used to force disarm the drone completely - drone state critical - prints simplified message on OSD + logs drone stats
    std::string errorMSG; //also used to force disarm the drone - code threw exeption/error occured during runtime - prints "error + id" + logs all details

    //system health
    float batteryVoltage;
    uint32_t lastValidRCFrameTime;
    FlightState currentSystemState;
};

struct HardwarePinMap {
    USART_TypeDef* crsf_uart;
    DMA_Stream_TypeDef* crsf_dma_stream;
    uint32_t crsf_dma_channel;

    DMA_Stream_TypeDef* imu_dma_stream;
    SPI_TypeDef* imu_spi;
    uint32_t imu_spi_dma_channel;
    GPIO_TypeDef* imu_cs_port;
    uint16_t imu_cs_pin;

    ADC_TypeDef* battery_adc;
    uint32_t battery_channel;

    TIM_TypeDef* motor_timer;

    DMA_Stream_TypeDef* motor1_dma_stream;
    DMA_Stream_TypeDef* motor2_dma_stream;
    DMA_Stream_TypeDef* motor3_dma_stream;
    DMA_Stream_TypeDef* motor4_dma_stream;

    SPI_TypeDef* osd_spi;
    GPIO_TypeDef* osd_cs_port;
    uint16_t osd_cs_pin;
};


inline HardwarePinMap currentBoardConfig = { //ensure to configure this properly in stmcubemx software when getting real pinmaps
    .crsf_uart       = USART2,
    .crsf_dma_stream = DMA1_Stream5,
    .crsf_dma_channel = 4,

    .imu_dma_stream = DMA2_Stream0,
    .imu_spi         = SPI1,
    .imu_spi_dma_channel  = 3,
    .imu_cs_port     = GPIOA,
    .imu_cs_pin      = GPIO_PIN_4,

    .battery_adc     = ADC1,
    .battery_channel = ADC_CHANNEL_10,

    .motor_timer = TIM1, //map physical timer

    //map the corresponding DMA streams
    .motor1_dma_stream = DMA2_Stream1,
    .motor2_dma_stream = DMA2_Stream2,
    .motor3_dma_stream = DMA2_Stream6,
    .motor4_dma_stream = DMA2_Stream4,

    .osd_spi     = SPI2,
    .osd_cs_port = GPIOB,
    .osd_cs_pin  = GPIO_PIN_12
};

//make all tasks public to main.cpp
extern void sensorCalibration(); //init only

//these act as helpers for imuControlLoop
extern void readIMUDataRegisters();
extern void stateEstimation();
extern void flightLoop();

//interrupts + scheduled
[[noreturn]] extern void crsfParsing();
[[noreturn]] extern void radioLinkFailSafe();
[[noreturn]] extern void lowLevelFailSafe();
[[noreturn]] extern void dShotGeneration();
[[noreturn]] extern void flightStateMachine();
[[noreturn]] void imuControlLoop();

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