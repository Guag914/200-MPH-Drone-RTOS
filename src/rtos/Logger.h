//
// Created by Akshay Gillett on 7/16/26.
//

#ifndef DRONE_RTOS_LOGGER_H
#define DRONE_RTOS_LOGGER_H

#ifdef __cplusplus
#include <cstdint>

extern void printToUSART(char c);
extern void printToUSART(const char* str);
extern void printToUSART(uint8_t val);
extern void printToUSART(uint16_t val);
extern void printToUSART(uint32_t val);
extern void printToUSART(int32_t val);
extern void printToUSART(float val);
extern void printToUSART(double val);
extern void printToUSART(bool b);

#endif
#endif