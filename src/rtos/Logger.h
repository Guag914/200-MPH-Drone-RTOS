//
// Created by Akshay Gillett on 7/16/26.
//

#ifndef DRONE_RTOS_LOGGER_H
#define DRONE_RTOS_LOGGER_H

#ifdef __cplusplus
#include <cstdint>
#include <string>

extern void printToUSART(char c);
extern void printToUSART(const char* str);
extern void printToUSART(uint8_t val);
extern void printToUSART(uint16_t val);
extern void printToUSART(uint32_t val);
extern void printToUSART(int32_t val);
extern void printToUSART(float val);
extern void printToUSART(double val);
extern void printToUSART(bool b);
extern void printToUSART(int c);
extern void printToUSART(const std::string& c);

extern void printToCLI(char c);
extern void printToCLI(const char* str);
extern void printToCLI(int value);
extern void printToCLI(uint8_t value);
extern void printToCLI(uint16_t val);
extern void printToCLI(uint32_t val);
extern void printToCLI(int32_t value);
extern void printToCLI(float value);
extern void printToCLI(double value);
extern void printToCLI(bool value);

#endif
#endif