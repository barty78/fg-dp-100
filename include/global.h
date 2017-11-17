#ifndef GLOBAL_H
#define GLOBAL_H

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

// Version Definitions
#define HARDWARE_ID "DP-100"
#define FIRMWARE_VERSION "001"
#define SVN_RELEASE "571"  // $Revision:$

#define RS485

#define ID_STRING_LENGTH	24
static char	id_string[ID_STRING_LENGTH+1];

// Debug Definitions
#define CHECK_STACK 1    // Global Thread Stack debugging flag, (Set to 1 to enable stack monitoring and overflow exceptions)
#define CHECK_THREADS 1  // Monitor Thread loop durations


// Battery-Backed SRAM variables
#define ERROR_STATE (*(__IO uint32_t *) (RTC_BKP0R))
//#define FLOW_COUNT (*(__IO uint32_t *) (BKPSRAM_BASE + 2))
//#define FLOW_COUNT_CRC (*(__IO uint32_t *) (BKPSRAM_BASE + 6))

// Error Definitions
#define SYSTEM_ERROR               0x0001
#define THREAD_ERROR               0x0002
#define STACK_OVERFLOW_ERROR       0x0003
#define USER_RESET_ERROR           0x0004
#define WRITEMESSAGE_TIMEOUT_ERROR 0x0005
#define READPACKET_TIMEOUT_ERROR   0x0006
#define PARSEPACKET_TIMEOUT_ERROR  0x0007
#define READIO_TIMEOUT_ERROR       0x0008
#define WRITEIO_TIMEOUT_ERROR      0x0009
#define UNDERVOLTAGE_ERROR         0x000A
#define SURGEPROTECTION_ERROR      0x000B
#define PWR_5_12_24_VOLTAGE_ERROR  0x000C
#define MONITOR_TIMEOUT_ERROR      0x000D
#define I2CREAD_TIMEOUT_ERROR      0x000E
#define I2CWRITE_TIMEOUT_ERROR     0x000F
#define DISPLAY_TIMEOUT_ERROR      0x0010
#define BLINK_TIMEOUT_ERROR        0x0011

#define THREAD_WATCHDOG_DELAY (60000)  // 60 seconds Shutdown Delay, (system will be shutdown when any thread watchdog flag is not reset within this period)

#define STM32_UUID ((uint32_t *)0x1FFF7A10)  // STM32F205 Unique Identifier Address

uint8_t flagFirmwareReset;

// Global debugging flag. Only set when testing code. Do NOT set in production code!
#ifdef STMDEBUG
#define DEBUG 1
#define NUCLEO 1          // Global Nucleo Development Board flag. Indicates that code is being executed on Nucleo Dev Board. Only set when testing code. Do NOT set in production code!
#else
#define DEBUG 0
#define NUCLEO 0         // Global Nucleo Development Board flag. Indicates that code is being executed on Nucleo Dev Board. Only set when testing code. Do NOT set in production code!
#endif

#if CHECK_STACK == 1
// #define configCHECK_FOR_STACK_OVERFLOW 1
// #define INCLUDE_uxTaskGetStackHighWaterMark 1

UBaseType_t blinkThreadStackHighWaterMark;

UBaseType_t writeMessageThreadStackHighWaterMark, readPacketThreadStackHighWaterMark, parsePacketThreadStackHighWaterMark,
            readIOThreadStackHighWaterMark, writeIOThreadStackHighWaterMark, monitorThreadStackHighWaterMark, blinkThreadStackHighWaterMark,
            heartbeatThreadStackHighWaterMark;

 void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName);
#endif

void delay(uint32_t period);
uint8_t delayedResponse(uint8_t condition, uint8_t* flag, uint8_t* flagMsg, uint32_t* start, uint32_t* end, uint32_t* send, uint32_t delay, uint32_t delayMsg, char* message);
void initSystem();
void enterStop(UART_HandleTypeDef* handle);
void firmwareReset(uint16_t error);
/*
 Exported macro ------------------------------------------------------------
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
 Exported functions ------------------------------------------------------- */

#endif
