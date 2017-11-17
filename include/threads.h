#ifndef THREADS_H
#define THREADS_H

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include "global.h"

#if CHECK_THREADS == 1
 uint32_t blinkStartTick, blinkEndTick;
 uint32_t writeMessageStartTick, readPacketStartTick, parsePacketStartTick, readIOStartTick, /*writeIOStartTick,*/ monitorStartTick, heartbeatStartTick;
 uint32_t writeMessageEndTick, readPacketEndTick, parsePacketEndTick, readIOEndTick, /*writeIOEndTick,*/ monitorEndTick, heartbeatEndTick;
 uint32_t retryWaitTick;
#endif

TickType_t *xLastWakeTime;

osThreadId blinkTID;
osThreadId writeMessageTID, readPacketTID, parsePacketTID, readIOTID, /*writeIOTID,*/ monitorTID, heartbeatTID;

uint8_t initThreads();


void blinkThread(void const *argument);

void heartbeatThread(void const *argument);

void writeMessageThread(void const *argument);
void readPacketThread(void const *argument);
void parsePacketThread(void const *argument);

void readIOThread(void const *argument);
//void writeIOThread(void const *argument);

void monitorThread(void const *argument);
#endif

