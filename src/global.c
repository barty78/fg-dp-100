///////////////////////////////////////////////////////////////////////////////
//
// GLOBAL.C
//
//  DESCRIPTION: Global Definitions, Variables and Routines
//
//  CREATED:     18 OCT 2016
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include "comms.h"
#include "global.h"

// Battery-Backed RAM variables
//__attribute__((section(".battram"))) uint16_t ERROR_STATE;

///////////////////////////////////////////////////////////////////////////////
//
// vApplicationStackOverflowHook
//
//  DESCRIPTION: Thread Stack Overflow Callback Function.
//
//  NOTES:       1. Requires that the "configCHECK_FOR_STACK_OVERFLOW" macro in the "FreeRTOSConfig.h" is set to 1.
//               2. Every time a thread runs out, FreeRTOS check for the value of the current stack pointer.
//                  If it is higher than the top of the thread stack, then it is likely that a stack overflow is happened.
//                  In this case, this callback function is executed.
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

#if CHECK_STACK == 1
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName)
{
 //firmwareReset(STACK_OVERFLOW_ERROR);
asm("BKPT #0"); // If a stack overflow is detected then the debugger stops the firmware execution here
}
#endif

///////////////////////////////////////////////////////////////////////////////
//
// delay
//
//  DESCRIPTION: Uses "HAL_Tick()" to implement a blocking delay function.
//
//  NOTES:       1. Delay period argument is specified in milliseconds
//               2. This function won't exit until the delay period has expired
//               3. Handles wrapping of the 32-bit "endDelay" value
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void delay(uint32_t period)
{
 uint32_t startDelay = HAL_GetTick();
 uint32_t endDelay = startDelay + period;

 while (1)
 {
  uint32_t HALTick = HAL_GetTick();

  if (startDelay < endDelay || HALTick < startDelay)  // Ignore tick values before the "wrap"
  {
   if (HALTick >= endDelay) return;
  }
 }
}


///////////////////////////////////////////////////////////////////////////////
//
// delayedResponse
//
//  DESCRIPTION:  Generic template function for monitoring a condition and then waiting an intial delay
 //               before optionally first sending a message and then after a second specified delay, returning a "result" == 1
//
//  NOTES:        1. "flag" and "start" and either "send" or "end", (with corresponding flag) must be defined
//                2. Messaging is disabled if "delayMsg" == 0 or "flagMsg" == NULL or "message" == NULL
//                3. Action result is disabled if "delay" == 0
//                4. Function Arguments:
//                     uint8_t condition  This must == "1" in order for "*flag" to be set and to set the "*start", "*send" and "*end" timer counts, otherwise the "*flag" and "*flagMsg" values are reset.
//                     uint8_t* flag      This is a flag used to record the first instance of "condition" being true
//                     uint8_t* flagMsg   This is set on the first instance of "condition" being true and if "delayMsg" > 0
//                     uint32_t* start    This stores the tick count value recorded at the first instance of "condition" being true
//                     uint32_t* end      This stores a final tick count value calculated at the first instance of "condition" being true
//                     uint32_t* send     This stores a tick count value (for when a message should be sent) calculated at the first instance of "condition" being true
//                     uint32_t delay     This is the delay in milliseconds before the function should return "1" as long as the "condition" is true throughout
//                     uint32_t delayMsg  This is the delay in milliseconds before the function should send a message as long as the "condition" is true throughout
//                     char* message      This is a message string to output
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint8_t delayedResponse(uint8_t condition, uint8_t* flag, uint8_t* flagMsg, uint32_t* start, uint32_t* end, uint32_t* send, uint32_t delay, uint32_t delayMsg, char* message)
{
 uint8_t result = 0;
 uint32_t HALTick;   // This is a 1 msec 32 bit up counter

 if (flag == NULL) return 0;

 if (condition)
 {
  if (!(*flag))
  {
   *flag = 1;
   if (flagMsg != NULL) *flagMsg = 0;
   *start = HAL_GetTick();
   if (delayMsg > 0) *send = *start + delayMsg;
   if (delay > 0) *end = *start + delay;
  }

  HALTick = HAL_GetTick();
  if ((delayMsg != 0) && (flagMsg != NULL) && (message != NULL) && !(*flagMsg) && (*start < *send || HALTick < *start))  // Ignore tick values before the "wrap"
  {
   if (HALTick >= *send)
   {
    *flagMsg = 1;
    sendResponse(message);
   }
  }

  if ((delay != 0) && (*start < *end || HALTick < *start))  // Ignore tick values before the "wrap"
  {
   if (HALTick >= *end)
   {
    if (flag != NULL) *flag = 0;
    if (flagMsg != NULL) *flagMsg = 0;
    result = 1;
   }
  }
 }
 else
 {
  if (flagMsg != NULL) *flagMsg = 0;
  if (flag != NULL) *flag = 0;
 }

 return result;
}

void enterStop(UART_HandleTypeDef *handle)
{
	UART_WakeUpTypeDef wkupTypeDef;
	wkupTypeDef.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;

	HAL_UARTEx_StopModeWakeUpSourceConfig(handle, wkupTypeDef);
	HAL_UARTEx_EnableStopMode(handle);

	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

///////////////////////////////////////////////////////////////////////////////
//
// initSystem
//
//  DESCRIPTION: Final system initialisation tasks run just prior to starting the threads
//
//  NOTES:       1. Checks for previous error states and reports these to the controller before clearing them from the battery-backed SRAM
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void initSystem()
{
 char response[RESPONSE_BUFFER_LENGTH];

 __PWR_CLK_ENABLE();          // Enable the PWR clock
 HAL_PWR_EnableBkUpAccess();  // Enable access to the backup domain
 //__BKPSRAM_CLK_ENABLE();      // Enable backup SRAM Clock
 //HAL_PWREx_EnableBkUpReg();   // Enable the Backup SRAM low power Regulator to retain it's content in VBAT mode, (This will wait until the Backup SRAM low power Regulator is ready)

 if (DEBUG) writeMessage("DEBUG START\r\n");

 flagFirmwareReset = 0;

 switch (ERROR_STATE)
 {
  case SYSTEM_ERROR:
   sprintf(response, "<,FF,%04X,\"System Error\",", (unsigned)SYSTEM_ERROR);
   writeMessage(response);
  break;

  case THREAD_ERROR:
   sprintf(response, "<,FF,%04X,\"Unknown Thread Error\",", (unsigned)THREAD_ERROR);
   writeMessage(response);
  break;

  case STACK_OVERFLOW_ERROR:
   sprintf(response, "<,FF,%04X,\"Stack Overflow Error\",", (unsigned)STACK_OVERFLOW_ERROR);
   writeMessage(response);
  break;

  case USER_RESET_ERROR:
   sprintf(response, "<,FF,%04X,\"User Reset Error\",", (unsigned)USER_RESET_ERROR);
   writeMessage(response);
  break;

  case WRITEMESSAGE_TIMEOUT_ERROR:
   sprintf(response, "<,FF,%04X,\"Write Message Thread Timeout Error\",", (unsigned)WRITEMESSAGE_TIMEOUT_ERROR);
   writeMessage(response);
  break;

  case READPACKET_TIMEOUT_ERROR:
   sprintf(response, "<,FF,%04X,\"Read Packet Thread Timeout Error\",", (unsigned)READPACKET_TIMEOUT_ERROR);
   writeMessage(response);
  break;

  case PARSEPACKET_TIMEOUT_ERROR:
   sprintf(response, "<,FF,%04X,\"Parse Packet Thread Timeout Error\",", (unsigned)PARSEPACKET_TIMEOUT_ERROR);
   writeMessage(response);
  break;

  case READIO_TIMEOUT_ERROR:
   sprintf(response, "<,FF,%04X,\"Read IO Thread Timeout Error\",", (unsigned)READIO_TIMEOUT_ERROR);
   writeMessage(response);
  break;

  case WRITEIO_TIMEOUT_ERROR:
   sprintf(response, "<,FF,%04X,\"Write IO Thread Timeout Error\",", (unsigned)WRITEIO_TIMEOUT_ERROR);
   writeMessage(response);
  break;

  case UNDERVOLTAGE_ERROR:
   sprintf(response, "<,FF,%04X,\"Under-Voltage Error\",", (unsigned)UNDERVOLTAGE_ERROR);
   writeMessage(response);
  break;

  case MONITOR_TIMEOUT_ERROR:
   sprintf(response, "<,FF,%04X,\"Monitor Thread Timeout Error\",", (unsigned)MONITOR_TIMEOUT_ERROR);
   writeMessage(response);
  break;

  case I2CREAD_TIMEOUT_ERROR:
     sprintf(response, "<,FF,%04X,\"I2C Read Timeout Error\",", (unsigned)I2CREAD_TIMEOUT_ERROR);
     writeMessage(response);
    break;

  case I2CWRITE_TIMEOUT_ERROR:
     sprintf(response, "<,FF,%04X,\"I2C Write Timeout Error\",", (unsigned)I2CWRITE_TIMEOUT_ERROR);
     writeMessage(response);
    break;
 }

 ERROR_STATE = 0;
}

///////////////////////////////////////////////////////////////////////////////
//
// firmwareReset
//
//  DESCRIPTION: Force an MCU reset from the firmware
//
//  NOTES:       1. Sets the global "flagFirmwareReset" flag to inform all threads that this is a controlled reset
//               2. Writes the error state to the battery-backed SRAM, (so that it can be reported during the next boot)
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void firmwareReset(uint16_t error)
{
 HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
 flagFirmwareReset = 1;
 ERROR_STATE = error;
 HAL_NVIC_SystemReset();
}
