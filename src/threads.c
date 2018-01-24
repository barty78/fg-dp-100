///////////////////////////////////////////////////////////////////////////////
//
// THREADS.C
//
//  DESCRIPTION: Thread Routines
//
//  CREATED:     13 OCT 2016
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

#include <math.h>

#include "global.h"
#include "io.h"
#include "pca9956b.h"
#include "comms.h"
#include "parse.h"
#include "threads.h"
#include "packets.h"

//extern uint16_t ERROR_STATE;
extern int8_t displayID;

osMessageQDef(buttons, 10, uint8_t);
osMessageQDef(display, 1, leds);


///////////////////////////////////////////////////////////////////////////////
//
// initThreads
//
//  DESCRIPTION: Define and initialise all thread descriptors.
//
//  NOTES:       1. Uses CMSIS-RTOS API macro, "osThreadDef"

//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint8_t initThreads()
{
 #if CHECK_THREADS == 1  // Note: Setting the Start and End Tick values to the same value avoids an inadvertant watchdog trigger upon startup, (since this condition is checked in the monitor thread)
//  writeMessageStartTick = 0;
//  writeMessageEndTick = 0;
//  readPacketStartTick = 0;
//  readPacketEndTick = 0;
  parsePacketStartTick = 0;
  parsePacketEndTick = 0;
  readIOStartTick = 0;
  readIOEndTick = 0;
  //writeIOStartTick = 0;
  //writeIOEndTick = 0;
  monitorStartTick = 0;
  monitorEndTick = 0;
  blinkStartTick = 0;
  blinkEndTick = 0;
  heartbeatStartTick = 0;
  heartbeatEndTick = 0;
  retryWaitTick = 0;

  displayID = -1;
 #endif

 buttonQID = osMessageCreate(osMessageQ(buttons), NULL);
 vQueueAddToRegistry(buttonQID, "buttons");

 displayQID = osMessageCreate(osMessageQ(display), NULL);
 vQueueAddToRegistry(displayQID, "display");

 osThreadDef(heartbeat, heartbeatThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*4);
 heartbeatTID = osThreadCreate(osThread(heartbeat), NULL);

// osThreadDef(blink, blinkThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
// blinkTID = osThreadCreate(osThread(blink), NULL);

// osThreadDef(writeMessage, writeMessageThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
// writeMessageTID = osThreadCreate(osThread(writeMessage), NULL);

// osThreadDef(readPacket, readPacketThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
// readPacketTID = osThreadCreate(osThread(readPacket), NULL);

 osThreadDef(parsePacket, parsePacketThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*12); // + RX_BUFFER_LENGTH/4 + 2);
 parsePacketTID = osThreadCreate(osThread(parsePacket), NULL);

 osThreadDef(readIO, readIOThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*4);
 readIOTID = osThreadCreate(osThread(readIO), NULL);

 /*osThreadDef(writeIO, writeIOThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*2);
 writeIOTID = osThreadCreate(osThread(writeIO), NULL);*/

 osThreadDef(monitor, monitorThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*4);
 monitorTID = osThreadCreate(osThread(monitor), NULL);

 return 0;
}

//-----------------------------------------------------------------------------

void heartbeatThread(void const *argument)
{

char alive[RESPONSE_BUFFER_LENGTH];
  sprintf(alive, "%c,84,%d,%08X-%08X-%08X,", SOF_TX, panelType, (unsigned)(HAL_GetUIDw2()), (unsigned)(HAL_GetUIDw1()), (unsigned)HAL_GetUIDw0());
  const TickType_t xDelay = 10000 / portTICK_PERIOD_MS;

  for( ;; )
  {
#if CHECK_THREADS == 1
      heartbeatStartTick = HAL_GetTick();
      heartbeatEndTick = heartbeatStartTick + THREAD_WATCHDOG_DELAY;
#endif

      if (displayID == -1)
        {
          taskENTER_CRITICAL();
          sendResponse(alive);
          taskEXIT_CRITICAL();
        }

      vTaskDelay(xDelay);
  }

  osThreadTerminate(NULL);
}



void blinkThread(void const *argument)
{
	char alive[RESPONSE_BUFFER_LENGTH];
	const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
	const TickType_t xFreq = 30;
	uint8_t counter = 99;
	char digits[3];
	uint8_t i = 0;

	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{

#if CHECK_THREADS == 1
	    blinkStartTick = HAL_GetTick();
	    blinkEndTick = blinkStartTick + THREAD_WATCHDOG_DELAY;
#endif

//		taskENTER_CRITICAL();
//		refresh();
//		itoa10(counter, digits, 3);
//		if (counter-- <= 0)counter = 99;
//		display(digits);
//		refresh();

//		HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
//		sprintf(alive, "<,84,%08X-%08X-%08X,", (unsigned)(HAL_GetUIDw2()), (unsigned)(HAL_GetUIDw1()), (unsigned)HAL_GetUIDw0());
//		sendResponse(alive);
//		taskEXIT_CRITICAL();
		//writeMessage("HELLO WORLD\n");
/*
		if (i>0)pwm(i-1, 0.0);
		current(i, 0.75);
		//osDelay(10);
		pwm(i, 1.0);
		//osDelay(10);
//		reg(MODE2);
//		pca9956_status();

		if (++i >= n_of_ports) i = 0;
*/
		vTaskDelay(xDelay);
	}

	osThreadTerminate(NULL);
}

///////////////////////////////////////////////////////////////////////////////
//
// writeMessageThread
//
//  DESCRIPTION: Call USART Tx Interrupt-Managed transmitter function, "HAL_UART_Transmit_IT".
//
//  NOTES:       1. Checks "byteTransmitted" global flag, (set in "HAL_UART_TxCpltCallback")
//               2. USART is set to transmit only a single byte, (referenced at txBuffer[txMessageTail])
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void writeMessageThread(void const *argument)
{
 while(1)
 {
  #if CHECK_THREADS == 1
   writeMessageStartTick = HAL_GetTick();
   writeMessageEndTick = writeMessageStartTick + THREAD_WATCHDOG_DELAY;
  #endif

   taskENTER_CRITICAL();

   if (flagByteTransmitted == 1)
     {
       if (txMessageHead != txMessageTail)  // Data remaining in Transmit Buffer
         {
           flagByteTransmitted = 0;
#ifdef RS485
           //		   HAL_GPIO_TogglePin(RS485_EN_GPIO_Port, RS485_EN_Pin);							// Enable Transciever
           HAL_GPIO_TogglePin(RS485_RXE_GPIO_Port, RS485_RXE_Pin);                // Disable RX

//           HAL_UART_Transmit_DMA(handleLPUART1, (uint8_t*)(&(txBuffer[txMessageTail])), txMessageHead - txMessageTail);
//           HAL_UART_Transmit_DMA(handleLPUART1, (uint8_t*)txBuffer, 30);

#ifndef TEST
           if (txMessageHead <= 0)
             {
               //               HAL_UART_Transmit_IT(&handleLPUART1, (uint8_t*)(&(txBuffer[txMessageTail])), (TX_BUFFER_LENGTH - txMessageTail) + txMessageHead);  // Send Message
//               HAL_UART_Transmit_DMA(handleLPUART1, (uint8_t*)(&(txBuffer[txMessageTail])), (TX_BUFFER_LENGTH - txMessageTail) + txMessageHead);  // Send Message

             } else {
                 //                 HAL_UART_Transmit_IT(&handleLPUART1, (uint8_t*)(&(txBuffer[txMessageTail])), txMessageHead - txMessageTail);  // Send Message
//                 HAL_UART_Transmit_DMA(handleLPUART1, (uint8_t*)(&(txBuffer[txMessageTail])), txMessageHead - txMessageTail);  // Send Message
             }
#else
           HAL_UART_Transmit_IT(handleLPUART1, (uint8_t*)(&(txBuffer[txMessageTail])), 1);  // Send Message

#endif

#else
           HAL_UART_Transmit_IT(handleUART1, (uint8_t*)(&(txBuffer[txMessageTail])), 1);
#endif
         }
     }

   taskEXIT_CRITICAL();

   osThreadYield();
 }
 osThreadTerminate(NULL);

 if (!flagFirmwareReset) firmwareReset(THREAD_ERROR);
}

///////////////////////////////////////////////////////////////////////////////
//
// readPacketThread
//
//  DESCRIPTION: Assembles the incoming TIB data packet.
//
//  NOTES:       1. Transfers any new bytes in the rxBuffer to the packetBuffer, (which is a circular buffer of buffers)
//               2. Increments "rxMessageTail" pointer, (wrapping if necessary)
//               3. Checks for a <LF> packet end character before setting the "flagPacketReceived" flag
//               4. After a full packet is received it increments the "packetHead" pointer, (wrapping if necessary)
//               5. If it receives a ">" packet start character before a <LF> packet end character it ignores the data and starts again.
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////


void readPacketThread(void const *argument)
{
 while(1)
 {
  #if CHECK_THREADS == 1
   readPacketStartTick = HAL_GetTick();
   readPacketEndTick = readPacketStartTick + THREAD_WATCHDOG_DELAY;
  #endif

  taskENTER_CRITICAL();
   if (rxMessageHead != rxMessageTail)
   {
    if (rxBuffer[rxMessageTail] == SOF_RX)
      {
        HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
        packetPointer = 0;
      }
    packetBuffer[packetHead][packetPointer++] = rxBuffer[rxMessageTail];
    if (rxBuffer[rxMessageTail] == '\n')
    {
     packetPointer = 0;
     HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
     flagPacketReceived = 1;

     if (++packetHead >= PACKET_BUFFER_LENGTH) packetHead = 0;
    }
    else if (rxBuffer[rxMessageTail] == 0x18) firmwareReset(USER_RESET_ERROR);

    if (++rxMessageTail >= RX_BUFFER_LENGTH) rxMessageTail = 0;
   }
  taskEXIT_CRITICAL();

  osThreadYield();
 }
 osThreadTerminate(NULL);

 if (!flagFirmwareReset) firmwareReset(THREAD_ERROR);
}

///////////////////////////////////////////////////////////////////////////////
//
// parsePacketThread
//
//  DESCRIPTION: Parses the TIB data packets stored in the packetBuffer, (which is a circular buffer of buffers)
//
//  NOTES:       1. Transfers the contents of the currently selected packetBuffer entry to a temporary buffer, "packet"
//               2. Increments "packetTail" pointer, (wrapping if necessary)
//               3. Parses the command contained within the data packet.
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void parsePacketThread(void const *argument)
{
 uint32_t i, received;
 char command[RX_BUFFER_LENGTH];

 while(1)
 {
  #if CHECK_THREADS == 1
   parsePacketStartTick = HAL_GetTick();
   parsePacketEndTick = parsePacketStartTick + THREAD_WATCHDOG_DELAY;
  #endif

  taskENTER_CRITICAL();
   received = flagPacketReceived;
  taskEXIT_CRITICAL();

  if (received == 1)
    {
//      HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
      if (dataBuf[0] == SOF_RX)
        {
          taskENTER_CRITICAL();
          for (i=0; i<RX_BUFFER_LENGTH && dataBuf[i] != '\n'; i++) command[i] = dataBuf[i];
//          if (++packetTail >= PACKET_BUFFER_LENGTH) packetTail = 0;

#ifdef DISABLE
          //TODO - We want to check if the packet is just an echo of a message we just sent.
          //If it is, then clear the flag and wait for next packet.  Don't add it to the packetBuffer for parsing
          //If it is not, then there was contention and we need to backoff and retry.
          if (flagByteTransmitted && flagPacketReceived)
            {
              if (memcmp(&command, &lastMsg, strlen(lastMsg)) != 0)
                {
                  retryWaitTick = HAL_GetTick() + rand();
                } else {
                    retryWaitTick = 0;
                }
              if (HAL_GetTick() > retryWaitTick) sendResponse(lastMsg);

            }
#endif
          flagPacketReceived = 0;
          taskEXIT_CRITICAL();

          if (i < RX_BUFFER_LENGTH)  // Check for Rx Buffer Overrun
            {
              command[i] = 0;  // Null terminate the command string
              parseCommand(command); // Only parse the command if it has a valid SOF char.
            }

        }


    }

  osThreadYield();
 }
 osThreadTerminate(NULL);

 if (!flagFirmwareReset) firmwareReset(THREAD_ERROR);
}

///////////////////////////////////////////////////////////////////////////////
//
// readIOThread
//
//  DESCRIPTION: Reads all the various I/O inputs and places the results in intermediate data structures accessed via the Command Parser
//
//  NOTES:       1.
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void readIOThread(void const *argument)
{
 uint8_t debounceFlag[4] = {0, 0, 0, 0};
 uint32_t debounceStart[4], debounceEnd[4];

 while(1)
 {
  #if CHECK_THREADS == 1
   readIOStartTick = HAL_GetTick();
   readIOEndTick = readIOStartTick + THREAD_WATCHDOG_DELAY;
  #endif

  readButtonInputs(debounceFlag, debounceStart, debounceEnd);
  osThreadYield();

//  readADC();
//  osThreadYield();

 }
 osThreadTerminate(NULL);

 if (!flagFirmwareReset) firmwareReset(THREAD_ERROR);
}

///////////////////////////////////////////////////////////////////////////////
//
// writeIOThread
//
//  DESCRIPTION:  Implements delayed I/O actions such as turning off Solenoids and Power Shutdown
//
//  NOTES:       1. If the solenoid "durationEnd" value calculation involves an unsigned integer "wrap", then ignore tick values before the "wrap"
//               2. If the Power Shutdown "pwrShdnDelayEnd" value calculation involves an unsigned integer "wrap", then ignore tick values before the "wrap"
//                   0 000
//                   1 001
//                   2 010
//                   3 011
//                   4 100 <= Start
//                   5 101  ]
//                   6 110  ] Tick > Start (and Tick > End)
//                   7 111  ]
//                   -----
//                   0 000  ]
//                   1 001  ] Tick < Start
//                   2 010  ]
//                   3 011 <= End = Start + Delay, (Delay = 7)
//                   4 100
//                   5 101
//                   6 110
//                   7 111
//                   -----
//                   Trigger:           Tick >= End
//                   No Wrap:           Start < End
//                   Ignore if Wrapped: Tick < Start
//                   Note: If Delay > 7, (ie. the maximum value) the values will "wrap" twice.
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void writeIOThread(void const *argument)
{
 while(1)
 {
  #if CHECK_THREADS == 1
//   writeIOStartTick = HAL_GetTick();
//   writeIOEndTick = writeIOStartTick + THREAD_WATCHDOG_DELAY;
  #endif

  osThreadYield();
 }
 osThreadTerminate(NULL);

 if (!flagFirmwareReset) firmwareReset(THREAD_ERROR);
}

///////////////////////////////////////////////////////////////////////////////
//
// monitorThread
//
//  DESCRIPTION:  Monitors the status of various push buttons and inputs:
//
//  NOTES:        1. Thread watchdogs
//                2. Delayed Shutdown
//                3. Power button has been held down for 10 seconds
//                4. Truck power is lost and the UPS battery is depleted
//                5. Power from the truck has been lost or returned
//                6. Whenever the value of the push buttons change
//                7. Also the TIB should send error messages, (at the moment only if its been reset)
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void monitorThread(void const *argument)
{
 char response[RESPONSE_BUFFER_LENGTH];

 uint8_t pushButtonsThread, prevButtons;
 size_t freeHeap = configTOTAL_HEAP_SIZE;

 taskENTER_CRITICAL();
  prevButtons = pushButtons;
  buttonPressedTick = 0;
 taskEXIT_CRITICAL();

 while(1)
 {
     HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
  #if CHECK_THREADS == 1
   monitorStartTick = HAL_GetTick();
   monitorEndTick = monitorStartTick + THREAD_WATCHDOG_DELAY;
  #endif

  #if CHECK_STACK == 1
   heartbeatThreadStackHighWaterMark = uxTaskGetStackHighWaterMark(heartbeatTID);
//   blinkThreadStackHighWaterMark = uxTaskGetStackHighWaterMark(blinkTID);
//   writeMessageThreadStackHighWaterMark = uxTaskGetStackHighWaterMark(writeMessageTID);
//   readPacketThreadStackHighWaterMark = uxTaskGetStackHighWaterMark(readPacketTID);
   parsePacketThreadStackHighWaterMark = uxTaskGetStackHighWaterMark(parsePacketTID);
   readIOThreadStackHighWaterMark = uxTaskGetStackHighWaterMark(readIOTID);
   //writeIOThreadStackHighWaterMark = uxTaskGetStackHighWaterMark(writeIOTID);
   monitorThreadStackHighWaterMark = uxTaskGetStackHighWaterMark(monitorTID);
   freeHeap = xPortGetFreeHeapSize();
#endif

  // Monitor Push Button State
  taskENTER_CRITICAL();
   pushButtonsThread = pushButtons;
   displaySuppV = displaySuppVSum/(float)VOLTAGE_FILTER_LENGTH;
  taskEXIT_CRITICAL();

#ifdef ADV_BUTTONS
  if (pushButtonsThread != prevButtons && buttonPressedTick == 0) // && pushButtonsThread != 0)
    {
      buttonPressedTick = HAL_GetTick();
      pushedButton = pushButtonsThread;
    } else {
        if (pushButtonsThread == prevButtons && buttonPressedTick > 0)
          {
            if (buttonPressedTick + BUTTON_HOLD_DELAY < HAL_GetTick())
              {
                sprintf(response, "<,65,%02X,1,", (unsigned)pushedButton);    // Send message with long press flag
              } else {
                  sprintf(response, "<,65,%02X,0,", (unsigned)pushedButton);    // Send message with short press flag
              }
            prevButtons = pushButtonsThread;
            sendResponse(response);
            buttonPressedTick = 0;
          }
    }

#else
  if (pushButtonsThread != prevButtons)
    {
      prevButtons = pushButtonsThread;
//      sprintf(response, "<,65,%02X,", (unsigned)pushButtonsThread);
      osMessagePut(buttonQID, (unsigned)pushButtonsThread, 10);
//      sendResponse(response);
    }
#endif


  #if CHECK_THREADS == 1
   uint32_t threadWatchdogTick = HAL_GetTick();

//   if (blinkStartTick != blinkEndTick && (blinkStartTick < blinkEndTick || threadWatchdogTick < blinkStartTick) && (threadWatchdogTick >= blinkEndTick))
//    firmwareReset(BLINK_TIMEOUT_ERROR);  // Ignore tick values before the "wrap"
   if (heartbeatStartTick != heartbeatEndTick && (heartbeatStartTick < heartbeatEndTick || threadWatchdogTick < heartbeatStartTick) && (threadWatchdogTick >= heartbeatEndTick))
    firmwareReset(MONITOR_TIMEOUT_ERROR);  // Ignore tick values before the "wrap"
   if (monitorStartTick != monitorEndTick && (monitorStartTick < monitorEndTick || threadWatchdogTick < monitorStartTick) && (threadWatchdogTick >= monitorEndTick))
    firmwareReset(MONITOR_TIMEOUT_ERROR);  // Ignore tick values before the "wrap"
//   if (writeMessageStartTick != writeMessageEndTick && (writeMessageStartTick < writeMessageEndTick || threadWatchdogTick < writeMessageStartTick) && (threadWatchdogTick >= writeMessageEndTick))
//    firmwareReset(WRITEMESSAGE_TIMEOUT_ERROR);  // Ignore tick values before the "wrap"
//   if (readPacketStartTick != readPacketEndTick && (readPacketStartTick < readPacketEndTick || threadWatchdogTick < readPacketStartTick) && (threadWatchdogTick >= readPacketEndTick))
//    firmwareReset(READPACKET_TIMEOUT_ERROR);  // Ignore tick values before the "wrap"
   if (parsePacketStartTick != parsePacketEndTick && (parsePacketStartTick < parsePacketEndTick || threadWatchdogTick < parsePacketStartTick) && (threadWatchdogTick >= parsePacketEndTick))
    firmwareReset(PARSEPACKET_TIMEOUT_ERROR);  // Ignore tick values before the "wrap"
   if (readIOStartTick != readIOEndTick && (readIOStartTick < readIOEndTick || threadWatchdogTick < readIOStartTick) && (threadWatchdogTick >= readIOEndTick))
    firmwareReset(READIO_TIMEOUT_ERROR);  // Ignore tick values before the "wrap"
   /*if (writeIOStartTick != writeIOEndTick && (writeIOStartTick < writeIOEndTick || threadWatchdogTick < writeIOStartTick) && (threadWatchdogTick >= writeIOEndTick))
    firmwareReset(WRITEIO_TIMEOUT_ERROR);  // Ignore tick values before the "wrap"*/
  #endif

  osThreadYield();
 }
 osThreadTerminate(NULL);

 if (!flagFirmwareReset) firmwareReset(THREAD_ERROR);
}
