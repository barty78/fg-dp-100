///////////////////////////////////////////////////////////////////////////////
//
// COMMS.C
//
//  DESCRIPTION: Low level TIB Communications Functions
//
//  CREATED:     13 OCT 2016
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <string.h>

#include "global.h"
#include "comms.h"

///////////////////////////////////////////////////////////////////////////////
//
// HAL_UART_RxCpltCallback
//
//  DESCRIPTION: Called when USART Receive Complete
//
//  NOTES:       1. Increments Rx Circular Buffer Head Pointer, (wrapping if necessary)
//               2. USART is set to receive only a single byte
//               3. Re-enables USART1 receive interrup
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
 UBaseType_t uxSavedInterruptStatus;
 uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
#ifdef RS485
  if (UartHandle == handleLPUART1)
#else
  if (UartHandle == handleUART1)
#endif
  {
	  if (++rxMessageHead >= RX_BUFFER_LENGTH) rxMessageHead = 0;
#ifdef RS485
   HAL_UART_Receive_IT(handleLPUART1, (uint8_t*)(&(rxBuffer[rxMessageHead])), 1);
#else
   HAL_UART_Receive_IT(handleUART1, (uint8_t*)(&(rxBuffer[rxMessageHead])), 1);
#endif
  }
 taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

///////////////////////////////////////////////////////////////////////////////
//
// HAL_UART_TxCpltCallback
//
//  DESCRIPTION: Called when USART Transmit Complete
//
//  NOTES:       1. Increments Tx Circular Buffer Tail Pointer, (wrapping if necessary)
//               2. Sets global "flagByteTransmitted" flag
//               3. Reenables RX
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
 UBaseType_t uxSavedInterruptStatus;

 uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
#ifdef RS485
  if (UartHandle == handleLPUART1)
#else
  if (UartHandle == handleUART1)
#endif
  {
//   if (++txMessageTail >= TX_BUFFER_LENGTH) txMessageTail = 0;
      txMessageTail = txMessageHead;
   flagByteTransmitted = 1;  // Set transmission flag: transfer complete
   HAL_GPIO_TogglePin(RS485_EN_GPIO_Port, RS485_EN_Pin);

  }
 taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

///////////////////////////////////////////////////////////////////////////////
//
// crc_calcCrc8
//
//  DESCRIPTION: Calculate a Dallas CRC8 checksum
//
//  NOTES:       1. Specified in "TIB Functional Specification", (Section 5: "Comms CRC Generation")
//
//  AUTHOR: Chris Stephenson <chris@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint8_t crc_calcCrc8(void *data_pointer, uint16_t number_of_bytes)
{
 uint8_t temp1, bit_counter, feedback_bit, crc8_result = 0;
 uint8_t *ptr = (uint8_t *) data_pointer;

 while (number_of_bytes--)
 {
  temp1 = *ptr++;
  for (bit_counter=8; bit_counter; bit_counter--)
  {
   feedback_bit = (crc8_result & 0x01);
   crc8_result >>= 1;
   if (feedback_bit ^ (temp1 & 0x01))
   {
    crc8_result ^= 0x8c;
   }
   temp1 >>= 1;
  }
 }

 return (crc8_result);
}

///////////////////////////////////////////////////////////////////////////////
//
// initComms
//
//  DESCRIPTION: Initialise global Comms variables and buffers
//
//  NOTES:       1. Called once from "main" function before threads commence
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint8_t initComms()
{
 taskENTER_CRITICAL();
  flagByteTransmitted = 1;
  flagPacketSent = 0;

  rxMessageTail = 0;
  rxMessageHead = 0;
  txMessageTail = 0;
  txMessageHead = 0;

  packetTail = 0;
  packetHead = 0;
  packetPointer = 0;
  flagPacketReceived = 0;

  for (uint32_t i=0; i<RX_BUFFER_LENGTH; i++) rxBuffer[i] = 0;

  for (uint32_t i=0; i<PACKET_BUFFER_LENGTH; i++) packetBuffer[i] = malloc(RX_BUFFER_LENGTH * sizeof(char));

 taskEXIT_CRITICAL();
HAL_UART_Receive_DMA(handleLPUART1, (uint8_t*)dma_rx_buf, DMA_BUFFER_LENGTH);
#ifdef RS485
 HAL_UART_Receive_IT(handleLPUART1, (uint8_t*)(&(rxBuffer[rxMessageHead])), 1);
#else
 HAL_UART_Receive_IT(handleUART1, (uint8_t*)(&(rxBuffer[rxMessageHead])), 1);
#endif
 return 0;
}


///////////////////////////////////////////////////////////////////////////////
//
// writeMessage
//
//  DESCRIPTION: Adds message to head of transmit circular buffer
//
//  NOTES:       1. "msg" parameter must be null terminated.
//               2. Increments Tx Circular Buffer Head Pointer, (wrapping if necessary)
//               3. The "writeMessageThread" continually monitors the Tx Circular Buffer Pointers
//                  to determine if there are bytes to be sent.
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint8_t writeMessage(char* msg)
{
 taskENTER_CRITICAL();
 if (strlen(msg) > (TX_BUFFER_LENGTH - txMessageHead))
   {
     txMessageHead = 0;
     txMessageTail = 0;
   }
  for (uint32_t i=0; i<strlen(msg); i++)
  {
   txBuffer[txMessageHead++] = msg[i];
   if (txMessageHead >= TX_BUFFER_LENGTH)
     {
       txMessageHead = 0;


     }
  }
 taskEXIT_CRITICAL();

 return 0;
}

///////////////////////////////////////////////////////////////////////////////
//
// sendResponse
//
//  DESCRIPTION: Constructs and sends a valid TIB response packet from the null terminated "response" string argument
//
//  NOTES:       1. Calculates and appends a CRC byte
//               2. Terminates the response packet with a <LF> character
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void sendResponse(char* response)
{
 uint32_t len = strlen(response);
 uint8_t crc = crc_calcCrc8(response, len);
 char msg[RESPONSE_BUFFER_LENGTH];

 sprintf(msg, "%s%02X\n", response, crc);  // Append CRC to message before writing out to Tx
 strcpy(lastMsg, msg);
 writeMessage(msg);
}
