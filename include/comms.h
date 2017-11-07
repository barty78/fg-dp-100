#ifndef COMMS_H
#define COMMS_H

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#define RX_BUFFER_LENGTH 256
#define TX_BUFFER_LENGTH 512
#define PACKET_BUFFER_LENGTH 32
#define RESPONSE_BUFFER_LENGTH 256

UART_HandleTypeDef *handleUART1, *handleLPUART1;
uint8_t flagByteTransmitted;
char lastMsg[PACKET_BUFFER_LENGTH];
char rxBuffer[RX_BUFFER_LENGTH], txBuffer[TX_BUFFER_LENGTH];
char* packetBuffer[PACKET_BUFFER_LENGTH];

int rxMessageTail, rxMessageHead, txMessageTail, txMessageHead;
int packetTail, packetHead, packetPointer, flagPacketReceived;

uint8_t crc_calcCrc8(void *data_pointer, uint16_t number_of_bytes);
uint8_t initComms();
uint8_t writeMessage(char* msg);
void sendResponse(char* response);

#endif
