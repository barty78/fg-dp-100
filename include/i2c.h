#ifndef I2C_H
#define I2C_H

#include "stm32l4xx_hal.h"
#include "global.h"
#include "pca9956b.h"
#include "cmsis_os.h"

/**
 * @brief  I2C result enumeration
 */
typedef enum {
	I2C_Result_Ok = 0x00, /*!< Everything OK */
	I2C_Result_Error      /*!< An error has occurred */
} I2C_Result_t;

/*
#define RX_BUFFER_LENGTH 256
#define TX_BUFFER_LENGTH 4096
#define PACKET_BUFFER_LENGTH 32
#define RESPONSE_BUFFER_LENGTH 256
*/

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

/* Size of Transmission buffer */
//#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
#define TXBUFFERSIZE                      (512)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

char aTxBuffer[TXBUFFERSIZE];
char aRxBuffer[RXBUFFERSIZE];

I2C_HandleTypeDef *handleI2C2;
int flagI2CByteTransmitted;

I2C_Result_t i2c_read(uint8_t register_adddress, uint8_t *data, uint16_t count);
I2C_Result_t i2c_byte_read(uint8_t register_address, uint8_t data);
I2C_Result_t i2c_write(uint8_t dev_address, uint8_t *data, uint8_t size);
I2C_Result_t i2c_byte_write(uint8_t register_address, uint8_t data);

#endif
