///////////////////////////////////////////////////////////////////////////////
//
// I2C.C
//
//  DESCRIPTION: Low Level I2C Routines
//
//  CREATED:     20 OCT 2017
//  AUTHOR:      Peter Bartlett <peter@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>

#include "global.h"
#include "i2c.h"

#define FLAG_TIMEOUT ((int)0x1000)


///////////////////////////////////////////////////////////////////////////////
//
// HAL_I2C_MasterRxCpltCallback
//
//  DESCRIPTION: Called when I2C Receive Complete
//
//  NOTES:       1. Increments Rx Circular Buffer Head Pointer, (wrapping if necessary)
//               2. USART is set to receive only a single byte
//               3. Re-enables USART1 receive interrupt
//
//  AUTHOR: Peter Bartlett <peter@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2CHandle)
{
 UBaseType_t uxSavedInterruptStatus;

 uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
  if (I2CHandle == handleI2C2)
  {
   //if (++rxMessageHead >= RX_BUFFER_LENGTH) rxMessageHead = 0;
   //HAL_I2C_Master_Receive_DMA(handleI2C2, DEFAULT_I2C_ADDR, (uint8_t*)(&(aRxBuffer[rxMessageHead])), 1);
  }
 taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

///////////////////////////////////////////////////////////////////////////////
//
// HAL_I2C_MasterTxCpltCallback
//
//  DESCRIPTION: Called when I2C Transmit Complete
//
//  NOTES:       1. Increments Tx Circular Buffer Tail Pointer, (wrapping if necessary)
//               2. Sets global "flagByteTransmitted" flag
//
//  AUTHOR: Peter Bartlett <peter@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2CHandle)
{
 UBaseType_t uxSavedInterruptStatus;

 uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
  if (I2CHandle == handleI2C2)
  {
   //if (++txMessageTail >= TX_BUFFER_LENGTH) txMessageTail = 0;
   //flagI2CByteTransmitted = 1;  // Set transmission flag: transfer complete
  }
 taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2CHandle)
{
}

void initI2C(void)
{
}

I2C_Result_t i2c_byte_write(uint8_t register_address, uint8_t data)
{
	uint8_t d[2];

		/* Format array to send */
		d[0] = register_address;
		d[1] = data;

		/* Try to transmit via I2C */
		if (HAL_I2C_Master_Transmit_IT(handleI2C2, (uint16_t)DEFAULT_I2C_ADDR, (uint8_t *)d, 2) != HAL_OK) {
			/* Check error */
			if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {

			}

			/* Return error */
			return I2C_Result_Error;
		}

		/* Return OK */
		return I2C_Result_Ok;
}

I2C_Result_t i2c_byte_read(uint8_t register_address, uint8_t* data)
{
	/* Send address */
	if (HAL_I2C_Master_Transmit_IT(handleI2C2, (uint16_t)DEFAULT_I2C_ADDR, &register_address, 1) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {

		}

		/* Return error */
		return I2C_Result_Error;
	}

	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive_IT(handleI2C2, DEFAULT_I2C_ADDR, data, 1) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {

		}

		/* Return error */
		return I2C_Result_Error;
	}

	/* Return OK */
	return I2C_Result_Ok;
}

I2C_Result_t i2c_read (uint8_t register_address, uint8_t* data, uint16_t count)
{
	/* Send register address */
	if (HAL_I2C_Master_Transmit_IT(handleI2C2, (uint16_t)DEFAULT_I2C_ADDR, &register_address, 1) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {

		}

		/* Return error */
		return I2C_Result_Error;
	}

	/* Receive multiple byte */
	if (HAL_I2C_Master_Receive_IT(handleI2C2, DEFAULT_I2C_ADDR, data, count) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {

		}

		/* Return error */
		return I2C_Result_Error;
	}

	/* Return OK */
	return I2C_Result_Ok;
}

I2C_Result_t i2c_write(uint8_t register_address, uint8_t* data, uint16_t count)
{
	/* Try to transmit via I2C */
	if (HAL_I2C_Mem_Write_IT(handleI2C2, DEFAULT_I2C_ADDR, register_address, register_address, data, count) != HAL_OK) {
		/* Check error */
		if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {

		}

		/* Return error */
		return I2C_Result_Error;
	}

	/* Return OK */
	return I2C_Result_Ok;
}
