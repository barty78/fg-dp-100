/*
 * pca9956.c
 *
 *  Created on: 20Oct.,2017
 *      Author: Peter Bartlett <peter@masters-young.com.au>
 */

///////////////////////////////////////////////////////////////////////////////
//
// PCA9956B.C
//
//  DESCRIPTION: Low Level PCA9956B Routines
//
//  CREATED:     17 OCT 2016
//  AUTHOR:      Peter Bartlett <peter@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>

#include "global.h"
#include "pca9956b.h"
#include "i2c.h"

void pca9956_init(void)
{
	char tmp;

	pca9956_hardreset();
	HAL_Delay(100);	i2c_write( REGISTER_START, init_array, sizeof(init_array)/sizeof(init_array[0]));

	pwmall( 0.0 );
	HAL_Delay(100);
	currentall( 0.1 );
	HAL_Delay(100);
	i2c_byte_read(PWMALL, tmp);
}

void pca9956_status(void)
{
	char tmp;
	char reg_addr = PWMALL;

	i2c_byte_read(PWMALL, tmp);
}

void pca9956_hardreset(void)
{
	HAL_GPIO_WritePin(PCA_RST_GPIO_Port, PCA_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PCA_RST_GPIO_Port, PCA_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(PCA_OE_GPIO_Port, PCA_OE_Pin, GPIO_PIN_RESET);
}

void pca9956_reset(void)
{
	char    v   = 0x06;
	//i2c_write( 0x00, &v, 1 );
}

void blink(uint8_t en, uint8_t duty, uint8_t period)
{
	taskENTER_CRITICAL();
	if (en)
	{
		i2c_byte_write(MODE2, DMBLINK);
		i2c_byte_write(GRPFREQ, period);
		i2c_byte_write(GRPPWM, duty);

	} else
	{
		i2c_byte_write(MODE2, 5);
		i2c_byte_write(GRPPWM, 0xFF);
	}
	taskEXIT_CRITICAL();
}

void display( char* value )
{
	uint8_t ds1 = digitsToInt(value, 0, 1, 10);
	uint8_t ds2 = digitsToInt(value, 1, 1, 10);

	uint8_t tmp[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	for (int i = 0; i < 7; i++)
	{
		if (ds1_DigitLookup[ds1] & (1 << i))
		{
			tmp[i] = PWM_ON_VALUE;
		}
		if (ds2_DigitLookup[ds2] & (1 << i))
		{
			tmp[i+7] = PWM_ON_VALUE;
		}
	}
	pwmdisplay(&tmp);
}

void reg( int reg )
{
	unsigned char data[1];
	data[0] = reg;

	if (HAL_I2C_Master_Transmit(handleI2C2, DEFAULT_I2C_ADDR, data, 1, 1) != HAL_OK) {
		if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
		/* Return error */
		return I2C_Result_Error;
	}
	/* Return OK */
	return I2C_Result_Ok;
}

void alloff ( void )
{
	i2c_byte_write(PWMALL, 0);
}

void pwm( int port, float v )
{

	i2c_byte_write( pwm_register_access(port), (uint8_t)(v * 255.0) );
}

void pwmdisplay( float *vp )
{
	char reg_addr;
	char data[14];

	reg_addr = pwm_register_access(9);

	for (int i = 0; i <= 14; i++ )
	{
		data[i] = (uint8_t)(*vp++ * 255.0);
	}
	//i2c_write( reg_addr, data, sizeof(data));
}

void pwmall( float v )
{
	unsigned char data[2];

	data[0] = IREFALL;
	data[1] = (uint8_t)(v * 255.0);

    if (HAL_I2C_Master_Transmit(handleI2C2, DEFAULT_I2C_ADDR, data, sizeof(data), 1) != HAL_OK) {
            				/* Check error */
            				if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {
            					Error_Handler();
            				}

            				/* Return error */
            				return I2C_Result_Error;
            			}

            			/* Return OK */
            			return I2C_Result_Ok;
    //i2c_write( reg_addr, data, sizeof( data ) );
}

void current( int port, float v )
{
    unsigned char data[2];

    data[0] = current_register_access( port );
    data[1] = (uint8_t)(v * 255.0);

    if (HAL_I2C_Master_Transmit(handleI2C2, DEFAULT_I2C_ADDR, data, sizeof(data), 1) != HAL_OK) {
    				/* Check error */
    				if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {
    					Error_Handler();
    				}

    				/* Return error */
    				return I2C_Result_Error;
    			}

    			/* Return OK */
    			return I2C_Result_Ok;

    //i2c_byte_write( reg_addr, (uint8_t)(v * 255.0) );
}

void currentall( float v )
{
	unsigned char data[2];

	data[0] = IREFALL;
	data[1] = (uint8_t)(v * 255.0);


    if (HAL_I2C_Master_Transmit(handleI2C2, DEFAULT_I2C_ADDR, data, sizeof(data), 1) != HAL_OK) {
        				/* Check error */
        				if (HAL_I2C_GetError(handleI2C2) != HAL_I2C_ERROR_AF) {
        					Error_Handler();
        				}

        				/* Return error */
        				return I2C_Result_Error;
        			}

        			/* Return OK */
        			return I2C_Result_Ok;

//    i2c_write( reg_addr, data, sizeof( data ) );
}

char pwm_register_access( int port )
{
    if ( port < n_of_ports )
        return ( PWM_REGISTER_START + port );

    return ( PWMALL );
}

char current_register_access( int port )
{
    if ( port < n_of_ports )
        return ( IREF_REGISTER_START + port );

    return ( IREFALL );
}

/*int number_of_ports( void )
{
    return ( n_of_ports );
}*/
