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

/**
 *
 */
void pca9956_init(void)
{
  for (int i = 0; i < NUM_ALL_LEDS; i++)
    {
      leds_pwm[i] = 0;
      leds_iref[i] = 127;
    }

	pca9956_hardreset();
	HAL_Delay(100);
	i2c_WriteMulti(AUTO_INCREMENT | REGISTER_START, init_array, (sizeof(init_array)/sizeof(init_array[0]) + 1));
}

/**
 *
 */
void pca9956_status(void)
{
	char tmp;
	char reg_addr = PWMALL;

	i2c_byte_read(PWMALL, tmp);
}

/**
 *
 */
void pca9956_hardreset(void)
{
	HAL_GPIO_WritePin(PCA_RST_GPIO_Port, PCA_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PCA_RST_GPIO_Port, PCA_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(PCA_OE_GPIO_Port, PCA_OE_Pin, GPIO_PIN_RESET);
}

/**
 *
 */
void pca9956_reset(void)
{
	char    v   = 0x06;
	//i2c_write( 0x00, &v, 1 );
}

/**
 *
 */
void refresh(void)
{
  i2c_WriteMulti(PWM_REGISTER_START | AUTO_INCREMENT, leds_pwm, 23);
  i2c_WriteMulti(IREF_REGISTER_START | AUTO_INCREMENT, leds_iref, 23);
}

/**
 *
 * @param en
 * @param duty
 * @param period
 */
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

/**
 *
 * @param value
 */
void pwmleds( uint32_t value )
{
  uint8_t ledData[n_of_ports];

  for (int i = 0; i < n_of_ports; i++)
    {
      ledData[i] = value & (1 << i );
    }
  i2c_WriteMulti(PWM_REGISTER_START | AUTO_INCREMENT, ledData, sizeof(ledData)/sizeof(ledData[0]));
}

/**
 *
 * @param value
 */
void display( char* value )
{
  uint8_t ds1 = digitsToInt(value, 0, 1, 10);
  uint8_t ds2 = digitsToInt(value, 1, 1, 10);

  for (int i = 0; i < 7; i++)
    {
      leds_pwm[i + L9] = (ds1_DigitLookup[ds1] & (1 << i)) ? PWM_ON_VALUE : 0;
      leds_pwm[i + L16] = (ds2_DigitLookup[ds2] & (1 << i)) ? PWM_ON_VALUE : 0;

    }
}

/**
 *
 * @param reg
 */
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

/**
 *
 */
void alloff ( void )
{
  for (int i = 0; i < NUM_ALL_LEDS; i++)
    {
      leds_pwm[i] = 0;
    }
	i2c_byte_write(PWMALL, 0);
}

/**
 *
 * @param port
 * @param pwm
 */
void pwm( int port, uint8_t pwm )
{
  leds_pwm[port] = pwm;
  i2c_byte_write( pwm_register_access(port), pwm );
}

/**
 *
 * @param pwm
 */
void pwmall( uint8_t pwm )
{
  for (int i = 0; i < NUM_ALL_LEDS; i++)
    {
      leds_pwm[i] = 0;
    }
  i2c_byte_write( IREFALL, pwm );
}

/**
 *
 * @param port
 * @param cur
 */
void current( int port, uint8_t cur )
{
  leds_iref[port] = cur;
  i2c_byte_write( current_register_access(port), cur );
}

/**
 *
 * @param cur
 */
void currentall( uint8_t cur )
{
  for (int i = 0; i < NUM_ALL_LEDS; i++)
      {
        leds_iref[i] = 0;
      }
  i2c_byte_write( IREFALL, cur);
}

/**
 *
 * @param cur
 */
void currentDisplay( uint8_t cur)
{
  uint8_t data[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  for (int i = 0; i < NUM_LEDS; i++)
    {
      data[i] = cur;
    }

  i2c_WriteMulti( DS1_IREF_REGISTER_START | AUTO_INCREMENT, data, sizeof(data)/sizeof(data[0]));
}

/**
 *
 * @param port
 * @return
 */
char pwm_register_access( int port )
{
    if ( port < n_of_ports )
        return ( PWM_REGISTER_START + port );

    return ( PWMALL );
}

/**
 *
 * @param port
 * @return
 */
char current_register_access( int port )
{
    if ( port < n_of_ports )
        return ( IREF_REGISTER_START + port );

    return ( IREFALL );
}
