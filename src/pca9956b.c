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
    colour_pwm[RED] = 0xFF;
    colour_pwm[AMBER] = 0x7F;
    colour_pwm[GREEN] = 0x01;
    colour_pwm[SEG] = 0xFF;
  /*for (int i = 0; i < NUM_LED_COL; i++)
    {
      colour_pwm[i] = PWM_OFF;
    }
*/
  for (int i = 0; i < NUM_ALL_LEDS; i++)
    {
      leds_pwm[i] = 0;
      leds_iref[i] = 255;
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

void dimDisplay(float value)
{
  for (int i = 0; i < NUM_ALL_LEDS; i++) leds_pwm[i] = (uint8_t) leds_pwm[i] * value;
}

/**
 *
 */
void refresh(void)
{
  uint8_t array[45];
  memcpy(&array, &leds_pwm, 23);
  memcpy(&array[23], &leds_iref, 23);
  i2c_WriteMulti(PWM_REGISTER_START | AUTO_INCREMENT, array, 46);
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
    for (int i = 0; i < NUM_ALL_LEDS; i++)
    {
      int8_t j = -1;
      if ((MASK_GRP_GREEN & (1 << i)) == (1 << i)) j = GREEN;
      if ((MASK_GRP_AMBER & (1 << i)) == (1 << i)) j = AMBER;
      if ((MASK_GRP_RED & (1 << i)) == (1 << i)) j = RED;
      if ((MASK_GRP_SEG & (1 << i)) == (1 << i)) j = SEG;
      if (j >= 0) leds_pwm[i] = (value & (1 << i )) ? colour_pwm[j] : PWM_OFF;
    }
  i2c_WriteMulti(PWM_REGISTER_START | AUTO_INCREMENT, leds_pwm, sizeof(leds_pwm)/sizeof(leds_pwm[0]));
}

void setGroupPWM( uint8_t red_pwm, uint8_t amber_pwm, uint8_t green_pwm, uint8_t seg_pwm )
{

}

void dimLeds( uint8_t value )
{

}

/**
 *
 * @param value
 */
void display( char* value, uint8_t pwm)
{
  uint8_t ds1, ds2, ds1raw, ds2raw;
  ds1 = digitsToInt(value, 0, 1, 10);
  ds2 = digitsToInt(value, 1, 1, 10);
  ds1raw = ds1_DigitLookup[ds1];
  ds2raw = ds2_DigitLookup[ds2];


  displayBits(ds1raw, ds2raw, pwm);
//  for (int i = 0; i < 7; i++)
//    {
//      leds_pwm[i + L9] = (ds1_DigitLookup[ds1] & (1 << i)) ? PWM_ON_VALUE : 0;
//      leds_pwm[i + L16] = (ds2_DigitLookup[ds2] & (1 << i)) ? PWM_ON_VALUE : 0;
//    }
}

/**
 * @brief This function takes the display data structure and unpacks it into a array buffer to be written to PCA9956B controller
 * @param data - dispdata structure
 */
void updateDisplay(dispdata data)
{
  static uint8_t buffer[NUM_ALL_LEDS * 2];    // One buffer to write to I2c, holds pwm & iref for all leds.  Written as a block of regs.
  static uint32_t grpmask = 0;
  static uint8_t shift = 0;

  if ((data.ledStateBuffer & (1 << 23)) == (1 << 23))
    {
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }

  for (int i = 0; i < NUM_ALL_LEDS; i++)
    {
      if ((MASK_GRP_RED & (1 << i)) == (1 << i)) shift = 0; //grpmask = 0xFF;
      if ((MASK_GRP_AMBER & (1 << i)) == (1 << i)) shift = 8; //grpmask = (0xFF << 8);
      if ((MASK_GRP_GREEN & (1 << i)) == (1 << i)) shift = 16; //grpmask = (0xFF << 16);
      if ((MASK_GRP_SEG & (1 << i)) == (1 << i)) shift = 24; //grpmask = (0xFF << 24);


      if ((data.ledStateBuffer & (1 << i)) == (1 << i))
        {
          uint32_t mask = 0xFF << shift;
          buffer[i] = (data.group_pwm & mask) >> shift;
        } else
          {
            buffer[i] = 0;
          }
      buffer[i + NUM_ALL_LEDS] = (data.group_iref & (0xFF << shift)) >> shift;
    }

  i2c_WriteMulti(PWM_REGISTER_START | AUTO_INCREMENT, buffer, sizeof(buffer)/sizeof(buffer[0]));
}

void displayBits( uint8_t ds1, uint8_t ds2, uint8_t newpwm)
{
  for (int i = 0; i < 7; i++)
      {
        leds_pwm[i + L9] = (ds1 & (1 << i)) ? PWM_ON_VALUE : 0;
        leds_pwm[i + L16] = (ds2 & (1 << i)) ? PWM_ON_VALUE : 0;
//      leds_pwm[i + L9] = (ds1 & (1 << i)) ? newpwm : 0;
//      leds_pwm[i + L16] = (ds2 & (1 << i)) ? newpwm : 0;
      }
  refresh();
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

void pwm7seg (uint8_t pwm)
{
  for (int i = 0; i < 14; i++)
    {
      leds_pwm[i + L9] = pwm;
      i2c_byte_write( pwm_register_access(i + L9), pwm);
    }
}

void setleds (uint32_t pwm, uint32_t iref)
{
  for (int i = 0; i < NUM_ALL_LEDS; i++)
    {
//      leds.pwm[i]
    }
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

void current7seg( uint8_t cur )
{
  for (int i = 0; i < 14; i++ )
    {
      leds_iref[i + L9] = cur;
      i2c_byte_write( current_register_access(i + L9), cur);
    }
}

/*
 *
 */
void segoff ( void )
{
  for (int i = 0; i < 14; i++)
    {
      leds_pwm[i + L9] = 0;
      i2c_byte_write( pwm_register_access(i + L9), 0);
    }
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
