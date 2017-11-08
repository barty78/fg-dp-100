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

void initPCA(void)
{
    char init_array[] = {
        AUTO_INCREMENT | REGISTER_START,  			//  Command
        0x00, 0x00,                                 //  MODE1, MODE2
        0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,         //  LEDOUT[5:0]
        0x80, 0x00,                                 //  GRPPWM, GRPFREQ
    };

    pwm( ALLPORTS, 0.0 );
    current( ALLPORTS, 0.1 );

    i2c_write( init_array, sizeof( init_array ) );
}

void resetPCA(void)
{
	char    v   = 0x06;
	i2c_write( 0x00, &v, 1 );
}

void blink(uint8_t duty, uint8_t period)
{
	char data[2];

	if (period > 16)
		period = 16;

	data[0] = (float) duty / 256;
	data[1] = ((float) period * 15.26) - 1;

	i2c_write( GRPPWM, data, 2);
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

void pwm( int port, float v )
{
	char    reg_addr;

	reg_addr    = pwm_register_access( port );
	i2c_byte_write( reg_addr, (uint8_t)(v * 255.0) );
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
	i2c_write( reg_addr, data, sizeof(data));
}

void pwmall( float *vp )
{
    char reg_addr;
	char data[ n_of_ports ];

    reg_addr    = pwm_register_access( 0 );

    for ( int i = 0; i <= n_of_ports; i++ )
        data[ i ]   = (uint8_t)(*vp++ * 255.0);

    i2c_write( reg_addr, data, sizeof( data ) );
}

void current( int port, float v )
{
    char    reg_addr;

    reg_addr    = current_register_access( port );
    i2c_byte_write( reg_addr, (uint8_t)(v * 255.0) );
}

void currentall( float *vp )
{
	char    reg_addr;
	char    data[ n_of_ports + 0 ];

    reg_addr    = pwm_register_access( 0 );

    for ( int i = 0; i <= n_of_ports; i++ )
        data[ i ]   = (uint8_t)(*vp++ * 255.0);

    i2c_write( reg_addr, data, sizeof( data ) );
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
