#ifndef PCA9956B_H
#define PCA9956B_H

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
//#include "i2c.h"


#define     ALLPORTS        0xFF
#define     DEFAULT_PWM     1.0
#define     DEFAULT_CURRENT 0.1
#define		n_of_ports		24
#define		PWM_ON_VALUE	200

static const uint8_t ds1_DigitLookup[10] =
{
0x77, 0x11, 0x3E, 0x3B, 0x59, 0x6B, 0x6F, 0x31, 0x7F, 0x79
};

static const uint8_t ds2_DigitLookup[10] =
{
0x7B, 0x18, 0x6D, 0x3D, 0x1E, 0x37, 0x77, 0x19, 0x7F, 0x1F
};



/** PCA9956A pin names high-level API i.e. LedPwmOutCC */
typedef enum {
	L0,            /**< LED0  pin                              */
	L1,            /**< LED1  pin                              */
	L2,            /**< LED2  pin                              */
	L3,            /**< LED3  pin                              */
	L4,            /**< LED4  pin                              */
	L5,            /**< LED5  pin                              */
	L6,            /**< LED6  pin                              */
	L7,            /**< LED7  pin                              */
	L8,            /**< LED8  pin                              */
	L9,            /**< LED9  pin                              */
	L10,           /**< LED10 pin                              */
	L11,           /**< LED11 pin                              */
	L12,           /**< LED12 pin                              */
	L13,           /**< LED13 pin                              */
	L14,           /**< LED14 pin                              */
	L15,           /**< LED15 pin                              */
	L16,           /**< LED16 pin                              */
	L17,           /**< LED17 pin                              */
	L18,           /**< LED18 pin                              */
	L19,           /**< LED19 pin                              */
	L20,           /**< LED20 pin                              */
	L21,           /**< LED21 pin                              */
	L22,           /**< LED22 pin                              */
	L23,           /**< LED23 pin                              */
	L_NC = ~0x0L   /**< for when the pin is left no-connection */
} LedPinName;

typedef struct {
	LedPinName led;
} LED_PwmOut;

/** Name of the PCA9956A registers (for direct register access) */
enum command_reg {
	MODE1,          /**< MODE1 register      */
	MODE2,          /**< MODE2 register      */
	LEDOUT0,        /**< LEDOUT0 register    */
	LEDOUT1,        /**< LEDOUT1 register    */
	LEDOUT2,        /**< LEDOUT2 register    */
	LEDOUT3,        /**< LEDOUT3 register    */
	LEDOUT4,        /**< LEDOUT4 register    */
	LEDOUT5,        /**< LEDOUT5 register    */
	GRPPWM,         /**< GRPPWM register     */
	GRPFREQ,        /**< GRPFREQ register    */
	PWM0,           /**< PWM0 register       */
	PWM1,           /**< PWM1 register       */
	PWM2,           /**< PWM2 register       */
	PWM3,           /**< PWM3 register       */
	PWM4,           /**< PWM4 register       */
	PWM5,           /**< PWM5 register       */
	PWM6,           /**< PWM6 register       */
	PWM7,           /**< PWM7 register       */
	PWM8,           /**< PWM8 register       */
	PWM9,           /**< PWM9 register       */
	PWM10,          /**< PWM10 register      */
	PWM11,          /**< PWM11 register      */
	PWM12,          /**< PWM12 register      */
	PWM13,          /**< PWM13 register      */
	PWM14,          /**< PWM14 register      */
	PWM15,          /**< PWM15 register      */
	PWM16,          /**< PWM16 register      */
	PWM17,          /**< PWM17 register      */
	PWM18,          /**< PWM18 register      */
	PWM19,          /**< PWM19 register      */
	PWM20,          /**< PWM20 register      */
	PWM21,          /**< PWM21 register      */
	PWM22,          /**< PWM22 register      */
	PWM23,          /**< PWM23 register      */
	IREF0,          /**< IREF0 register      */
	IREF1,          /**< IREF1 register      */
	IREF2,          /**< IREF2 register      */
	IREF3,          /**< IREF3 register      */
	IREF4,          /**< IREF4 register      */
	IREF5,          /**< IREF5 register      */
	IREF6,          /**< IREF6 register      */
	IREF7,          /**< IREF7 register      */
	IREF8,          /**< IREF8 register      */
	IREF9,          /**< IREF9 register      */
	IREF10,         /**< IREF10 register     */
	IREF11,         /**< IREF11 register     */
	IREF12,         /**< IREF12 register     */
	IREF13,         /**< IREF13 register     */
	IREF14,         /**< IREF14 register     */
	IREF15,         /**< IREF15 register     */
	IREF16,         /**< IREF16 register     */
	IREF17,         /**< IREF17 register     */
	IREF18,         /**< IREF18 register     */
	IREF19,         /**< IREF19 register     */
	IREF20,         /**< IREF20 register     */
	IREF21,         /**< IREF21 register     */
	IREF22,         /**< IREF22 register     */
	IREF23,         /**< IREF23 register     */
	OFFSET  = 0x3A, /**< OFFSET register     */
	SUBADR1,        /**< SUBADR1 register    */
	SUBADR2,        /**< SUBADR2 register    */
	SUBADR3,        /**< SUBADR3 register    */
	ALLCALLADR,     /**< ALLCALLADR register */
	PWMALL,         /**< PWMALL register     */
	IREFALL,        /**< IREFALL register    */
	EFLAG0,         /**< EFLAG0 register     */
	EFLAG1,         /**< EFLAG1 register     */
	EFLAG2,         /**< EFLAG2 register     */
	EFLAG3,         /**< EFLAG3 register     */
	EFLAG4,         /**< EFLAG4 register     */
	EFLAG5,         /**< EFLAG5 register     */

	REGISTER_START          = MODE1,
	LEDOUT_REGISTER_START   = LEDOUT0,
	PWM_REGISTER_START      = PWM0,
	IREF_REGISTER_START     = IREF0,
	DS1_PWM_REGISTER_START	= PWM9,
	DS2_PWM_REGISTER_START	= PWM16,
	DS1_IREF_REGISTER_START	= IREF9,
	DS2_IREF_REGISTER_START	= IREF16
};


enum {
    DEFAULT_I2C_ADDR    = 0x01<<1,
	DMBLINK				= 0x20,
    AUTO_INCREMENT      = 0x80
};

static const uint8_t init_array[] = {
        AUTO_INCREMENT | REGISTER_START,  			//  Command
        0x00, 0x00,                                 //  MODE1, MODE2
//        0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,         //  LEDOUT[5:0]
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,         //  LEDOUT[5:0]
        0x80, 0x00,                                 //  GRPPWM, GRPFREQ
    };



//const int n_of_ports = 24;

void pca9956_init();
void pca9956_reset();

void display(char* value);

void blink(uint8_t en, uint8_t duty, uint8_t period);

void pwm(int port, float v);
void current(int port, float v);
void pwmdisplay(float* vp);
void pwmall(float v);
void currentall(float v);

char pwm_register_access( int port );
char current_register_access( int port );


#endif
