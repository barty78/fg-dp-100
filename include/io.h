#ifndef IO_H
#define IO_H

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#define LED_ON  GPIO_PIN_SET
#define LED_OFF GPIO_PIN_RESET

#define PWR_ON  GPIO_PIN_SET
#define PWR_OFF GPIO_PIN_RESET

//#define ADC_CONFIG 0xA000  // Enable Program Mode, 833 samples per second, NAP Mode
#define ADC_CONFIG 0x8000  // Enable Program Mode, 208 samples per second, NAP Mode

#define SHUTDOWN_DELAY (25000)     // 25 seconds Shutdown delay, (= 25,000 milliseconds)
#define SHUTDOWN_DELAY_MSG (5000)  // 5 seconds Shutdown message delay, (= 5,000 milliseconds)

#define MIN_TRUCK_VOLTAGE 10  // System will be shutdown when voltages fall below these levels
#define MIN_BATT_VOLTAGE 10

#define VOLTAGE_FILTER_LENGTH 50  // Recursive Averaging Filter Length
#define ANALOG_FILTER_LENGTH 20   // Recursive Averaging Filter Length

#define LOW_VOLTAGE_SHUTDOWN_DELAY (60000)      // 60 seconds Shutdown Delay, (system will be shutdown when voltages fall below minimum thresholds)
#define LOW_VOLTAGE_SHUTDOWN_DELAY_MSG (40000)  // 40 seconds Shutdown message delay

#define TRUCK_VOLTAGE_DELAY_MSG (5000)  // 5 seconds Low Truck Voltage Message Delay, (A message will be sent when truck voltage remains below the minimum threshold for this duration)

#define DEBOUNCE_DELAY (100)  // 100 millisecond Push Button Debounce delay

#define HOLD_DELAY (2000)	// 2 second long button press hold delay

#define MOVING_AVERAGE(average, sample, length) (average + sample - average/(float)length)  // Calculate recursive moving average values


typedef struct
{
 GPIO_TypeDef* port;
 uint16_t pin;
 GPIO_PinState state;
} DigitalInput;

typedef struct
{
 GPIO_TypeDef* port;
 uint16_t pin;
} GPIOPin;


CRC_HandleTypeDef* handleCRC;
TIM_HandleTypeDef *handleTIM1;//, *handleTIM6, *handleTIM8;
ADC_HandleTypeDef* handleADC1;
DMA_HandleTypeDef* handleDMA1;

uint8_t analogChannel;
uint16_t ADCValues[2];

uint8_t flagADCConversionCompleted;
uint8_t pushButtons;
GPIOPin buttonInputs[3];
float displaySuppV, displaySuppVSum;

uint8_t initIO();
uint16_t readDigitalInputs();
HAL_StatusTypeDef readAnalogInput(uint8_t channel);
uint8_t readButtonInputs(uint8_t* debounceFlag, uint32_t* debounceStart, uint32_t* debounceEnd);
uint8_t readADC();
uint8_t setPower(uint8_t state);

#endif
