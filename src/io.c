///////////////////////////////////////////////////////////////////////////////
//
// IO.C
//
//  DESCRIPTION: Low Level I/O Routines
//
//  CREATED:     17 OCT 2016
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>

#include "global.h"
#include "io.h"


///////////////////////////////////////////////////////////////////////////////
//
// HAL_ADC_ConvCpltCallback
//
//  DESCRIPTION: Called when MCU's internal 12-bit ADC DMA Scan Complete
//
//  NOTES:       1. Sets global "flagADCConversionCompleted" flag
//                  (This will eventually result in a call to "HAL_ADC_Start_DMA" in "readADC" which resets the ADC error condition).
//               2. Don't stop DMA here, (by calling "HAL_ADC_Stop_DMA") or else over-runs will occur
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
 UBaseType_t uxSavedInterruptStatus;

 uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
  if (hadc == handleADC1) flagADCConversionCompleted = 1;
 taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

///////////////////////////////////////////////////////////////////////////////
//
// HAL_ADC_ErrorCallback
//
//  DESCRIPTION: Triggered by MCU's internal 12-bit ADC OverRun error
//
//  NOTES:       1. Calls "HAL_ADC_Start_DMA" which resets the ADC error condition
//               2. Must stop DMA here, (by calling "HAL_ADC_Stop_DMA") before restarting the ADC-DMA
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
 HAL_ADC_Stop_DMA(hadc);
 HAL_ADC_Start_DMA(handleADC1, (uint32_t*)ADCValues, 2);
}


///////////////////////////////////////////////////////////////////////////////
//
// HAL_GPIO_EXTI_Callback
//
//  DESCRIPTION: Triggered by MCU's GPIO Interrupt
//
//  NOTES:       1. Handle the RTD conversion complete interrupt
//
//  AUTHOR: Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 UBaseType_t uxSavedInterruptStatus;

 uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
 // if (GPIO_Pin == _RTD_INT.pin) flagRTDConversionCompleted = 1;
 taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


///////////////////////////////////////////////////////////////////////////////
//
// initIO
//
//  DESCRIPTION: Assigns and Initialises all of the IO Ports
//
//  NOTES:       1. Called once from "main" function before threads commence
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint8_t initIO()
{
 // Assign GPIO Ports and Pins

 // Configure unused GPIO pins: PB3 PB4 PB5  (These were originally expected to be used for ADC inputs but couldn't be assigned in the MCU)
 GPIO_InitTypeDef GPIO_InitStruct;

 GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 // Configure GPIO Ports and Pins for Push Button Inputs  (Note: /BUT1 swapped with /BUT3 to match PB connector labels on PCB)
 // Silkscreen => Function Mappings: PB4 => Power, PB1 => PB3, PB2 => PB2, PB3 => PB1
 for (uint8_t i=0; i<sizeof(buttonInputs); i++)
 {
  switch(i)
  {
   case 0:  // /BUT1
    buttonInputs[0].port = GPIOB;
    buttonInputs[0].pin = GPIO_PIN_3;
   break;
   case 1:  // /BUT2
    buttonInputs[1].port = GPIOB;
    buttonInputs[1].pin = GPIO_PIN_4;  // GPIO_PIN_13
   break;
   case 2:  // /BUT3
    buttonInputs[2].port = GPIOB;
    buttonInputs[2].pin = GPIO_PIN_5;
   break;
  }
 }

 // Initialise Global Variables
 flagADCConversionCompleted = 1;

 displaySuppV = 0.0;

 pushButtons = 0;

 //__HAL_TIM_GET_COUNTER(handleTIM1) = 0;
 //HAL_TIM_Base_Start(handleTIM1);         // Start the Flow Counters

 return 0;
}


///////////////////////////////////////////////////////////////////////////////
//
// readButtonInputs
//
//  DESCRIPTION: Read the state of all the Button Inputs
//
//  NOTES:       1. This routine is called by the "readIOThread" to set the value of the "pushButtons" variable
//               2. /BUT1-3    Inputs (with Pullups)  Read Button state, (active low)
//               3. Pin is LOW when Button Pressed
//               4. The routine uses the "delayedResponse" function template to debounce each button by "DEBOUNCE_DELAY" milliseconds
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint8_t readButtonInputs(uint8_t* debounceFlag, uint32_t* debounceStart, uint32_t* debounceEnd)
{
 GPIO_PinState pinState;

 for (uint8_t i=0; i<3; i++)
 {
  uint8_t condition, bit = (uint8_t)pow(2, i);
  pinState = HAL_GPIO_ReadPin(buttonInputs[i].port, buttonInputs[i].pin);

  taskENTER_CRITICAL();
   condition = (((pinState == GPIO_PIN_RESET) && ((pushButtons & bit) == 0)) || ((pinState == GPIO_PIN_SET) && ((pushButtons & bit) == bit))) ? 1 : 0;
  taskEXIT_CRITICAL();

  if (delayedResponse(condition, &(debounceFlag[i]), NULL, &(debounceStart[i]), &(debounceEnd[i]), NULL, DEBOUNCE_DELAY, 0, NULL))
  {
   taskENTER_CRITICAL();
    if ((pinState == GPIO_PIN_RESET)) pushButtons |= bit;
    else pushButtons &= ~bit;
   taskEXIT_CRITICAL();
  }
 }

 return 0;
}

///////////////////////////////////////////////////////////////////////////////
//
// readADC
//
//  DESCRIPTION: Reads the Supply voltages monitored by the internal 12-bit ADC
//
//  NOTES:       1. This routine is called by the "readIOThread".
//               2. Must call "HAL_ADC_Stop_DMA" followed by "HAL_ADC_Start_DMA" here instead of in the interrupt handler to prevent the DMA interrupts "starving" the rest of the tasks from CPU time
//               3. Raw ADC data is stored in "ADCValues" vector, (transferred there by the DMA)
//               4. "ADCValues" vector must also be declared as "uint16_t" data type... No idea why? (I would have reasonably expected it to be "uint32_t")
//               5. ADC channels are allocated as follows:
//                PB0   ADC_3V3
//                PB1   ADC_5V
//                PB2   ADC_12V
//                PB3   ADC_24V
//                PB4   ADC_VIN
//                PB5   ADC_VBAT
//                PB6   ADC_ICHRG
//               6. PB2 - 6 cannot be allocated as analog inputs and are not used
//                PB0 - 1 ADC1-IN8-9
//               7. Recursive Moving Average algorithm: MA*[i]= MA*[i-1] +X[i] - MA*[i-1]/N, where MA* is the moving average*N.
//                                                      MA[i]= MA*[i]/N
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint8_t readADC()
{
 taskENTER_CRITICAL();
  if (flagADCConversionCompleted)
  {
   float displaySuppVSample = ((float)(ADCValues[0] & 0x0FFF)) / 106.8727; // (2^12-1) / VREF / (R1+R2)/R2 = 4095.0 / 2.5 / (1000 + 69.8) * 69.8
   float upsBattVSample = ((float)(ADCValues[1] & 0x0FFF)) / 106.8727;   // (2^12-1) / VREF / (R1+R2)/R2 = 4095.0 / 2.5 / (1000 + 69.8) * 69.8

   displaySuppVSum = MOVING_AVERAGE(displaySuppVSum, displaySuppVSample, VOLTAGE_FILTER_LENGTH);  // Calculate recursive moving average values

   flagADCConversionCompleted = 0;

   HAL_ADC_Stop_DMA(handleADC1);
   HAL_ADC_Start_DMA(handleADC1, (uint32_t*)ADCValues, 2);  // Triggers a DMA Read of the MCU's internal 12-bit ADC channels
  }
 taskEXIT_CRITICAL();

 return 0;
}

