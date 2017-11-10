///////////////////////////////////////////////////////////////////////////////
//
// PARSE.C
//
//  DESCRIPTION: Command Parsing Routines
//
//  CREATED:     14 OCT 2016
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "global.h"
#include "comms.h"
#include "io.h"
#include "parse.h"

//extern uint32_t FLOW_COUNT;
//extern uint32_t FLOW_COUNT_CRC;

///////////////////////////////////////////////////////////////////////////////
//
// itoa16
//
//  DESCRIPTION: Converts unsigned integer to ascii - Base 16
//
//  NOTES:       1. Ascii digits are stored at (buff[index] ... buff[index+length-1])
//
//  AUTHOR:      Peter Bartlett <peter@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////


void itoa16(uint32_t val, char* buff, const int buffsize) //lint !e970 need char and int types here. std function call reqd.
{
    int32_t i = buffsize - 1;
    int32_t j;

    if (i >= 0)
    {
        buff[i] = '\0';
        --i;

        // Pick off digits
        if (1)
        {
            for (/* */; /* val && */ (i >= 0); --i)
            {
                buff[i] = "0123456789ABCDEF"[val % 16];
                val /= 16;
            }
        }

        // Shift to start of buffer
        for (j = 0, i++; (i > j) && (i < buffsize); i++, j++)
        {
            buff[j] = buff[i];
        }
    }
}


///////////////////////////////////////////////////////////////////////////////
//
// digitsToInt
//
//  DESCRIPTION: Converts zero-padded ascii digits to an unsigned integer
//
//  NOTES:       1. Ascii digits are stored at (command[index] ... command[index+length-1])
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint32_t digitsToInt(char* command, uint32_t index, uint8_t length, uint8_t base)
{
 uint32_t result = 0;

 for (uint8_t i=0; i<length; i++)
 {
  uint32_t digit = command[index+i];
  uint32_t power = (uint32_t)(pow(base, length-i-1));

  if ((base == 10 || base == 16) && digit >= '0' && digit <= '9') result += (digit - '0') * power;
  else if (base == 16 && digit >= 'a' && digit <= 'f') result += (digit - 'a' + 10) * power;
  else if (base == 16 && digit >= 'A' && digit <= 'F') result += (digit - 'A' + 10) * power;
  else return 0;
 }

 return result;
}

///////////////////////////////////////////////////////////////////////////////
//
// parseCommand
//
//  DESCRIPTION: Parses the TIB data packet stored in the "command" buffer
//
//  NOTES:       1. Implements this via a
//
//  AUTHOR:      Keith Willis <keith@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

uint8_t parseCommand(char* command)
{
 int lenCommand = strlen(command);
 uint8_t crcIn = digitsToInt(command, lenCommand-2, 2, 16);
 uint8_t crcOut = crc_calcCrc8(command, lenCommand-2);
 char response[RESPONSE_BUFFER_LENGTH];
 volatile int xx, yy, zz;

 if (crcOut != crcIn)
 {
  sprintf(response, "<,FE,\"Invalid CRC: %02X, Valid CRC = %02X for Command [%s]\",", crcIn, crcOut, command);
  HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
  sendResponse(response);
  if (!DEBUG) return 1;  // Invalid CRC (ignored in DEBUG mode)
 }

 if (command[1] != ',') return -1;

 if (DEBUG) // Debug Command Echo
 {
  sprintf(response, "%s\n", command);  // Add <LF> to the message string
  //writeMessage(response);
 }

 uint32_t delay;

 switch(command[2])  // First digit in command packet
 {
  case '1':  // LEDs
   switch(command[3])  // Second digit in command packet
   {
   case '1':	// msgLedCmd	Syntax: ">,11,[ledVal:6],[],[],[CRC8]<LF>
	   if (!DEBUG) writeMessage("msgAllLedCmd\r\n");
	   if (command[10] != ',') return 1;
	   xx = digitsToInt(command, 5, 6, 16);
	   break;
   case '2':	// msgSingleLedCmd	Syntax: ">,12,[led:2],[pwm:2],[iref:2],[CRC8]<LF>"	Example: ">,12,01,FF,FF,[CRC8]<LF>"

	   if (!DEBUG) writeMessage("msgSingleLedCmd\r\n");
	   if (command[4] != ',' || command[7] != ',' || command[10] != ',' || command[13] != ',') return 1;
	   uint8_t port = digitsToInt(command, 5, 2, 10);
	   pwm(port, (float)digitsToInt(command, 8, 2, 16));
	   current(port, (float)digitsToInt(command, 11, 2, 16));
	   break;
   case '3':	// msgLedBlink		Syntax: ">,13,[CRC8]<LF>"	Syntax: ">,[onOff:1],[period:2],[duty:2],[CRC8]<LF>"	Example: ">,13,1,FF,80,[CRC8]<LF>" (period=16.8s, duty=50%)
	   if (!DEBUG) writeMessage("msgBlinkLedCmd\r\n");
	   if (command[4] != ',' || command[6] != ',') return 1;

	   blink(digitsToInt(command, 5, 1, 10), digitsToInt(command, 7, 2, 16), digitsToInt(command, 10, 2, 16));
	   break;
   case '4':		// msgLedAllOff		Syntax: ">,14,[CRC8]<LF>"
	   if (!DEBUG) writeMessage("msgLedAllOffCmd\r\n");
	   if (command[4] != ',') return 1;
	   alloff();
	   break;
   case '5':
	   break;

    /*case '1':  // msgSolAllOffCmd  Syntax: ">,11,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgSolAllOffCmd\r\n");
     if (command[4] != ',') return 1;
     taskENTER_CRITICAL();
      for (uint8_t i=0; i<24; i++) writeSolenoid(i, SOLENOID_OFF, 0);  // Turn OFF all solenoids
     taskEXIT_CRITICAL();
     sendResponse("<,12,");
    break;
    case '3':  // msgSolWriteCmd  Syntax: ">,13,[solenoid:2],[onOff:1],[duration:3],[CRC8]<LF>"  Example: ">,13,09,1,095,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgSolWriteCmd\r\n");
     if (command[4] != ',' || command[7] != ',' || command[9] != ',' || command[13] != ',') return 1;

     solenoid = digitsToInt(command, 5, 2, 10);   // zero-padded two digit ascii-coded decimal number
     onOff = digitsToInt(command, 8, 1, 10);      // one digit ascii-coded decimal number
     duration = digitsToInt(command, 10, 3, 10);  // zero-padded three digit ascii-coded decimal number

     if (solenoid > 23 || onOff > 1) return -1;
     if (onOff == 0 || duration <= 50) duration = 0;
     if (duration > 250) duration = 250;

     // Do Solenoid Write Command
     taskENTER_CRITICAL();
      if (!NUCLEO) writeSolenoid(solenoid, onOff, duration);  // Turn ON selected solenoid
     taskEXIT_CRITICAL();

     sprintf(response, "<,15,%02d,%d,%03d,", solenoid, onOff, duration);
     sendResponse(response);
    break;
    case '4':  // msgSolStatCmd  Syntax: ">,14,[solenoid],[CRC8]<LF>"  Example: ">,14,09,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgSolStatCmd\r\n");
     if (command[4] != ',' || command[7] != ',') return 1;

     solenoid = digitsToInt(command, 5, 2, 10);   // zero-padded two digit ascii-coded decimal number
     if (solenoid > 23) return -1;

     // Do Solenoid Status Command  (Currently returning the Solenoid Driver's "status" output)
     taskENTER_CRITICAL();
//      onOff = Solenoids[solenoid].outputState;
      status = readSolenoidStatus(solenoid);
      duration = Solenoids[solenoid].duration;
     taskEXIT_CRITICAL();

//     sprintf(response, "<,15,%02d,%d,%03d,", solenoid, onOff, duration);
     sprintf(response, "<,15,%02d,%d,%03d,", solenoid, status, duration);
     sendResponse(response);
    break;*/
   }
  break;

  case '2':  // Digital Inputs
   switch(command[3])  // Second digit in command packet
   {
    case '1':  // msgDigInReadAllCmd  Syntax: >,21,[CRC8]<LF>
     if (DEBUG) writeMessage("msgDigInReadAllCmd\r\n");
     if (command[4] != ',' ) return 1;


    break;
   }
  break;

  case '3':  // Analogue Inputs
   switch(command[3])  // Second digit in command packet
   {
    case '1':  // msgAnaInReadAllCmd  Syntax: ">,31,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgAnaInReadAllCmd\r\n");
     if (command[4] != ',' ) return 1;

    break;
   }
  break;

  case '4':  // RTD Inputs
   switch(command[3])  // Second digit in command packet
   {
    case '1':  // msgRtdReadAllCmd  Syntax: ">,41,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgRtdReadAllCmd\r\n");
     if (command[4] != ',' ) return 1;

//     sprintf(response, "<,42,%+03d.%02d,%+03d.%02d,%+03d.%02d,%+03d.%02d,", SPLIT_FLOAT_100(RTDValues[0]), SPLIT_FLOAT_100(RTDValues[1]), SPLIT_FLOAT_100(RTDValues[2]), SPLIT_FLOAT_100(RTDValues[3]));
//     sprintf(response, "<,42,%+03d.%02d,%+03d.%02d,%+03d.%02d,%+03d.%02d,", SPLIT_FLOAT(RTDValues[0], 2), SPLIT_FLOAT(RTDValues[1], 2), SPLIT_FLOAT(RTDValues[2], 2), SPLIT_FLOAT(RTDValues[3], 2));

     sendResponse(response);
    break;
   }
  break;

  case '5':  // Flow Counter
   switch(command[3])  // Second digit in command packet
   {
    case '1':  // msgCntReadCmd  Syntax: ">,51,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgCntReadCmd\r\n");
     if (command[4] != ',' ) return 1;
//     sprintf(response, "<,52,%08X,", (unsigned)FLOW_COUNT);
     sendResponse(response);
    break;
    case '3':  // msgCntWriteCmd  Syntax: ">,53,[counterVal:8],[CRC8]<LF>"  Example: ">,53,0123ABCD,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgCntWriteCmd\r\n");
//     writeFlowCount(digitsToInt(command, 5, 8, 16));
//     sprintf(response, "<,54,%08X,", (unsigned)FLOW_COUNT);
     sendResponse(response);
    break;
    case '5':  // msgCntCrcReadCmd  Syntax: ">,55,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgCntCrcReadCmd\r\n");
     if (command[4] != ',' ) return 1;
//     sprintf(response, "<,56,%08X,", (unsigned)FLOW_COUNT_CRC);
     sendResponse(response);
    break;
   }
  break;

  case '6':  // Push Buttons
   switch(command[3])  // Second digit in command packet
   {
    case '1':  // msgButReadAllCmd  Syntax: ">,61,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgButReadAllCmd\r\n");
     if (command[4] != ',') return 1;
     sprintf(response, "<,62,%02X,", (unsigned)pushButtons);
     sendResponse(response);
    break;
    /*case '3':  // msgButWriteAllCmd  Syntax: ">,63,[leds:2],[CRC8]<LF>"  Example: ">,63,0A,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgButWriteAllCmd\r\n");
     if (command[4] != ',' || command[7] != ',') return 1;
     writeButtonLEDs(digitsToInt(command, 5, 2, 16));  // zero-padded two digit ascii-coded hexadecimal number
     sendResponse("<,64,");
    break;*/
   }
  break;

  case '7':  // Power Supply
   switch(command[3])  // Second digit in command packet
   {
    case '1':  // msgPwrStatReadCmd  Syntax: ">,71,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgPwrStatReadCmd\r\n");
     if (command[4] != ',' ) return 1;

     taskENTER_CRITICAL();
      sprintf(response, "<,72,%02d.%02d,", SPLIT_FLOAT(displaySuppVSum/(float)VOLTAGE_FILTER_LENGTH, 2));
     taskEXIT_CRITICAL();

     sendResponse(response);
    break;
    case '3':  // msgPwrShdnCmd  Syntax: ">,73,[delay:3],[CRC8]<LF>"  Example: ">,73,020,[CRC8]<LF>"
     if (DEBUG) writeMessage("msgPwrShdnCmd\r\n");
     if (command[4] != ',' || command[8] != ',') return 1;

     delay = digitsToInt(command, 5, 3, 10);   // zero-padded three digit ascii-coded decimal number
     if (delay > 300) delay = 300;

      sendResponse("<,74,");
    break;
   }
  break;

  case '8':  // System Commands
   switch(command[3])  // Second digit in command packet
   {
    case '1':  // msgVerReadCmd  Syntax: ">,81,[CRC8]<LF>"
     //if (DEBUG) writeMessage("msgVerReadCmd\r\n");
     if (command[4] != ',' ) return 1;
     sprintf(response, "<,83,%s-%s.%s,", HARDWARE_ID, FIRMWARE_VERSION, SVN_RELEASE);
     sendResponse(response);
    break;
    case '2':  // msgUidReadCmd  Syntax: ">,82,[CRC8]<LF>"
     //if (DEBUG) writeMessage("msgUidReadCmd\r\n");
     if (command[4] != ',' ) return 1;
     sprintf(response, "<,84,%08X-%08X-%08X,", (unsigned)(HAL_GetUIDw2()), (unsigned)(HAL_GetUIDw1()), (unsigned)HAL_GetUIDw0());
     sendResponse(response);
    break;
   }
  break;
 }

 return 0;
}
