/*
Library:					DHT22 - AM2302 Temperature and Humidity Sensor
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			21/11/2018
Last modified:		-/-
Description:			This is an STM32 device driver library for the DHT22 Temperature and Humidity Sensor, using STM HAL libraries

References:				This library was written based on the DHT22 datasheet
										- https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf
										
* Copyright (C) 2018 - M. Yaqoob
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

//Header files
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

//Pin Mode enum
typedef enum
{
	ONE_OUTPUT = 0,
	ONE_INPUT,
}OnePinMode_Typedef;



//*** Functions prototypes ***//
//OneWire Initialise
void DHT22_Init(GPIO_TypeDef* DataPort, uint16_t DataPin);
//Change pin mode
static void ONE_WIRE_PinMode(OnePinMode_Typedef mode);
//One Wire pin HIGH/LOW Write
static void ONE_WIRE_Pin_Write(bool state);
static bool ONE_WIRE_Pin_Read(void);
//Microsecond delay
static void DelayMicroSeconds(uint32_t uSec);
//Begin function
static void DHT22_StartAcquisition(void);
//Read 5 bytes
static void DHT22_ReadRaw(uint8_t *data);

//Get Temperature and Humidity data
bool DHT22_GetTemp_Humidity(float *Temp, float *Humidity);

