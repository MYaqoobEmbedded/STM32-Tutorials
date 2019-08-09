/*
Library:					Resistive touch screen controller SPI - TSC2046
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			09/12/2018
Last modified:		-/-
Description:			This is an STM32 device driver library for the TSC2046 resistive touch controller, using STM HAL libraries
										
* Copyright (C) 2018 - M. Yaqoob
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

//Header files
//** CHANGE BASED ON STM32 CHIP F4/F7/F1...**//
#include "stm32f4xx_hal.h"

#include "MY_ILI9341.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

//List of defines
//1. X-Axis measurement
#define CMD_X_AXIS		0x50
#define CMD_Y_AXIS		0x10
#define CMD_Z_AXIS		0x30


//Typedefs
//1. Touch coordinates
typedef struct
{
	uint16_t x_touch;
	uint16_t y_touch;
}TS_TOUCH_RAW_Def;

//2. Calibration typedef
typedef struct
{
	uint16_t TL_X;
	uint16_t TL_Y;
	
	uint16_t BR_X;
	uint16_t BR_Y;
	
	float Scale_X;
	float Scale_Y;
	
	float Bias_X;
	float Bias_Y;
	
	uint16_t Width;
	uint16_t Height;
}TS_CALIBRATE_Def;

//3. Touch Screen Data
typedef struct
{
	bool isPressed;
	uint16_t X;
	uint16_t Y;
}TS_TOUCH_DATA_Def;

//Functions prototypes
//1. Send TSC2046 Command and wait for a response
uint16_t TSC2046_SendCommand(uint8_t cmd);
//2. Calibrate resistive touch panel
void TSC2046_Calibrate(void);
//3. Poll for touch status
TS_TOUCH_RAW_Def TSC2046_GetRawTouch(void);

//4. Begin function
bool  TSC2046_Begin(SPI_HandleTypeDef *touchSPI, GPIO_TypeDef *csPort, uint16_t csPin);

//5. Get raw touch data
//i. get x-axis raw touch 12-bit value
uint16_t TSC2046_getRaw_X(void);
//ii. get y-axis raw touch 12-bit value
uint16_t TSC2046_getRaw_Y(void);
//iii. get z-axis raw touch 12-bit value
uint16_t TSC2046_getRaw_Z(void);

//6. Print calibration points (to LCD driver)
//i. Top-Left corner point
void TSC2046_TL_point(void);
//ii. Bottom-Right corner point
void TSC2046_BR_point(void);

//7. Get orientation (from LCD driver)
uint8_t TSC2046_getOrientation(void);

//8. Get touch sccreen data
TS_TOUCH_DATA_Def TSC2046_GetTouchData(void);

