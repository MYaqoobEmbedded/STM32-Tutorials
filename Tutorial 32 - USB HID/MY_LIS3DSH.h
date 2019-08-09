/*
Library:					Accelerometer - LIS3DSH
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			12/12/2018
Last modified:		-/-
Description:			This is an STM32 device driver library for the LIS3DSH Accelerometer, using STM HAL libraries

References:
			1) STMicroelectronics LIS3DSH datasheet
				 https://www.st.com/resource/en/datasheet/lis3dsh.pdf
			2) ST opensource LIS3DSH accelerometer dsp drivers.
										
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

//List of Defines
//1. LIS3DHS registers addresses
#define LIS3DSH_WHO_AM_I_ADDR                0x0F
#define LIS3DSH_OFF_X_ADDR                   0x10
#define LIS3DSH_OFF_Y_ADDR                   0x11
#define LIS3DSH_OFF_Z_ADDR                   0x12

#define LIS3DSH_CTRL_REG4_ADDR               0x20
#define LIS3DSH_CTRL_REG1_ADDR               0x21
#define LIS3DSH_CTRL_REG2_ADDR               0x22
#define LIS3DSH_CTRL_REG3_ADDR               0x23              
#define LIS3DSH_CTRL_REG5_ADDR               0x24
#define LIS3DSH_CTRL_REG6_ADDR               0x25

#define LIS3DSH_STATUS_ADDR                  0x27

#define LIS3DSH_OUT_X_L_ADDR                 0x28
#define LIS3DSH_OUT_X_H_ADDR                 0x29
#define LIS3DSH_OUT_Y_L_ADDR                 0x2A
#define LIS3DSH_OUT_Y_H_ADDR                 0x2B
#define LIS3DSH_OUT_Z_L_ADDR                 0x2C
#define LIS3DSH_OUT_Z_H_ADDR                 0x2D


//2. Data rate
#define LIS3DSH_DATARATE_POWERDOWN           ((uint8_t)0x00)  /* Power Down Mode*/
#define LIS3DSH_DATARATE_3_125               ((uint8_t)0x10)  /* 3.125 Hz Normal Mode */
#define LIS3DSH_DATARATE_6_25                ((uint8_t)0x20)  /* 6.25  Hz Normal Mode */
#define LIS3DSH_DATARATE_12_5                ((uint8_t)0x30)  /* 12.5  Hz Normal Mode */
#define LIS3DSH_DATARATE_25                  ((uint8_t)0x40)  /* 25    Hz Normal Mode */
#define LIS3DSH_DATARATE_50                  ((uint8_t)0x50)  /* 50    Hz Normal Mode */
#define LIS3DSH_DATARATE_100                 ((uint8_t)0x60)  /* 100   Hz Normal Mode */
#define LIS3DSH_DATARATE_400                 ((uint8_t)0x70)  /* 400   Hz Normal Mode */
#define LIS3DSH_DATARATE_800                 ((uint8_t)0x80)  /* 800   Hz Normal Mode */
#define LIS3DSH_DATARATE_1600                ((uint8_t)0x90)  /* 1600  Hz Normal Mode */

//3. Full scale
#define LIS3DSH_FULLSCALE_2                  ((uint8_t)0x00)  /* 2 g  */
#define LIS3DSH_FULLSCALE_4                  ((uint8_t)0x08)  /* 4 g  */
#define LIS3DSH_FULLSCALE_6                  ((uint8_t)0x10)  /* 6 g  */
#define LIS3DSH_FULLSCALE_8                  ((uint8_t)0x18)  /* 8 g  */
#define LIS3DSH_FULLSCALE_16                 ((uint8_t)0x20)  /* 16 g */

//4. Anti-Aliasing Bandwidth
#define LIS3DSH_FILTER_BW_800                ((uint8_t)0x00)  /* 800 Hz */
#define LIS3DSH_FILTER_BW_400                ((uint8_t)0x80)  /* 400 Hz  */
#define LIS3DSH_FILTER_BW_200                ((uint8_t)0x40)  /* 200 Hz */
#define LIS3DSH_FILTER_BW_50                 ((uint8_t)0xC0)  /* 50 Hz  */

//5. Enabled axis 
#define LIS3DSH_X_ENABLE                     ((uint8_t)0x01)
#define LIS3DSH_Y_ENABLE                     ((uint8_t)0x02)
#define LIS3DSH_Z_ENABLE                     ((uint8_t)0x04)
#define LIS3DSH_XYZ_ENABLE                   ((uint8_t)0x07)

//6. Sensitivity values (Based on Full-Scale)
#define LIS3DSH_SENSITIVITY_0_06G            0.06  /* 0.06 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_12G            0.12  /* 0.12 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_18G            0.18  /* 0.18 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_24G            0.24  /* 0.24 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_73G            0.73  /* 0.73 mg/digit*/

//Typedefs
//1. Accelerometer Configuration struct
typedef struct 
{
	uint8_t dataRate;
	uint8_t fullScale;
	uint8_t antiAliasingBW;
	uint8_t enableAxes;
	bool interruptEnable;
}LIS3DSH_InitTypeDef;

//2. Accelerometer raw data
typedef struct 
{
	int16_t x;
	int16_t y;
	int16_t z;
}LIS3DSH_DataRaw;

//3. Accelerometer mg data (scaled data)
typedef struct
{
	float x;
	float y;
	float z;
}LIS3DSH_DataScaled;

//Functions prototypes
//Private functions
//1. Write IO
void LIS3DSH_WriteIO(uint8_t reg, uint8_t *dataW, uint8_t size);
//2. Read IO
void LIS3DSH_ReadIO(uint8_t reg, uint8_t *dataR, uint8_t size);


//1. Accelerometer initialise function
void LIS3DSH_Init(SPI_HandleTypeDef *accSPI, LIS3DSH_InitTypeDef *accInitDef);
//2. Get Accelerometer raw data
LIS3DSH_DataRaw LIS3DSH_GetDataRaw(void);
//3. Get Accelerometer mg data
LIS3DSH_DataScaled LIS3DSH_GetDataScaled(void);
//4. Poll for Data Ready
bool LIS3DSH_PollDRDY(uint32_t msTimeout);

//** Calibration functions **//
//1. Set X-Axis calibrate
void LIS3DSH_X_calibrate(float x_min, float x_max);
//2. Set Y-Axis calibrate
void LIS3DSH_Y_calibrate(float y_min, float y_max);
//3. Set Z-Axis calibrate
void LIS3DSH_Z_calibrate(float z_min, float z_max);

