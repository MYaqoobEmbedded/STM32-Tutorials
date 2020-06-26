/*
Library:					STM32F4 Audio Codec - CS43L22
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			29/01/2016
Last modified:			29/12/2018
Description:			This is an STM32 device driver library for the CS43L22 Audio Codec, using STM HAL libraries

References:
			1) Cirrus Logic CS43L22 datasheet
				 https://www.mouser.com/ds/2/76/CS43L22_F2-1142121.pdf
			2) ST opensource CS43L22 Audio Codec dsp drivers.
										
* Copyright (C) 2018 - M. Yaqoob
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public License version 3 as published by the Free Software Foundation.
	
   This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

#include "MY_CS43L22.h"

static I2C_HandleTypeDef i2cx;
extern I2S_HandleTypeDef hi2s3;

//(1): Functions definitions
//------------ Static functions ------------//
static void write_register(uint8_t reg, uint8_t *data);
static void read_register(uint8_t reg, uint8_t *data);

// Function(1): Write to register
static void write_register(uint8_t reg, uint8_t *data)
{
  uint8_t iData[2];
	iData[0] = reg;
	iData[1] = data[0];
	HAL_I2C_Master_Transmit(&i2cx, DAC_I2C_ADDR, iData, 2, 100);
	//HAL_I2C_Master_Transmit(&i2cx, DAC_I2C_ADDR, data, size, 100);
}
// Function(2): Read from register
static void read_register(uint8_t reg, uint8_t *data)
{
  uint8_t iData[2];
	iData[0] = reg;
	HAL_I2C_Master_Transmit(&i2cx, DAC_I2C_ADDR, iData, 1, 100);
	HAL_I2C_Master_Receive(&i2cx, DAC_I2C_ADDR, data, 1, 100);
}

//-------------- Public Functions ----------------//
// Function(1): Initialisation
void CS43_Init(I2C_HandleTypeDef i2c_handle, CS43_MODE outputMode)
{
  uint8_t iData[2];
	__HAL_UNLOCK(&hi2s3);     // THIS IS EXTREMELY IMPORTANT FOR I2S3 TO WORK!!
	__HAL_I2S_ENABLE(&hi2s3); // THIS IS EXTREMELY IMPORTANT FOR I2S3 TO WORK!!
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	//(1): Get the I2C handle
	i2cx = i2c_handle;
	//(2): Power down
	iData[1] = 0x01;
	write_register(POWER_CONTROL1,iData);
	//(3): Enable Right and Left headphones
	iData[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
	iData[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
	iData[1] |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	iData[1] |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	write_register(POWER_CONTROL2,&iData[1]);
	//(4): Automatic clock detection
	iData[1] = (1 << 7);
	write_register(CLOCKING_CONTROL,&iData[1]);
	//(5): Interface control 1
	read_register(INTERFACE_CONTROL1, iData);
	iData[1] &= (1 << 5); // Clear all bits except bit 5 which is reserved
	iData[1] &= ~(1 << 7);  // Slave
	iData[1] &= ~(1 << 6);  // Clock polarity: Not inverted
	iData[1] &= ~(1 << 4);  // No DSP mode
	iData[1] &= ~(1 << 2);  // Left justified, up to 24 bit (default)
	iData[1] |= (1 << 2);
	
	iData[1] |=  (3 << 0);  // 16-bit audio word length for I2S interface
	write_register(INTERFACE_CONTROL1,&iData[1]);
	//(6): Passthrough A settings
	read_register(PASSTHROUGH_A, &iData[1]);
	iData[1] &= 0xF0;      // Bits [4-7] are reserved
	iData[1] |=  (1 << 0); // Use AIN1A as source for passthrough
	write_register(PASSTHROUGH_A,&iData[1]);
	//(7): Passthrough B settings
	read_register(PASSTHROUGH_B, &iData[1]);
	iData[1] &= 0xF0;      // Bits [4-7] are reserved
	iData[1] |=  (1 << 0); // Use AIN1B as source for passthrough
	write_register(PASSTHROUGH_B,&iData[1]);
	//(8): Miscellaneous register settings
	read_register(MISCELLANEOUS_CONTRLS, &iData[1]);
	if(outputMode == MODE_ANALOG)
	{
		iData[1] |=  (1 << 7);   // Enable passthrough for AIN-A
		iData[1] |=  (1 << 6);   // Enable passthrough for AIN-B
		iData[1] &= ~(1 << 5);   // Unmute passthrough on AIN-A
		iData[1] &= ~(1 << 4);   // Unmute passthrough on AIN-B
		iData[1] &= ~(1 << 3);   // Changed settings take affect immediately
	}
	else if(outputMode == MODE_I2S)
	{
		iData[1] = 0x02;
	}
	write_register(MISCELLANEOUS_CONTRLS,&iData[1]);
	//(9): Unmute headphone and speaker
	read_register(PLAYBACK_CONTROL, &iData[1]);
	iData[1] = 0x00;
	write_register(PLAYBACK_CONTROL,&iData[1]);
	//(10): Set volume to default (0dB)
	iData[1] = 0;
	write_register(PASSTHROUGH_VOLUME_A,&iData[1]);
	write_register(PASSTHROUGH_VOLUME_B,&iData[1]);
	write_register(PCM_VOLUME_A,&iData[1]);
	write_register(PCM_VOLUME_B,&iData[1]);
}

// Function(2): Enable Right and Left headphones
void CS43_Enable_RightLeft(uint8_t side)
{
  uint8_t iData[2];
	switch (side)
	{
		case 0:
			iData[1] =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		case 1:
			iData[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		case 2:
			iData[1] =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		case 3:
			iData[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		default:
			break;
	}
	iData[1] |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	iData[1] |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	write_register(POWER_CONTROL2,&iData[1]);
}

// Function(3): Set Volume Level
void CS43_SetVolume(uint8_t volume)
{
  uint8_t iData[2];
  /* Set the Master volume */
  iData[1] = VOLUME_MASTER(volume);
  write_register(CS43L22_REG_MASTER_A_VOL,&iData[1]);
  write_register(CS43L22_REG_MASTER_B_VOL,&iData[1]);
}

void CS43_SetMute(bool mute)
{
  uint8_t iData[2];
  if(mute)
  {
    iData[1] = 0xFF;
    write_register(POWER_CONTROL2,&iData[1]);
    iData[1] = 0x01;
    write_register(CS43L22_REG_HEADPHONE_A_VOL,&iData[1]);
    write_register(CS43L22_REG_HEADPHONE_B_VOL,&iData[1]);
  }
  else
  {
    iData[1] = 0x00;
    write_register(CS43L22_REG_HEADPHONE_A_VOL,&iData[1]);
    write_register(CS43L22_REG_HEADPHONE_B_VOL,&iData[1]);
    iData[1] = 0xAF;
    write_register(POWER_CONTROL2,&iData[1]);
  }
}

// Function(4): Start the Audio DAC
void CS43_Start(void)
{
  uint8_t iData[2];
  CS43_SetMute(0);
	// Write 0x99 to register 0x00.
	iData[1] = 0x99;
	write_register(CONFIG_00,&iData[1]);
	// Write 0x80 to register 0x47.
	iData[1] = 0x80;
	write_register(CONFIG_47,&iData[1]);
	// Write '1'b to bit 7 in register 0x32.
	read_register(CONFIG_32, &iData[1]);
	iData[1] |= 0x80;
	write_register(CONFIG_32,&iData[1]);
	// Write '0'b to bit 7 in register 0x32.
	read_register(CONFIG_32, &iData[1]);
	iData[1] &= ~(0x80);
	write_register(CONFIG_32,&iData[1]);
	// Write 0x00 to register 0x00.
	iData[1] = 0x00;
	write_register(CONFIG_00,&iData[1]);
	//Set the "Power Ctl 1" register (0x02) to 0x9E
	iData[1] = 0x9E;
	write_register(POWER_CONTROL1,&iData[1]);
}

void CS43_Stop(void)
{
  uint8_t iData[2];
  CS43_SetMute(1);
  iData[1] = 0x04;
  write_register(MISCELLANEOUS_CONTRLS,&iData[1]);
	iData[1] = 0x9F;
	write_register(POWER_CONTROL1,&iData[1]);
}
