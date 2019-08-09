/*
Library:						4x4 Keypad drive for STM32 MCUs
Written by:					Mohamed Yaqoob
Date written:				03/04/2018
Description:				The MY_Keypad4x4 library consists of the following public functions
										Function(1)- Keypad4x4_Init
										Function(2)- Keypad4x4_ReadKeypad
										Function(3)- Keypad4x4_GetChar
*/

//Header files
#include "stm32f4xx_hal.h"
#include <stdbool.h>

//***** Constant variables and typedefs *****//
typedef struct
{
	GPIO_TypeDef* IN0_Port;
	GPIO_TypeDef* IN1_Port;
	GPIO_TypeDef* IN2_Port;
	GPIO_TypeDef* IN3_Port;	
	GPIO_TypeDef* OUT0_Port;	
	GPIO_TypeDef* OUT1_Port;
	GPIO_TypeDef* OUT2_Port;
	GPIO_TypeDef* OUT3_Port;
	
	uint16_t	IN0pin;
	uint16_t	IN1pin;
	uint16_t	IN2pin;
	uint16_t	IN3pin;
	uint16_t	OUT0pin;
	uint16_t	OUT1pin;
	uint16_t	OUT2pin;
	uint16_t	OUT3pin;
}Keypad_WiresTypeDef;

//List of keys as chars


//***** Functions prototype *****//
//Function(1): Set Keypad pins and ports
void Keypad4x4_Init(Keypad_WiresTypeDef  *KeypadWiringStruct);
//Function(2): Get pin positions for colomn change use, only for out pins
static void Keypad4x4_FindPins_positions(void);
//Function(3): Change colomn number
static void Keypad4x4_ChangeColomn(uint8_t colNum_0_to_3);
//Function(4): Read active keypad button
void Keypad4x4_ReadKeypad(bool keys[16]);
//Function(5): Get character
char* Keypad4x4_GetChar(uint8_t keypadSw);



