/*
Library:						4x4 Keypad drive for STM32 MCUs
Written by:					Mohamed Yaqoob
Date written:				03/04/2018
Description:				The MY_Keypad4x4 library consists of the following public functions
										Function(1)- Keypad4x4_Init
										Function(2)- Keypad4x4_ReadKeypad
										Function(3)- Keypad4x4_GetChar
*/

//***** Header files *****//
#include "MY_Keypad4x4.h"

//***** Library variables *****//
//1. Keypad pinout variable
static Keypad_WiresTypeDef KeypadStruct;
//2. OUT pins position, or pin number in decimal for use in colomn change function
static uint8_t OutPositions[4];
//
static char *Keypad_keys[16] =
{
	"1",
	"2",
	"3",
	"A",
	"4",
	"5",
	"6",
	"B",
	"7",
	"8",
	"9",
	"C",
	"*",
	"0",
	"#",
	"D"
};

//***** Functions definition *****//
//Function(1): Set Keypad pins and ports
void Keypad4x4_Init(Keypad_WiresTypeDef  *KeypadWiringStruct)
{
	//Step(1): Copy the Keypad wirings to the library
	KeypadStruct = *KeypadWiringStruct;
	//Step(2): Find the positions of the 4 OUT pins
	Keypad4x4_FindPins_positions();
	//Step(3): Initialise all pins to set all OUT pins to RESET
	KeypadStruct.OUT0_Port->OTYPER |= (1UL << OutPositions[0]);
	KeypadStruct.OUT1_Port->OTYPER |= (1UL << OutPositions[1]);
	KeypadStruct.OUT2_Port->OTYPER |= (1UL << OutPositions[2]);
	KeypadStruct.OUT3_Port->OTYPER |= (1UL << OutPositions[3]);
	
	HAL_GPIO_WritePin(KeypadStruct.OUT0_Port, KeypadStruct.OUT0pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KeypadStruct.OUT1_Port, KeypadStruct.OUT1pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KeypadStruct.OUT2_Port, KeypadStruct.OUT2pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KeypadStruct.OUT3_Port, KeypadStruct.OUT3pin, GPIO_PIN_SET);
}
//Function(2): Get pin positions for colomn change use, only for out pins
static void Keypad4x4_FindPins_positions(void)
{
	uint8_t idx=0;
	for(idx=0; idx<16; idx++)
	{
		if(((KeypadStruct.OUT0pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[0] = idx;
		}
		if(((KeypadStruct.OUT1pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[1] = idx;
		}
		if(((KeypadStruct.OUT2pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[2] = idx;
		}
		if(((KeypadStruct.OUT3pin>>idx)&0x0001) == 0x0001)
		{
			OutPositions[3] = idx;
		}
	}
}
//Function(3): Change colomn number
static void Keypad4x4_ChangeColomn(uint8_t colNum_0_to_3)
{
	if(colNum_0_to_3==0)
	{
		//Set selected colomn
		KeypadStruct.OUT0_Port->OTYPER &= ~(1UL << OutPositions[0]);
		
		//Make other colomns floating
		KeypadStruct.OUT1_Port->OTYPER |= (1UL << OutPositions[1]);
		KeypadStruct.OUT2_Port->OTYPER |= (1UL << OutPositions[2]);
		KeypadStruct.OUT3_Port->OTYPER |= (1UL << OutPositions[3]);
	}
	else if(colNum_0_to_3==1)
	{
		//Set selected colomn
		KeypadStruct.OUT1_Port->OTYPER &= ~(1UL << OutPositions[1]);
		
		//Make other colomns floating
		KeypadStruct.OUT0_Port->OTYPER |= (1UL << OutPositions[0]);
		KeypadStruct.OUT2_Port->OTYPER |= (1UL << OutPositions[2]);
		KeypadStruct.OUT3_Port->OTYPER |= (1UL << OutPositions[3]);
	}
	else if(colNum_0_to_3==2)
	{
		//Set selected colomn
		KeypadStruct.OUT2_Port->OTYPER &= ~(1UL << OutPositions[2]);
		
		//Make other colomns floating
		KeypadStruct.OUT0_Port->OTYPER |= (1UL << OutPositions[0]);
		KeypadStruct.OUT1_Port->OTYPER |= (1UL << OutPositions[1]);
		KeypadStruct.OUT3_Port->OTYPER |= (1UL << OutPositions[3]);
	}
	else if(colNum_0_to_3==3)
	{
		//Set selected colomn
		KeypadStruct.OUT3_Port->OTYPER &= ~(1UL << OutPositions[3]);
		
		//Make other colomns floating
		KeypadStruct.OUT0_Port->OTYPER |= (1UL << OutPositions[0]);
		KeypadStruct.OUT1_Port->OTYPER |= (1UL << OutPositions[1]);
		KeypadStruct.OUT2_Port->OTYPER |= (1UL << OutPositions[2]);
	}
}

//Function(4): Read active keypad button
void Keypad4x4_ReadKeypad(bool keys[16])
{
	//Step(1): Make Col0 High and check the rows
	Keypad4x4_ChangeColomn(0);
	keys[0] = HAL_GPIO_ReadPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[4] = HAL_GPIO_ReadPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[8] = HAL_GPIO_ReadPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[12] = HAL_GPIO_ReadPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);
	
	//Step(2): Make Col1 High and check the rows
	Keypad4x4_ChangeColomn(1);
	keys[1] = HAL_GPIO_ReadPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[5] = HAL_GPIO_ReadPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[9] = HAL_GPIO_ReadPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[13] = HAL_GPIO_ReadPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);
	
	//Step(3): Make Col2 High and check the rows
	Keypad4x4_ChangeColomn(2);
	keys[2] = HAL_GPIO_ReadPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[6] = HAL_GPIO_ReadPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[10] = HAL_GPIO_ReadPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[14] = HAL_GPIO_ReadPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);
	
	//Step(4): Make Col3 High and check the rows
	Keypad4x4_ChangeColomn(3);
	keys[3] = HAL_GPIO_ReadPin(KeypadStruct.IN0_Port, KeypadStruct.IN0pin);
	keys[7] = HAL_GPIO_ReadPin(KeypadStruct.IN1_Port, KeypadStruct.IN1pin);
	keys[11] = HAL_GPIO_ReadPin(KeypadStruct.IN2_Port, KeypadStruct.IN2pin);
	keys[15] = HAL_GPIO_ReadPin(KeypadStruct.IN3_Port, KeypadStruct.IN3pin);
}	
//Function(5): Get character
char* Keypad4x4_GetChar(uint8_t keypadSw)
{
	return Keypad_keys[keypadSw];
}

