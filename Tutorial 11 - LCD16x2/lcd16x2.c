/*
  Library:        lcd16x2 - Parallel 8/4 bits
  Written by:     Mohamed Yaqoob
  Date Written:   04/12/2017
  Updated:        26/06/2020
  Description:    This is a library for the standard 16X2 LCD display, for the STM32 MCUs based on HAL libraries.
                  It perfroms the basic Text/Number printing to your 16X2 LCD, in 8 bits and 4 bits modes of operation.

  References**:
                  This was written based on the open source Arduino LiquidCrystal library
                  and by referring to the DATASHEET of the LCD16X2, also with the help of
                  the following YouTube tutorials on LCD 16X2:
                  (1): 'RC Tractor Guy' YouTube tutorial on the following link:
                       https://www.youtube.com/watch?v=efi2nlsvbCI
                  (2): 'Explore Embedded' YouTube tutorial on the following link:
                       https://www.youtube.com/watch?v=YDJISiPUdA8

 * Copyright (C) 2017 - M.Yaqoob - MutexEmbedded
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public License version 3 as published by the Free Software Foundation.

   This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

#include "lcd16x2.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

//Milisecond function
#define LCD_MS_DELAY(X) (HAL_Delay(X))

/* List of COMMANDS */
#define LCD_CLEARDISPLAY      0x01
#define LCD_RETURNHOME        0x02
#define LCD_ENTRYMODESET      0x04
#define LCD_DISPLAYCONTROL    0x08
#define LCD_CURSORSHIFT       0x10
#define LCD_FUNCTIONSET       0x20
#define LCD_SETCGRAMADDR      0x40
#define LCD_SETDDRAMADDR      0x80

/* List of commands Bitfields */
//1) Entry mode Bitfields
#define LCD_ENTRY_SH          0x01
#define LCD_ENTRY_ID          0x02
//2) Entry mode Bitfields
#define LCD_ENTRY_SH          0x01
#define LCD_ENTRY_ID          0x02
//3) Display control
#define LCD_DISPLAY_B         0x01
#define LCD_DISPLAY_C         0x02
#define LCD_DISPLAY_D         0x04
//4) Shift control
#define LCD_SHIFT_RL          0x04
#define LCD_SHIFT_SC          0x08
//5) Function set control
#define LCD_FUNCTION_F        0x04
#define LCD_FUNCTION_N        0x08
#define LCD_FUNCTION_DL       0x10

/* LCD Library Variables */
static bool is8BitsMode = true;
static GPIO_TypeDef* PORT_RS_and_E;               // RS and E PORT
static uint16_t PIN_RS, PIN_E;                    // RS and E pins
static GPIO_TypeDef* PORT_LSB;                    // LSBs D0, D1, D2 and D3 PORT
static uint16_t D0_PIN, D1_PIN, D2_PIN, D3_PIN;   // LSBs D0, D1, D2 and D3 pins
static GPIO_TypeDef* PORT_MSB;                    // MSBs D5, D6, D7 and D8 PORT
static uint16_t D4_PIN, D5_PIN, D6_PIN, D7_PIN;   // MSBs D5, D6, D7 and D8 pins
#define T_CONST   20
static uint8_t DisplayControl = 0x0F;
static uint8_t FunctionSet = 0x38;

/* private functions prototypes */
/**
 * @brief DWT Cortex Tick counter for Microsecond delay
 */
static uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  /* Enable TRC */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  /* Enable clock cycle counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;
  /* 3 NO OPERATION instructions */
  __NOP();
  __NOP();
  __NOP();
  /* Check if clock cycle counter has started */
  if(DWT->CYCCNT)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t usec)
{
 uint32_t clk_cycle_start = DWT->CYCCNT;
 usec *= (HAL_RCC_GetHCLKFreq() / 1000000);
 while ((DWT->CYCCNT - clk_cycle_start) < usec);
}

/**
 * @brief Enable Pulse function
 */
static void lcd16x2_enablePulse(void)
{
  HAL_GPIO_WritePin(PORT_RS_and_E, PIN_E, GPIO_PIN_SET);
  DWT_Delay_us(T_CONST);
  HAL_GPIO_WritePin(PORT_RS_and_E, PIN_E, GPIO_PIN_RESET);
  DWT_Delay_us(60);
}

/**
 * @brief RS control
 */
static void lcd16x2_rs(bool state)
{
  HAL_GPIO_WritePin(PORT_RS_and_E, PIN_RS, (GPIO_PinState)state);
}

/**
 * @brief Write parallel signal to lcd
 */
static void lcd16x2_write(uint8_t wbyte)
{
  uint8_t LSB_nibble = wbyte&0xF, MSB_nibble = (wbyte>>4)&0xF;
  if(is8BitsMode)
  {
    //LSB data
    HAL_GPIO_WritePin(PORT_LSB, D0_PIN, (GPIO_PinState)(LSB_nibble&0x1));
    HAL_GPIO_WritePin(PORT_LSB, D1_PIN, (GPIO_PinState)(LSB_nibble&0x2));
    HAL_GPIO_WritePin(PORT_LSB, D2_PIN, (GPIO_PinState)(LSB_nibble&0x4));
    HAL_GPIO_WritePin(PORT_LSB, D3_PIN, (GPIO_PinState)(LSB_nibble&0x8));
    //MSB data
    HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(MSB_nibble&0x1));
    HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(MSB_nibble&0x2));
    HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(MSB_nibble&0x4));
    HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(MSB_nibble&0x8));
    lcd16x2_enablePulse();
  }
  else
  {
    //MSB data
    HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(MSB_nibble&0x1));
    HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(MSB_nibble&0x2));
    HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(MSB_nibble&0x4));
    HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(MSB_nibble&0x8));
    lcd16x2_enablePulse();
    //LSB data
    HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(LSB_nibble&0x1));
    HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(LSB_nibble&0x2));
    HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(LSB_nibble&0x4));
    HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(LSB_nibble&0x8));
    lcd16x2_enablePulse();
  }
}

/**
 * @brief Write command
 */
static void lcd16x2_writeCommand(uint8_t cmd)
{
  lcd16x2_rs(false);
  lcd16x2_write(cmd);
}

/**
 * @brief Write data
 */
static void lcd16x2_writeData(uint8_t data)
{
  lcd16x2_rs(true);
  lcd16x2_write(data);
}

/**
 * @brief 4-bits write
 */
static void lcd16x2_write4(uint8_t nib)
{
  nib &= 0xF;
  lcd16x2_rs(false);
  //LSB data
  HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(nib&0x1));
  HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(nib&0x2));
  HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(nib&0x4));
  HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(nib&0x8));
  lcd16x2_enablePulse();
}

/* Public functions definitions */

/**
 * @brief Initialise LCD on 8-bits mode
 * @param[in] *port_rs_e RS and EN GPIO Port (e.g. GPIOB)
 * @param[in] *port_0_3 D0 to D3 GPIO Port
 * @param[in] *port_4_7 D4 to D7 GPIO Port
 * @param[in] x_pin GPIO pin (e.g. GPIO_PIN_1)
 */
void lcd16x2_init_8bits(
    GPIO_TypeDef* port_rs_e, uint16_t rs_pin, uint16_t e_pin,
    GPIO_TypeDef* port_0_3, uint16_t d0_pin, uint16_t d1_pin, uint16_t d2_pin, uint16_t d3_pin,
    GPIO_TypeDef* port_4_7, uint16_t d4_pin, uint16_t d5_pin, uint16_t d6_pin, uint16_t d7_pin)
{
  DWT_Delay_Init();
  //Set GPIO Ports and Pins data
  PORT_RS_and_E = port_rs_e;
  PIN_RS = rs_pin;
  PIN_E = e_pin;
  PORT_LSB = port_0_3;
  D0_PIN = d0_pin;
  D1_PIN = d1_pin;
  D2_PIN = d2_pin;
  D3_PIN = d3_pin;
  PORT_MSB = port_4_7;
  D4_PIN = d4_pin;
  D5_PIN = d5_pin;
  D6_PIN = d6_pin;
  D7_PIN = d7_pin;
  is8BitsMode = true;
  FunctionSet = 0x38;

  //Initialise LCD
  //1. Wait at least 15ms
  LCD_MS_DELAY(20);
  //2. Attentions sequence
  lcd16x2_writeCommand(0x30);
  LCD_MS_DELAY(5);
  lcd16x2_writeCommand(0x30);
  LCD_MS_DELAY(1);
  lcd16x2_writeCommand(0x30);
  LCD_MS_DELAY(1);
  //3. Function set; Enable 2 lines, Data length to 8 bits
  lcd16x2_writeCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N | LCD_FUNCTION_DL);
  //4. Display control (Display ON, Cursor ON, blink cursor)
  lcd16x2_writeCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D);
  //5. Clear LCD and return home
  lcd16x2_writeCommand(LCD_CLEARDISPLAY);
  LCD_MS_DELAY(2);
}

/**
 * @brief Initialise LCD on 4-bits mode
 * @param[in] *port_4_7 D4 to D7 GPIO Port
 * @param[in] x_pin GPIO pin (e.g. GPIO_PIN_1)
 */
void lcd16x2_init_4bits(
    GPIO_TypeDef* port_rs_e, uint16_t rs_pin, uint16_t e_pin,
    GPIO_TypeDef* port_4_7, uint16_t d4_pin, uint16_t d5_pin, uint16_t d6_pin, uint16_t d7_pin)
{
  DWT_Delay_Init();
  //Set GPIO Ports and Pins data
  PORT_RS_and_E = port_rs_e;
  PIN_RS = rs_pin;
  PIN_E = e_pin;
  PORT_MSB = port_4_7;
  D4_PIN = d4_pin;
  D5_PIN = d5_pin;
  D6_PIN = d6_pin;
  D7_PIN = d7_pin;
  is8BitsMode = false;
  FunctionSet = 0x28;

  //Initialise LCD
  //1. Wait at least 15ms
  LCD_MS_DELAY(20);
  //2. Attentions sequence
  lcd16x2_write4(0x3);
  LCD_MS_DELAY(5);
  lcd16x2_write4(0x3);
  LCD_MS_DELAY(1);
  lcd16x2_write4(0x3);
  LCD_MS_DELAY(1);
  lcd16x2_write4(0x2);  //4 bit mode
  LCD_MS_DELAY(1);
  //4. Function set; Enable 2 lines, Data length to 4 bits
  lcd16x2_writeCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N);
  //3. Display control (Display ON, Cursor ON, blink cursor)
  lcd16x2_writeCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D);
  //4. Clear LCD and return home
  lcd16x2_writeCommand(LCD_CLEARDISPLAY);
  LCD_MS_DELAY(3);
}

/**
 * @brief Set cursor position
 * @param[in] row - 0 or 1 for line1 or line2
 * @param[in] col - 0 - 15 (16 columns LCD)
 */
void lcd16x2_setCursor(uint8_t row, uint8_t col)
{
  uint8_t maskData;
  maskData = (col)&0x0F;
  if(row==0)
  {
    maskData |= (0x80);
    lcd16x2_writeCommand(maskData);
  }
  else
  {
    maskData |= (0xc0);
    lcd16x2_writeCommand(maskData);
  }
}
/**
 * @brief Move to beginning of 1st line
 */
void lcd16x2_1stLine(void)
{
  lcd16x2_setCursor(0,0);
}
/**
 * @brief Move to beginning of 2nd line
 */
void lcd16x2_2ndLine(void)
{
  lcd16x2_setCursor(1,0);
}

/**
 * @brief Select LCD Number of lines mode
 */
void lcd16x2_twoLines(void)
{
  FunctionSet |= (0x08);
  lcd16x2_writeCommand(FunctionSet);
}
void lcd16x2_oneLine(void)
{
  FunctionSet &= ~(0x08);
  lcd16x2_writeCommand(FunctionSet);
}

/**
 * @brief Cursor ON/OFF
 */
void lcd16x2_cursorShow(bool state)
{
  if(state)
  {
    DisplayControl |= (0x03);
    lcd16x2_writeCommand(DisplayControl);
  }
  else
  {
    DisplayControl &= ~(0x03);
    lcd16x2_writeCommand(DisplayControl);
  }
}

/**
 * @brief Display clear
 */
void lcd16x2_clear(void)
{
  lcd16x2_writeCommand(LCD_CLEARDISPLAY);
  LCD_MS_DELAY(3);
}

/**
 * @brief Display ON/OFF, to hide all characters, but not clear
 */
void lcd16x2_display(bool state)
{
  if(state)
  {
    DisplayControl |= (0x04);
    lcd16x2_writeCommand(DisplayControl);
  }
  else
  {
    DisplayControl &= ~(0x04);
    lcd16x2_writeCommand(DisplayControl);
  }
}

/**
 * @brief Shift content to right
 */
void lcd16x2_shiftRight(uint8_t offset)
{
  for(uint8_t i=0; i<offset;i++)
  {
    lcd16x2_writeCommand(0x1c);
  }
}

/**
 * @brief Shift content to left
 */
void lcd16x2_shiftLeft(uint8_t offset)
{
  for(uint8_t i=0; i<offset;i++)
  {
    lcd16x2_writeCommand(0x18);
  }
}

/**
 * @brief Print to display any datatype (e.g. lcd16x2_printf("Value1 = %.1f", 123.45))
 */
void lcd16x2_printf(const char* str, ...)
{
  char stringArray[20];
  va_list args;
  va_start(args, str);
  vsprintf(stringArray, str, args);
  va_end(args);
  for(uint8_t i=0;  i<strlen(stringArray) && i<16; i++)
  {
    lcd16x2_writeData((uint8_t)stringArray[i]);
  }
}
