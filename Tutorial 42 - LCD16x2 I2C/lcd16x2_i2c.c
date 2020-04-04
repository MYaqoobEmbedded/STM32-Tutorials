/*
 * lcd16x2_i2c.c
 *
 *  Created on: Mar 28, 2020
 *      Author: Mohamed Yaqoob
 */

#include "lcd16x2_i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* LCD Commands */
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_CURSORSHIFT     0x10
#define LCD_FUNCTIONSET     0x20
#define LCD_SETCGRAMADDR    0x40
#define LCD_SETDDRAMADDR    0x80

/* Commands bitfields */
//1) Entry mode Bitfields
#define LCD_ENTRY_SH      0x01
#define LCD_ENTRY_ID      0x02
//2) Display control
#define LCD_DISPLAY_B     0x01
#define LCD_DISPLAY_C     0x02
#define LCD_DISPLAY_D     0x04
//3) Shift control
#define LCD_SHIFT_RL      0x04
#define LCD_SHIFT_SC      0x08
//4) Function set control
#define LCD_FUNCTION_F    0x04
#define LCD_FUNCTION_N    0x08
#define LCD_FUNCTION_DL   0x10

/* I2C Control bits */
#define LCD_RS        (1 << 0)
#define LCD_RW        (1 << 1)
#define LCD_EN        (1 << 2)
#define LCD_BK_LIGHT  (1 << 3)

/* Library variables */
static I2C_HandleTypeDef* lcd16x2_i2cHandle;
static uint8_t LCD_I2C_SLAVE_ADDRESS=0;
#define LCD_I2C_SLAVE_ADDRESS_0  0x4E
#define LCD_I2C_SLAVE_ADDRESS_1  0x7E

/* Private functions */
static void lcd16x2_i2c_sendCommand(uint8_t command)
{
  const uint8_t command_0_3 = (0xF0 & (command<<4));
  const uint8_t command_4_7 = (0xF0 & command);
  uint8_t i2cData[4] =
  {
      command_4_7 | LCD_EN | LCD_BK_LIGHT,
      command_4_7 | LCD_BK_LIGHT,
      command_0_3 | LCD_EN | LCD_BK_LIGHT,
      command_0_3 | LCD_BK_LIGHT,
  };
  HAL_I2C_Master_Transmit(lcd16x2_i2cHandle, LCD_I2C_SLAVE_ADDRESS, i2cData, 4, 200);
}

static void lcd16x2_i2c_sendData(uint8_t data)
{
  const uint8_t data_0_3 = (0xF0 & (data<<4));
  const uint8_t data_4_7 = (0xF0 & data);
  uint8_t i2cData[4] =
  {
      data_4_7 | LCD_EN | LCD_BK_LIGHT | LCD_RS,
      data_4_7 | LCD_BK_LIGHT | LCD_RS,
      data_0_3 | LCD_EN | LCD_BK_LIGHT | LCD_RS,
      data_0_3 | LCD_BK_LIGHT | LCD_RS,
  };
  HAL_I2C_Master_Transmit(lcd16x2_i2cHandle, LCD_I2C_SLAVE_ADDRESS, i2cData, 4, 200);
}


/**
 * @brief Initialise LCD16x2
 * @param[in] *pI2cHandle - pointer to HAL I2C handle
 */
bool lcd16x2_i2c_init(I2C_HandleTypeDef *pI2cHandle)
{
  HAL_Delay(50);
  lcd16x2_i2cHandle = pI2cHandle;
  if(HAL_I2C_IsDeviceReady(lcd16x2_i2cHandle, LCD_I2C_SLAVE_ADDRESS_0, 5, 500) != HAL_OK)
  {
    if(HAL_I2C_IsDeviceReady(lcd16x2_i2cHandle, LCD_I2C_SLAVE_ADDRESS_1, 5, 500) != HAL_OK)
    {
      return false;
    }
    else
    {
      LCD_I2C_SLAVE_ADDRESS = LCD_I2C_SLAVE_ADDRESS_1;
    }
  }
  else
  {
    LCD_I2C_SLAVE_ADDRESS = LCD_I2C_SLAVE_ADDRESS_0;
  }
  //Initialise LCD for 4-bit operation
  //1. Wait at least 15ms
  HAL_Delay(45);
  //2. Attentions sequence
  lcd16x2_i2c_sendCommand(0x30);
  HAL_Delay(5);
  lcd16x2_i2c_sendCommand(0x30);
  HAL_Delay(1);
  lcd16x2_i2c_sendCommand(0x30);
  HAL_Delay(8);
  lcd16x2_i2c_sendCommand(0x20);
  HAL_Delay(8);

  lcd16x2_i2c_sendCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N);
  HAL_Delay(1);
  lcd16x2_i2c_sendCommand(LCD_DISPLAYCONTROL);
  HAL_Delay(1);
  lcd16x2_i2c_sendCommand(LCD_CLEARDISPLAY);
  HAL_Delay(3);
  lcd16x2_i2c_sendCommand(0x04 | LCD_ENTRY_ID);
  HAL_Delay(1);
  lcd16x2_i2c_sendCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_D);
  HAL_Delay(3);

  return true;
}

/**
 * @brief Set cursor position
 * @param[in] row - 0 or 1 for line1 or line2
 * @param[in] col - 0 - 15 (16 columns LCD)
 */
void lcd16x2_i2c_setCursor(uint8_t row, uint8_t col)
{
  uint8_t maskData;
  maskData = (col)&0x0F;
  if(row==0)
  {
    maskData |= (0x80);
    lcd16x2_i2c_sendCommand(maskData);
  }
  else
  {
    maskData |= (0xc0);
    lcd16x2_i2c_sendCommand(maskData);
  }
}

/**
 * @brief Move to beginning of 1st line
 */
void lcd16x2_i2c_1stLine(void)
{
  lcd16x2_i2c_setCursor(0,0);
}
/**
 * @brief Move to beginning of 2nd line
 */
void lcd16x2_i2c_2ndLine(void)
{
  lcd16x2_i2c_setCursor(1,0);
}

/**
 * @brief Select LCD Number of lines mode
 */
void lcd16x2_i2c_TwoLines(void)
{
  lcd16x2_i2c_sendCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N);
}
void lcd16x2_i2c_OneLine(void)
{
  lcd16x2_i2c_sendCommand(LCD_FUNCTIONSET);
}

/**
 * @brief Cursor ON/OFF
 */
void lcd16x2_i2c_cursorShow(bool state)
{
  if(state)
  {
    lcd16x2_i2c_sendCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D);
  }
  else
  {
    lcd16x2_i2c_sendCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_D);
  }
}

/**
 * @brief Display clear
 */
void lcd16x2_i2c_clear(void)
{
  lcd16x2_i2c_sendCommand(LCD_CLEARDISPLAY);
  HAL_Delay(3);
}

/**
 * @brief Display ON/OFF, to hide all characters, but not clear
 */
void lcd16x2_i2c_display(bool state)
{
  if(state)
  {
    lcd16x2_i2c_sendCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D);
  }
  else
  {
    lcd16x2_i2c_sendCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C);
  }
}

/**
 * @brief Shift content to right
 */
void lcd16x2_i2c_shiftRight(uint8_t offset)
{
  for(uint8_t i=0; i<offset;i++)
  {
    lcd16x2_i2c_sendCommand(0x1c);
  }
}

/**
 * @brief Shift content to left
 */
void lcd16x2_i2c_shiftLeft(uint8_t offset)
{
  for(uint8_t i=0; i<offset;i++)
  {
    lcd16x2_i2c_sendCommand(0x18);
  }
}

/**
 * @brief Print to display
 */
void lcd16x2_i2c_printf(const char* str, ...)
{
  char stringArray[20];
  va_list args;
  va_start(args, str);
  vsprintf(stringArray, str, args);
  va_end(args);
  for(uint8_t i=0;  i<strlen(stringArray) && i<16; i++)
  {
    lcd16x2_i2c_sendData((uint8_t)stringArray[i]);
  }
}
