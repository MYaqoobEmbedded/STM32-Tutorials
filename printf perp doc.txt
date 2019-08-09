Part1: Keil
--------------
Step(1): Generate normal CubeMX project
Step(2): Add the following on top of main
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PTD */

Step(3): Add body definition of PUTCHAR_PROTOTYPE
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

Step(4): Add stdio header (to use printf)
#include <stdio.h>

Step(5): Test printing hello world
printf("Hello world\r\n");

Step(6): Test printing integer
Step(7): Test printing float

Part2: for GNU compiler (SW4STM32), need to add the following flag
--------------------
Open your SW4STM32, go to: project -> properties > C/C+ build > Settings > MCU GCC Linker > Miscellaneous > Linker flags: add this flag: -u _printf_float
