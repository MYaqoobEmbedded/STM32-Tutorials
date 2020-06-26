/*
 * audioI2S.c
 *
 *  Created on: 17 Apr 2020
 *      Author: Mohamed Yaqoob
 */

#include "audioI2S.h"
#include "MY_CS43L22.h"
#include "stm32f4xx_hal.h"

//I2S PLL parameters for different I2S Sampling Frequency
const uint32_t I2SFreq[8] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
const uint32_t I2SPLLN[8] = {256, 429, 213, 429, 426, 271, 258, 344};
const uint32_t I2SPLLR[8] = {5, 4, 4, 4, 4, 6, 3, 1};

static I2S_HandleTypeDef *hAudioI2S;

//Static functions

/**
 * @brief I2S Clock Config
 */
static void audioI2S_pllClockConfig(uint32_t audioFreq)
{
  RCC_PeriphCLKInitTypeDef rccclkinit;
  uint8_t index = 0, freqindex = 0xFF;

  for(index = 0; index < 8; index++)
  {
    if(I2SFreq[index] == audioFreq)
    {
      freqindex = index;
    }
  }
  /* Enable PLLI2S clock */
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
  /* PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  if ((freqindex & 0x7) == 0)
  {
    /* I2S clock config
    PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) × (PLLI2SN/PLLM)
    I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = I2SPLLN[freqindex];
    rccclkinit.PLLI2S.PLLI2SR = I2SPLLR[freqindex];
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else
  {
    /* I2S clock config
    PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) × (PLLI2SN/PLLM)
    I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 258;
    rccclkinit.PLLI2S.PLLI2SR = 3;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
}

/**
 * @brief update I2S peripheral with selected Sampling Frequency
 */
static bool I2S3_freqUpdate(uint32_t AudioFreq)
{
  /* Initialize the hAudioOutI2s Instance parameter */
  hAudioI2S->Instance         = SPI3;

 /* Disable I2S block */
  __HAL_I2S_DISABLE(hAudioI2S);

  /* I2S3 peripheral configuration */
  hAudioI2S->Init.AudioFreq   = AudioFreq;
  hAudioI2S->Init.ClockSource = I2S_CLOCK_PLL;
  hAudioI2S->Init.CPOL        = I2S_CPOL_LOW;
  hAudioI2S->Init.DataFormat  = I2S_DATAFORMAT_16B;
  hAudioI2S->Init.MCLKOutput  = I2S_MCLKOUTPUT_ENABLE;
  hAudioI2S->Init.Mode        = I2S_MODE_MASTER_TX;
  hAudioI2S->Init.Standard    = I2S_STANDARD_PHILIPS;
  /* Initialize the I2S peripheral with the structure above */
  if(HAL_I2S_Init(hAudioI2S) != HAL_OK)
  {
    return false;
  }
  else
  {
    return true;
  }
}

/**
 * @brief set I2S HAL handle
 */
void audioI2S_setHandle(I2S_HandleTypeDef *pI2Shandle)
{
  hAudioI2S = pI2Shandle;
}

/* I2S Audio library function definitions */
/**
 * @brief Initialises I2S Audio settings
 * @param audioFreq - WAV file Audio sampling rate (44.1KHz, 48KHz, ...)
 * @param volume - CS43L22 Codec volume settings (0 - 100)
 * @retval state - true: Successfully, false: Failed
 */
bool audioI2S_init(uint32_t audioFreq)
{
  //Update PLL Clock Frequency setting
  audioI2S_pllClockConfig(audioFreq);
  //Update I2S peripheral sampling frequency
  I2S3_freqUpdate(audioFreq);
  return true;
}

/**
 * @brief Starts Playing Audio from buffer
 */
bool audioI2S_play(uint16_t* pDataBuf, uint32_t len)
{
  //Start Codec
  CS43_Start();
  //Start I2S DMA transfer
  HAL_I2S_Transmit_DMA(hAudioI2S, pDataBuf, DMA_MAX(len/AUDIODATA_SIZE));
  return true;
}

/**
 * @brief Change I2S DMA Buffer pointer
 */
bool audioI2S_changeBuffer(uint16_t* pDataBuf, uint32_t len)
{
  HAL_I2S_Transmit_DMA(hAudioI2S, pDataBuf, DMA_MAX(len));
  return true;
}

/**
 * @brief Pause audio out
 */
void audioI2S_pause(void)
{
  CS43_Stop();
  HAL_I2S_DMAPause(hAudioI2S);
}

/**
 * @brief Resume audio out
 */
void audioI2S_resume(void)
{
  CS43_Start();
  HAL_I2S_DMAResume(hAudioI2S);
}

/**
 * @brief Set Volume
 */
void audioI2S_setVolume(uint8_t volume)
{
  CS43_SetVolume(volume);
}

/**
 * @brief Stop audio
 */
void audioI2S_stop(void)
{
  CS43_Stop();
  HAL_I2S_DMAStop(hAudioI2S);
}

/**
 * @brief Half/Full transfer Audio callback for buffer management
 */
__weak void audioI2S_halfTransfer_Callback(void)
{

}
__weak void audioI2S_fullTransfer_Callback(void)
{

}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(hi2s->Instance == SPI3)
  {
    audioI2S_fullTransfer_Callback();
  }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if(hi2s->Instance == SPI3)
  {
    audioI2S_halfTransfer_Callback();
  }
}

