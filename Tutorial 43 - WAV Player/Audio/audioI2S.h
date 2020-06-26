/*
 * audioI2S.h
 *
 *  Created on: 17 Apr 2020
 *      Author: Mohamed Yaqoob
 */

#ifndef AUDIOI2S_H_
#define AUDIOI2S_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"

//Audio library defines
#define DMA_MAX_SZE                 0xFFFF
#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define AUDIODATA_SIZE              2   /* 16-bits audio data size */

/* I2S Audio library function prototypes */

/**
 * @brief set I2S HAL handle
 */
void audioI2S_setHandle(I2S_HandleTypeDef *pI2Shandle);

/**
 * @brief Initialises I2S Audio settings
 * @param audioFreq - WAV file Audio sampling rate (44.1KHz, 48KHz, ...)
 * @param volume - CS43L22 Codec volume settings (0 - 100)
 * @retval state - true: Successfully, false: Failed
 */
bool audioI2S_init(uint32_t audioFreq);

/**
 * @brief Starts Playing Audio from buffer
 */
bool audioI2S_play(uint16_t* pDataBuf, uint32_t len);

/**
 * @brief Change I2S DMA Buffer pointer
 */
bool audioI2S_changeBuffer(uint16_t* pDataBuf, uint32_t len);

/**
 * @brief Pause audio out
 */
void audioI2S_pause(void);

/**
 * @brief Resume audio out
 */
void audioI2S_resume(void);

/**
 * @brief Set Volume
 */
void audioI2S_setVolume(uint8_t volume);

/**
 * @brief Stop audio
 */
void audioI2S_stop(void);

/**
 * @brief Half/Full transfer Audio callback for buffer management
 */
void audioI2S_halfTransfer_Callback(void);
void audioI2S_fullTransfer_Callback(void);


#endif /* AUDIOI2S_H_ */
