/*
 * wav_player.h
 *
 *  Created on: 17 Apr 2020
 *      Author: Mohamed Yaqoob
 */

#ifndef WAV_PLAYER_H_
#define WAV_PLAYER_H_

#include <stdbool.h>
#include <stdint.h>

//Audio buffer state
typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

typedef enum
{
  PLAY_Idle=0,
  PLAY_Pause,
  PLAY_Resume,
}PLAY_State_e;

typedef struct
{
  uint32_t   ChunkID;       /* 0 */
  uint32_t   FileSize;      /* 4 */
  uint32_t   FileFormat;    /* 8 */
  uint32_t   SubChunk1ID;   /* 12 */
  uint32_t   SubChunk1Size; /* 16*/
  uint16_t   AudioFormat;   /* 20 */
  uint16_t   NbrChannels;   /* 22 */
  uint32_t   SampleRate;    /* 24 */

  uint32_t   ByteRate;      /* 28 */
  uint16_t   BlockAlign;    /* 32 */
  uint16_t   BitPerSample;  /* 34 */
  uint32_t   SubChunk2ID;   /* 36 */
  uint32_t   SubChunk2Size; /* 40 */

}WAV_HeaderTypeDef;

/**
 * @brief Select WAV file to play
 * @retval returns true when file is found in USB Drive
 */
bool wavPlayer_fileSelect(const char* filePath);

/**
 * @brief WAV File Play
 */
void wavPlayer_play(void);

/**
 * @brief WAV stop
 */
void wavPlayer_stop(void);

/**
 * @brief Process WAV
 */
void wavPlayer_process(void);

/**
 * @brief isEndofFile reached
 */
bool wavPlayer_isFinished(void);

/**
 * @brief WAV pause/resume
 */
void wavPlayer_pause(void);
void wavPlayer_resume(void);

#endif /* WAV_PLAYER_H_ */
