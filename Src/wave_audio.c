/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/waverecorder.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   I2S Audio recorder program.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "bsp_audio.h"
#include "wave_audio.h"
#include "string.h"
#include "pdm_filter.h"

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
* @{
*/

static const char* volume_string [] = {
    "|",
    "||",
    "|||",
    "||||",
    "|||||",
    "||||||",
    "|||||||",
    "||||||||",
    "|||||||||",
    "||||||||||",
    "|||||||||||",
    "||||||||||||",
    "|||||||||||||",
    "||||||||||||||",
    "|||||||||||||||",
    "||||||||||||||||",
    "|||||||||||||||||",
    "||||||||||||||||||",
    "|||||||||||||||||||",
    "||||||||||||||||||||",
    "|||||||||||||||||||||",
    "||||||||||||||||||||||",
    "|||||||||||||||||||||||",
    "||||||||||||||||||||||||",
    "|||||||||||||||||||||||||",
    "||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
    "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||",
};

uint32_t size_of_volume_string = sizeof(volume_string) / sizeof(volume_string[0]);

int __errno;

/* Private typedef -----------------------------------------------------------*/
typedef struct {
  int32_t offset;
  uint32_t fptr;
}Audio_BufferTypeDef;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Wave recorded counter.*/
__IO uint32_t WaveCounter = 0;

// extern __IO uint32_t TimeRecBase;

// FIL WavFile;

/*USB variable to check if USB connected or not*/
// extern MSC_ApplicationTypeDef AppliState;

/* Variable used to switch play from audio sample available on USB to recorded file*/
uint32_t WaveRecStatus = 0;

uint8_t pHeaderBuff[44];
uint16_t WrBuffer[WR_BUFFER_SIZE];

static uint16_t RecBuf[PCM_OUT_SIZE*2];/* PCM stereo samples are saved in RecBuf */
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];
__IO uint32_t ITCounter = 0;
Audio_BufferTypeDef  BufferCtl;

/* Temporary data sample */
__IO uint32_t AUDIODataReady = 0, AUDIOBuffOffset = 0;

WAVE_FormatTypeDef WaveFormat;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
  A single MEMS microphone MP45DT02 mounted on STM32F4-Discovery is connected
  to the Inter-IC Sound (I2S) peripheral. The I2S is configured in master
  receiver mode. In this mode, the I2S peripheral provides the clock to the MEMS
  microphone through CLK_IN and acquires the data (Audio samples) from the MEMS
  microphone through PDM_OUT.

  Data acquisition is performed in 16-bit PDM format and using I2S IT mode.

  In order to avoid data-loss, two buffers are used:
   - When there are enough data to be transmitted to USB, there is a buffer reception
   switch.

  To avoid data-loss:
  - IT ISR priority must be set at a higher priority than USB, this priority
    order must be respected when managing other interrupts;

  Note that a PDM Audio software decoding library provided in binary is used in
  this application. For IAR EWARM toolchain, the library is labeled
  "libPDMFilter_CM4_IAR.a".
*/


/**
  * @brief  Start Audio recording.
  * @param  pbuf: pointer to a buffer
  *         size: Buffer size
  * @retval None
  */
uint8_t WaveRecorderStart(uint16_t* pBuf, uint32_t wSize)
{
  return (BSP_AUDIO_IN_Record(pBuf, wSize));
}

/**
  * @brief  Stop Audio recording.
  * @param  None
  * @retval None
  */
uint32_t WaveRecorderStop(void)
{
  return BSP_AUDIO_IN_Stop();
}



#define VOLUME_DIVIDER    (16)
/*
 * The input is 4096 byte, 2 channel, 16 bit, little endian data.
 *
 * 1024 samples
 */
static void print_sound(uint8_t* buf) {

  uint32_t i, j;
  long long sum_square;
  double root;
  unsigned int rooti;
  int16_t* p;

  sum_square = 0;

  for (i = 0; i < 1024; i++) {
    j = i * 4;

    p = (int16_t*)(&buf[j]);

    sum_square += ((long long)(*p)) * ((long long)(*p));
  }

  sum_square = sum_square / 1024;

  root = sqrt((double)sum_square);
  rooti = (unsigned int)root;

  // printf("sum_square %u\r\n", rooti);
  if ((rooti / VOLUME_DIVIDER) >= size_of_volume_string)
  {
    printf("%s", volume_string[size_of_volume_string - 1]);
  }
  else {
    printf("%s", volume_string[rooti / VOLUME_DIVIDER]);
  }

  printf("\r\n");
}

/**
  * @brief  Update the recorded data.
  * @param  None
  * @retval None
  */
void WaveRecorderProcess(void)
{
  /* Current size of the recorded buffer */
  uint32_t byteswritten = 0;

  // WaveCounter = 0;
  // LEDsState = LEDS_OFF;

  /* Increment the Wave counter */
  BufferCtl.fptr = byteswritten;

  BufferCtl.offset = BUFFER_OFFSET_NONE;
  BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
  BSP_AUDIO_IN_Record((uint16_t*)&InternalBuffer[0], INTERNAL_BUFF_SIZE);

  /* Reset the time recording base variable */
  ITCounter = 0;
  while (1)
  {
    if(AUDIODataReady == 1)
    {
      print_sound((uint8_t*)(WrBuffer+AUDIOBuffOffset));

      BufferCtl.fptr += byteswritten;
      AUDIODataReady = 0;
    }
  }
}

/**
  * @brief Calculates the remaining file size and new position of the pointer.
  * @param None
  * @retval None
  */
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /* PDM to PCM data convert */
  BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[INTERNAL_BUFF_SIZE/2], (uint16_t*)&RecBuf[0]);

  /* Copy PCM data in internal buffer */
  memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);

  BufferCtl.offset = BUFFER_OFFSET_NONE;

  if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*4))-1)
  {
    AUDIODataReady = 1;
    AUDIOBuffOffset = 0;
    ITCounter++;
  }
  else if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*2))-1)
  {
    AUDIODataReady = 1;
    AUDIOBuffOffset = WR_BUFFER_SIZE/2;
    ITCounter = 0;
  }
  else
  {
    ITCounter++;
  }
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  /* PDM to PCM data convert */
  BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[0], (uint16_t*)&RecBuf[0]);

  /* Copy PCM data in internal buffer */
  memcpy((uint16_t*)&WrBuffer[ITCounter * (PCM_OUT_SIZE*2)], RecBuf, PCM_OUT_SIZE*4);

  BufferCtl.offset = BUFFER_OFFSET_NONE;

  if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*4))-1)
  {
    AUDIODataReady = 1;
    AUDIOBuffOffset = 0;
    ITCounter++;
  }
  else if(ITCounter == (WR_BUFFER_SIZE/(PCM_OUT_SIZE*2))-1)
  {
    AUDIODataReady = 1;
    AUDIOBuffOffset = WR_BUFFER_SIZE/2;
    ITCounter = 0;
  }
  else
  {
    ITCounter++;
  }
}

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
