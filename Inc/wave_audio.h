/**
  ******************************************************************************
  * @file    Audio_playback_and_record/inc/waverecorder.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for waverecorder.c module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIO_H
#define __AUDIO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "pdm_filter.h"
#include "main.h"


#if 0
/** the following definitions are from stm32f4-discovery header **/

/*------------------------------------------------------------------------------
                          AUDIO IN CONFIGURATION
------------------------------------------------------------------------------*/
/* SPI Configuration defines */
#define I2S2                            SPI2
#define I2S2_CLK_ENABLE()               __SPI2_CLK_ENABLE()
#define I2S2_SCK_PIN                    GPIO_PIN_13
#define I2S2_SCK_GPIO_PORT              GPIOB
#define I2S2_SCK_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define I2S2_SCK_AF                     GPIO_AF5_SPI2

#define I2S2_MOSI_PIN                   GPIO_PIN_15
#define I2S2_MOSI_GPIO_PORT             GPIOB
#define I2S2_MOSI_GPIO_CLK_ENABLE()     __GPIOB_CLK_ENABLE()
#define I2S2_MOSI_AF                    GPIO_AF5_SPI2

/* I2S DMA Stream Rx definitions */
#define I2S2_DMAx_CLK_ENABLE()          __DMA1_CLK_ENABLE()
#define I2S2_DMAx_STREAM                DMA1_Stream3
#define I2S2_DMAx_CHANNEL               DMA_CHANNEL_0
#define I2S2_DMAx_IRQ                   DMA1_Stream3_IRQn
#define I2S2_DMAx_PERIPH_DATA_SIZE      DMA_PDATAALIGN_HALFWORD
#define I2S2_DMAx_MEM_DATA_SIZE         DMA_MDATAALIGN_HALFWORD

#define I2S2_IRQHandler                 DMA1_Stream3_IRQHandler

/* Select the interrupt preemption priority and subpriority for the IT/DMA interrupt */
#define AUDIO_IN_IRQ_PREPRIO            6   /* Select the preemption priority level(0 is the highest) */

/*------------------------------------------------------------------------------
             CONFIGURATION: Audio Driver Configuration parameters
------------------------------------------------------------------------------*/

#define AUDIODATA_SIZE                  2   /* 16-bits audio data size */

/* Audio status definition */
#define AUDIO_OK                        0
#define AUDIO_ERROR                     1
#define AUDIO_TIMEOUT                   2

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define DEFAULT_AUDIO_IN_FREQ                 I2S_AUDIOFREQ_16K
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION       16
#define DEFAULT_AUDIO_IN_CHANNEL_NBR          1 /* Mono = 1, Stereo = 2 */
#define DEFAULT_AUDIO_IN_VOLUME               64

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE                    128*DEFAULT_AUDIO_IN_FREQ/16000*DEFAULT_AUDIO_IN_CHANNEL_NBR
/* PCM buffer output size */
#define PCM_OUT_SIZE                          DEFAULT_AUDIO_IN_FREQ/1000
#define CHANNEL_DEMUX_MASK                    0x55

#endif

/* Exported types ------------------------------------------------------------*/
/* Exported Defines ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Defines for the Audio recording process */
#define WR_BUFFER_SIZE           4096  /*  More the size is higher, the recorded quality is better */
#define DEFAULT_TIME_REC         30000  /* Recording time in millisecond (Systick Time Base*TIME_REC= 1ms*30000)
                                        (default: 30s), modified to 300s*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

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

}WAVE_FormatTypeDef;


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint32_t  WaveRecorderInit(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
uint8_t   WaveRecorderStart(uint16_t* pBuf, uint32_t wSize);
uint32_t  WaveRecorderStop(void);
void      WaveRecorderProcess(void);
void      WaveRecorderProcessingAudioBuffer(void);
#endif /* __WAVERECORDER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
