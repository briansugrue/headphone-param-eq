/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_audio_if.c
  * @version        : v1.0_Cube
  * @brief          : Generic media access layer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */
#include "main.h"
#include "eq.h"
#include <string.h>
extern I2S_HandleTypeDef hi2s3;

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_AUDIO_IF
  * @{
  */

/** @defgroup USBD_AUDIO_IF_Private_TypesDefinitions USBD_AUDIO_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Defines USBD_AUDIO_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* All sizes below are in stereo FRAMES (1 frame = L + R = 4 bytes) */
#define BYTES_PER_FRAME   4U
#define DMA_FRAMES        256U                       /* our I2S DMA buffer: 256 frames = 5.33 ms  */
#define HALF_FRAMES       (DMA_FRAMES / 2U)          /* refilled every half/full IRQ (2.67 ms)    */
#define CB_FRAMES         ((uint32_t)AUDIO_TOTAL_BUF_SIZE / BYTES_PER_FRAME) /* class buffer     */

#define FRAMES_PER_MS     (USBD_AUDIO_FREQ / 1000U)            /* 48 frames per ms */
#define TARGET_FILL_MS    15U                                  /* margin you want  */
#define FILL_TOL_MS       5U                                   /* dead band        */

#define TARGET_FILL       (TARGET_FILL_MS * FRAMES_PER_MS)     /* 720 frames       */
#define FILL_TOL          (FILL_TOL_MS * FRAMES_PER_MS)        /* 240 frames       */
#define FILL_GUARD        48U                                  /* extra margin for under/overrun detection  */

/* Compile-time sanity checks: the build fails with "size of array is negative"
 * if the dead band doesn't fit inside the buffer limits */
#define STATIC_CHECK(name, cond)  typedef char static_check_##name[(cond) ? 1 : -1]

STATIC_CHECK(lower_edge, TARGET_FILL > (FILL_TOL + HALF_FRAMES + FILL_GUARD));
STATIC_CHECK(upper_edge, (TARGET_FILL + FILL_TOL + HALF_FRAMES + FILL_GUARD) < CB_FRAMES);
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Macros USBD_AUDIO_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_Variables USBD_AUDIO_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* Buffer the I2S DMA plays from (circular). Must stay in normal RAM, not CCM. */
static int16_t dmaBuf[DMA_FRAMES * 2U] __attribute__((aligned(4)));

static uint32_t rd;            /* our read position in the class buffer (frames)          */
static uint32_t lastWr;        /* wr position seen on the previous callback (frames)      */
static uint32_t adv;           /* frames the USB side has advanced while we were muted    */
static uint8_t  playing;       /* 1 = streaming, 0 = muted/waiting for data               */
static uint8_t  dmaRunning;    /* 1 once HAL_I2S_Transmit_DMA has been started            */

/* Watch these in the debugger (Live Expressions) */
volatile uint32_t dbgSkips;     /* frames dropped because USB was running ahead of I2S    */
volatile uint32_t dbgRepeats;   /* frames repeated because USB was running behind         */
volatile uint32_t dbgUnderruns; /* times the stream ran dry and we muted                  */
volatile uint32_t dbgOverruns;  /* times we had to hard-resync because the buffer filled  */
volatile uint32_t dbgFill;      /* last measured unread frames in class buffer            */


/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_IF_Private_FunctionPrototypes USBD_AUDIO_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_FS(uint32_t options);
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol);
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_GetState_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static void fill_half(uint32_t half);

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_FS =
{
  AUDIO_Init_FS,
  AUDIO_DeInit_FS,
  AUDIO_AudioCmd_FS,
  AUDIO_VolumeCtl_FS,
  AUDIO_MuteCtl_FS,
  AUDIO_PeriodicTC_FS,
  AUDIO_GetState_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the AUDIO media low layer over USB FS IP
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
  /* USER CODE BEGIN 0 */
  UNUSED(AudioFreq);
  UNUSED(Volume);
  UNUSED(options);
  return (USBD_OK);
  /* USER CODE END 0 */
}

/**
  * @brief  De-Initializes the AUDIO media low layer
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_DeInit_FS(uint32_t options)
{
  /* USER CODE BEGIN 1 */
  UNUSED(options);
  return (USBD_OK);
  /* USER CODE END 1 */
}

/**
  * @brief  Handles AUDIO command.
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 2 */
  UNUSED(pbuf);
  UNUSED(size);

  switch(cmd)
  {
    case AUDIO_CMD_START:
    {
      /* Called by the class when its buffer has been filled once (80 ms of audio).
         The class buffer is NOT given to the DMA any more: we read it ourselves. */
      USBD_AUDIO_HandleTypeDef *h =
          (USBD_AUDIO_HandleTypeDef *)hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];
      if (h == NULL) break;

      uint32_t wr = (*(volatile uint16_t *)&h->wr_ptr) / BYTES_PER_FRAME;

      /* Start reading 40 ms behind the write position = middle of the buffer */
      rd      = (wr + CB_FRAMES - TARGET_FILL) % CB_FRAMES;
      lastWr  = wr;
      adv     = 0U;
      playing = 1U;

      /* Prime both halves of our DMA buffer, then start (once only) */
      EQ_Init(48000.0f);
      fill_half(0U);
      fill_half(1U);

      if (!dmaRunning)
      {
        if (HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)dmaBuf, DMA_FRAMES * 2U) == HAL_OK)
        {
          dmaRunning = 1U;
        }
      }
      break;
    }

    case AUDIO_CMD_PLAY:
      /* Not used: the class's own read pointer logic is bypassed (see callbacks) */
      break;
  }
  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Controls AUDIO Volume.
  * @param  vol: volume level (0..100)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol)
{
  /* USER CODE BEGIN 3 */
  UNUSED(vol);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  Controls AUDIO Mute.
  * @param  cmd: command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd)
{
  /* USER CODE BEGIN 4 */
  UNUSED(cmd);
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  AUDIO_PeriodicT_FS
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Gets AUDIO State.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_GetState_FS(void)
{
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  Manages the DMA full transfer complete event.
  * @retval None
  */
void TransferComplete_CallBack_FS(void)
{
  /* USER CODE BEGIN 7 */
  /* DMA just finished the 2nd half and wrapped to the 1st: refill the 2nd half */
  fill_half(1U);
  /* USER CODE END 7 */
}

/**
  * @brief  Manages the DMA Half transfer complete event.
  * @retval None
  */
void HalfTransfer_CallBack_FS(void)
{
  /* USER CODE BEGIN 8 */
  /* DMA just finished the 1st half and moved to the 2nd: refill the 1st half */
  fill_half(0U);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @brief  Fills one half of the DMA buffer from the class buffer, keeping the
  *         two clocks matched by dropping or repeating a single stereo frame
  *         when the USB side drifts too far ahead or behind.
  *         Runs in the DMA interrupt (same priority as the USB interrupt, so
  *         they cannot pre-empt each other).
  * @param  half: 0 = first half of dmaBuf, 1 = second half
  */
static void fill_half(uint32_t half)
{
  int16_t *dst = &dmaBuf[half * HALF_FRAMES * 2U];

  USBD_AUDIO_HandleTypeDef *h =
      (USBD_AUDIO_HandleTypeDef *)hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];

  /* USB class not initialised (host disconnected / reconfigured): silence */
  if (h == NULL)
  {
    memset(dst, 0, HALF_FRAMES * BYTES_PER_FRAME);
    playing = 0U;
    adv = 0U;
    return;
  }

  uint32_t wr   = (*(volatile uint16_t *)&h->wr_ptr) / BYTES_PER_FRAME;
  uint32_t fill = (wr + CB_FRAMES - rd) % CB_FRAMES;   /* unread frames */

  if (!playing)
  {
    /* Muted: wait until the host has delivered a fresh TARGET_FILL frames */
    uint32_t d = (wr + CB_FRAMES - lastWr) % CB_FRAMES;
    lastWr = wr;
    adv = (d != 0U) ? (adv + d) : 0U;

    if (adv >= TARGET_FILL)
    {
      rd      = (wr + CB_FRAMES - TARGET_FILL) % CB_FRAMES;
      fill    = TARGET_FILL;
      playing = 1U;
    }
    else
    {
      memset(dst, 0, HALF_FRAMES * BYTES_PER_FRAME);
      return;
    }
  }

  /* Stream dried up (host stopped sending): mute instead of looping old audio */
  if (fill < (HALF_FRAMES + FILL_GUARD))
  {
    dbgUnderruns++;
    playing = 0U;
    adv     = 0U;
    lastWr  = wr;
    memset(dst, 0, HALF_FRAMES * BYTES_PER_FRAME);
    return;
  }

  /* Far too full (something stalled us): hard resync to the middle */
  if (fill > (CB_FRAMES - HALF_FRAMES - FILL_GUARD))
  {
    dbgOverruns++;
    rd   = (wr + CB_FRAMES - TARGET_FILL) % CB_FRAMES;
    fill = TARGET_FILL;
  }

  dbgFill = fill;

  /* Clock drift correction: move the read pointer one frame extra or one frame short */
  int32_t adj = 0;
  if (fill > (TARGET_FILL + FILL_TOL))
  {
    adj = 1;                 /* USB ahead of I2S -> skip one frame */
    dbgSkips++;
  }
  else if (fill < (TARGET_FILL - FILL_TOL))
  {
    adj = -1;                /* USB behind I2S -> repeat one frame */
    dbgRepeats++;
  }

  /* Copy HALF_FRAMES frames, handling wrap-around of the class buffer */
  const int16_t *src = (const int16_t *)h->buffer;
  uint32_t first = CB_FRAMES - rd;
  if (first > HALF_FRAMES) first = HALF_FRAMES;

  memcpy(dst, &src[rd * 2U], first * BYTES_PER_FRAME);
  if (first < HALF_FRAMES)
  {
    memcpy(&dst[first * 2U], src, (HALF_FRAMES - first) * BYTES_PER_FRAME);
  }

  rd = (uint32_t)((int32_t)rd + (int32_t)HALF_FRAMES + adj + (int32_t)CB_FRAMES) % CB_FRAMES;

  /* ---- EQ HOOK ----
   * Process 'dst' in place here (HALF_FRAMES stereo frames, interleaved L,R int16)
     once the pass-through has been confirmed clean. */
     EQ_Process(dst, HALF_FRAMES);
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
