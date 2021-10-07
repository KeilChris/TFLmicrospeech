/*
 * Copyright (c) 2021 Arm Limited. All rights reserved.
 */

#include <stddef.h>
#include "audio_drv.h"
#include "arm_vsi.h"
#ifdef _RTE_
#include "RTE_Components.h"
#endif
#include CMSIS_device_header

/* Audio Peripheral definitions */
#define AudioIn         ARM_VSI0                /* Audio Input access struct */
#define AudioIn_IRQn    ARM_VSI0_IRQn           /* Audio Input Interrupt number */
#define AudioIn_Handler ARM_VSI0_Handler        /* Audio Input Interrupt handler */

/* Audio Peripheral registers */
#define CONTROL         Regs[0] /* Control receiver */
#define CHANNELS        Regs[1] /* Number of channels */
#define SAMPLE_BITS     Regs[2] /* Sample number of bits (8..32) */
#define SAMPLE_RATE     Regs[3] /* Sample rate (samples per second) */

/* Audio Control register definitions */
#define CONTROL_ENABLE_Pos      0U                              /* CONTROL: ENABLE Position */
#define CONTROL_ENABLE_Msk      (1UL << CONTROL_ENABLE_Pos)     /* CONTROL: ENABLE Mask */

/* Driver State */
static uint8_t Initialized = 0U;

/* Event Callback */
static AudioDrv_Event_t CB_Event = NULL;

/* Audio Input Interrupt Handler */
void AudioIn_Handler (void) {

  AudioIn->IRQ.Clear = 0x00000001U;
  __ISB();
  __DSB();
  if (CB_Event != NULL) {
    CB_Event(AUDIO_DRV_EVENT_RX_DATA);
  }
}

/* Initialize Audio Interface */
int32_t AudioDrv_Initialize (AudioDrv_Event_t cb_event) {

  CB_Event = cb_event;

  AudioIn->Timer.Control = 0U;
  AudioIn->DMA.Control   = 0U;
  AudioIn->IRQ.Clear     = 0x00000001U;
  AudioIn->IRQ.Enable    = 0x00000001U;
  AudioIn->CONTROL       = 0U;

//NVIC_EnableIRQ(AudioIn_IRQn);
  NVIC->ISER[(((uint32_t)AudioIn_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)AudioIn_IRQn) & 0x1FUL));
  __ISB();
  __DSB();

  Initialized = 1U;

  return AUDIO_DRV_OK;
}

/* De-initialize Audio Interface */
int32_t AudioDrv_Uninitialize (void) {

//NVIC_DisableIRQ(AudioIn_IRQn);
  NVIC->ICER[(((uint32_t)AudioIn_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)AudioIn_IRQn) & 0x1FUL));
  __DSB();
  __ISB();

  AudioIn->Timer.Control = 0U;
  AudioIn->DMA.Control   = 0U;
  AudioIn->IRQ.Clear     = 0x00000001U;
  AudioIn->IRQ.Enable    = 0x00000000U;
  AudioIn->CONTROL       = 0U;

  Initialized = 0U;

  return AUDIO_DRV_OK;
}

/* Configure Audio Interface */
int32_t AudioDrv_Configure (uint32_t interface, uint32_t channels, uint32_t sample_bits, uint32_t sample_rate) {
  uint32_t format;

  if (Initialized == 0U) {
    return AUDIO_DRV_ERROR;
  }

  if ((channels <  1U) ||
      (channels > 32U) ||
      (sample_bits <  8U) ||
      (sample_bits > 32U) ||
      (sample_rate == 0U)) {
    return AUDIO_DRV_ERROR_PARAMETER;
  }

  switch (interface) {
    case AUDIO_DRV_INTERFACE_TX:
      return AUDIO_DRV_ERROR_UNSUPPORTED;
      break;
    case AUDIO_DRV_INTERFACE_RX:
      if ((AudioIn->CONTROL & CONTROL_ENABLE_Msk) != 0U) {
        return AUDIO_DRV_ERROR;
      }
      AudioIn->CHANNELS    = channels;
      AudioIn->SAMPLE_BITS = sample_bits;
      AudioIn->SAMPLE_RATE = sample_rate;
      break;
    default:
      return AUDIO_DRV_ERROR_PARAMETER;
  }

  return AUDIO_DRV_OK;
}

/* Set Audio Interface buffer */
int32_t AudioDrv_SetBuf (uint32_t interface, void *buf, uint32_t block_num, uint32_t block_size) {

  if (Initialized == 0U) {
    return AUDIO_DRV_ERROR;
  }

  switch (interface) {
    case AUDIO_DRV_INTERFACE_TX:
      return AUDIO_DRV_ERROR_UNSUPPORTED;
      break;
    case AUDIO_DRV_INTERFACE_RX:
      if ((AudioIn->DMA.Control & ARM_VSI_DMA_Enable_Msk) != 0U) {
        return AUDIO_DRV_ERROR;
      }
      AudioIn->DMA.Address   = (uint32_t)buf;
      AudioIn->DMA.BlockNum  = block_num;
      AudioIn->DMA.BlockSize = block_size;
      break;
    default:
      return AUDIO_DRV_ERROR_PARAMETER;
  }

  return AUDIO_DRV_OK;
}

/* Control Audio Interface */
int32_t AudioDrv_Control (uint32_t control) {
  uint32_t sample_size;
  uint32_t sample_rate;
  uint32_t block_size;

  if (Initialized == 0U) {
    return AUDIO_DRV_ERROR;
  }

//if ((control & AUDIO_DRV_CONTROL_TX_DISABLE) != 0U) {
//} else if ((control & AUDIO_DRV_CONTROL_TX_ENABLE) != 0U) {
//}

  if ((control & AUDIO_DRV_CONTROL_RX_DISABLE) != 0U) {
    AudioIn->Timer.Control = 0U;
    AudioIn->DMA.Control   = 0U;
    AudioIn->CONTROL       = 0U;
  } else if ((control & AUDIO_DRV_CONTROL_RX_ENABLE) != 0U) {
    AudioIn->CONTROL       = CONTROL_ENABLE_Msk;
    AudioIn->DMA.Control   = ARM_VSI_DMA_Direction_P2M |
                             ARM_VSI_DMA_Enable_Msk;
    sample_size = AudioIn->CHANNELS * ((AudioIn->SAMPLE_BITS + 7U) / 8U);
    sample_rate = AudioIn->SAMPLE_RATE;
    if ((sample_size == 0U) || (sample_rate == 0U)) {
      AudioIn->Timer.Interval = 0xFFFFFFFFU;
    } else {
      block_size = AudioIn->DMA.BlockSize;
      AudioIn->Timer.Interval = (1000000U * (block_size / sample_size)) / sample_rate;
    }
    AudioIn->Timer.Control = ARM_VSI_Timer_Trig_DMA_Msk |
                             ARM_VSI_Timer_Trig_IRQ_Msk |
                             ARM_VSI_Timer_Periodic_Msk |
                             ARM_VSI_Timer_Run_Msk;
  }

  return AUDIO_DRV_OK;
}

/* Get transmitted block count */
uint32_t AudioDrv_GetTxCount (void) {
  return (0UL);  // Unsupported
}

/* Get received block count */
uint32_t AudioDrv_GetRxCount (void) {
  return (AudioIn->Timer.Count);
}

/* Get Audio Interface status */
AudioDrv_Status_t AudioDrv_GetStatus (void) {
  AudioDrv_Status_t status;
  uint32_t sr;

  status.tx_active = 0U;  // Unsupported

  if ((AudioIn->CONTROL & CONTROL_ENABLE_Msk) != 0U) {
    status.rx_active = 1U;
  } else {
    status.rx_active = 0U;
  }

  return (status);
}
