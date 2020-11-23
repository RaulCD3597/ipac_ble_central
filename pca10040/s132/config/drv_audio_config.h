/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DRV_AUDIO_CONFIG_H
#define __DRV_AUDIO_CONFIG_H

#include "app_util.h"
#include "pca10040.h"

// Se agregan para coincidencia en modulo thingy
// Son los pines a los que se debe conectar el microfono.
#define MIC_DOUT                 25
#define MIC_CLK                  26
#define MIC_PWR_CTRL             27

#define IS_IO_VALID(io)             (((io) & ~0x1F) == 0)

/* Microphone configuration */

// PDM microphones:
#define CONFIG_PDM_MIC_LEFT         1
#define CONFIG_PDM_MIC_RIGHT        2

// <<< Use Configuration Wizard in Context Menu >>>

// <h> Microphone configuration
// <i> This section contains the configuration settings for all Thingy nRF52 features.

// <e> Enable Audio Support
// <i> Enable the audio subsystem. With this subsystem enabled, the remote will transmit voice commands to the host over Bluetooth.
#define CONFIG_AUDIO_ENABLED 1

// <q> Enable Gain Control
// <i> Enable software gain control. The gain is defined in the drv_audio_dsp.c file. It is recommended to use hardware gain control in the PDM configuration.
#define CONFIG_AUDIO_GAIN_CONTROL_ENABLED 0

// <h> Pulse-Density Modulation (PDM) Interface Configuration
// <i> Set the PDM interface configuration settings.
// <o> PDM Microphone
// <i> If ANR is enabled, select the PDM microphone that will be used to pick up voice. The other microphone will be used to pick up background noise.
//     <1=>Left
//     <2=>Right
#define CONFIG_PDM_MIC 1

#define CONFIG_PDM_GAIN 0x40

// <o> PDM Microphone Transient State Length [ms] <1-1000>
// <i> Set the length of the PDM microphone transient state. In this state, the microphone produces invalid data after wakeup or after it is powered on.
#define CONFIG_PDM_TRANSIENT_STATE_LEN 500

// <q> Enable PDM Microphone Power Control
// <i> Allow for powering down the microphone when audio streaming is not active. Remember to configure a PDM Microphone Power Control Pin in the I/O Configuration.
#define CONFIG_PDM_MIC_PWR_CTRL_ENABLED 0
// </h>
// </e>

// <h> PDM Interface
// <i> Configure the PDM interface.

// <o> PDM CLK Pin
//     <0=>P0.0   <1=>P0.1   <2=>P0.2   <3=>P0.3   <4=>P0.4   <5=>P0.5   <6=>P0.6   <7=>P0.7
//     <8=>P0.8   <9=>P0.9   <10=>P0.10 <11=>P0.11 <12=>P0.12 <13=>P0.13 <14=>P0.14 <15=>P0.15
//     <16=>P0.16 <17=>P0.17 <18=>P0.18 <19=>P0.19 <20=>P0.20 <21=>P0.21 <22=>P0.22 <23=>P0.23
//     <24=>P0.24 <25=>P0.25 <26=>P0.26 <27=>P0.27 <28=>P0.28 <29=>P0.29 <30=>P0.30 <31=>P0.31
//     <0xFFFFFFFF=>Disable
#define CONFIG_IO_PDM_CLK MIC_CLK

// <o> PDM DATA Pin
//     <0=>P0.0   <1=>P0.1   <2=>P0.2   <3=>P0.3   <4=>P0.4   <5=>P0.5   <6=>P0.6   <7=>P0.7
//     <8=>P0.8   <9=>P0.9   <10=>P0.10 <11=>P0.11 <12=>P0.12 <13=>P0.13 <14=>P0.14 <15=>P0.15
//     <16=>P0.16 <17=>P0.17 <18=>P0.18 <19=>P0.19 <20=>P0.20 <21=>P0.21 <22=>P0.22 <23=>P0.23
//     <24=>P0.24 <25=>P0.25 <26=>P0.26 <27=>P0.27 <28=>P0.28 <29=>P0.29 <30=>P0.30 <31=>P0.31
//     <0xFFFFFFFF=>Disable
#define CONFIG_IO_PDM_DATA MIC_DOUT

// <o> PDM Microphone Power Control Pin
//     <0=>P0.0   <1=>P0.1   <2=>P0.2   <3=>P0.3   <4=>P0.4   <5=>P0.5   <6=>P0.6   <7=>P0.7
//     <8=>P0.8   <9=>P0.9   <10=>P0.10 <11=>P0.11 <12=>P0.12 <13=>P0.13 <14=>P0.14 <15=>P0.15
//     <16=>P0.16 <17=>P0.17 <18=>P0.18 <19=>P0.19 <20=>P0.20 <21=>P0.21 <22=>P0.22 <23=>P0.23
//     <24=>P0.24 <25=>P0.25 <26=>P0.26 <27=>P0.27 <28=>P0.28 <29=>P0.29 <30=>P0.30 <31=>P0.31
//     <0xFFFFFFFF=>Disable
#define CONFIG_IO_PDM_MIC_PWR_CTRL MIC_PWR_CTRL

// <q> PDM Microphone Power Control Pin Active Low
// <i> Select this option if PDM Microphone Power Control signal is active-low.
#define CONFIG_PDM_MIC_PWR_CTRL_ACT_LOW 0

// Audio parameters:
#define CONFIG_AUDIO_FRAME_SIZE_SAMPLES      80
# define CONFIG_AUDIO_FRAME_SIZE_BYTES       (CONFIG_AUDIO_FRAME_SIZE_SAMPLES * 2)

# define CONFIG_PDM_BUFFER_SIZE_SAMPLES (1 * CONFIG_AUDIO_FRAME_SIZE_SAMPLES)

#endif /* __DRV_AUDIO_CONFIG_H */
