/*
 * Copyright 2021-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "audio.h"
#include "audio_data.h"
#include "demo_config.h"
#include "eiq_audio_worker.h"
#include "fsl_debug_console.h"
#include "kws_mfcc.hpp"

static EIQ_AudioWorker_t* s_worker = NULL;
/* Recording window is one frame */
const int kRecordingWin = 1;
static KWS_MFCC s_kws(kRecordingWin);
static int s_sampleCount = 0;
const int16_t *staticData;

void set_audio_data(int16_t * _audio_data)
{
    staticData = _audio_data;
}

status_t AUDIO_GetSpectralSample(uint8_t* dstData, size_t size)
{
    /* Audio buffer size must be a multiple of audio block size.
       Otherwise the remaining non-complete part of the buffer will not be processed. */
    assert(SAMP_FREQ % s_kws.audio_block_size == 0);

    AUDIO_PreprocessSample(staticData, NUM_FRAMES);
    s_kws.store_features(dstData);
    return kStatus_Success;
}

void AUDIO_PreprocessSample(const int16_t* srcData, size_t audioBlocksPerBuffer)
{
    for (int i = 0; i < audioBlocksPerBuffer; i++)
    {
        /* Redirect the source for MFCC extraction to the shifted chunks of frame length */
        s_kws.audio_buffer = srcData + i * s_kws.audio_block_size;
        s_kws.extract_features();
    }
}

