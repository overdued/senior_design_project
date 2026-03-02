#ifndef _RUN_KWS_MODEL_DEMO_h_
#define _RUN_KWS_MODEL_DEMO_h_

#include "zf_common_headfile.h"
#include "model.h"
#include "audio.h"
#include "audio_data.h"
#include "timer.h"
#include "output_postproc.h"
#include "demo_config.h"


extern uint8 audio_data_get_finish;

void uart_dma_init(void);
void uart_dma_callback(void);
void tflite_model_init(void);
void audio_wifi_init(void);
int audio_predict(void);
int audio_predict_use_data(void);
#endif