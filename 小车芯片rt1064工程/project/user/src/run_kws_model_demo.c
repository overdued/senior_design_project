#include "run_kws_model_demo.h"

#define KWS_LPUART                      LPUART8 
#define LPUART_RX_DMA_CHANNEL           1U                          //UART接收使用的DMA通道号
#define LPUART_RX_DMA_REQUEST           kDmaRequestMuxLPUART8Rx     //定义串口DMA接收请求源
#define LPUART_DMAMUX_BASEADDR          DMAMUX                      //定义所使用的DMA多路复用模块(DMAMUX)
#define LPUART_DMA_BASEADDR             DMA0                        //定义使用的DMA
#define KWS_DATA_LENGTH                 16000                       //UART接收和发送数据缓冲区长度

lpuart_edma_handle_t g_lpuartEdmaHandle;  //串口DMA传输句柄
edma_handle_t g_lpuartTxEdmaHandle;       //串口DMA发送句柄
edma_handle_t g_lpuartRxEdmaHandle;       //串口DMA接收句柄
lpuart_transfer_t g_sendXfer;             //定义发送传输结构体
lpuart_transfer_t g_receiveXfer;          //定义接收传输结构体
uint32_t g_bufflength = 0;

/* 收发缓冲区 */
AT_SDRAM_SECTION_ALIGN(int16_t audio_data[KWS_DATA_LENGTH], 4);

uint32_t get_data_lenth = 0;
uint8 audio_data_get_finish = 0;

tensor_dims_t inputDims;
tensor_type_t inputType;
tensor_dims_t outputDims;
tensor_type_t outputType;
uint8_t* inputData, *outputData;


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口DMA初始化
// 参数说明     void
// 使用示例     uart_dma_init();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
void uart_dma_init(void)
{
  edma_config_t config;
  
  DMAMUX_Init(LPUART_DMAMUX_BASEADDR);
  DMAMUX_SetSource(LPUART_DMAMUX_BASEADDR, LPUART_RX_DMA_CHANNEL, LPUART_RX_DMA_REQUEST);
  DMAMUX_EnableChannel(LPUART_DMAMUX_BASEADDR, LPUART_RX_DMA_CHANNEL);
  
  EDMA_GetDefaultConfig(&config);
  EDMA_Init(LPUART_DMA_BASEADDR, &config);
  EDMA_CreateHandle(&g_lpuartRxEdmaHandle, LPUART_DMA_BASEADDR, LPUART_RX_DMA_CHANNEL);
  
  LPUART_TransferCreateHandleEDMA(KWS_LPUART, &g_lpuartEdmaHandle, NULL, NULL, &g_lpuartTxEdmaHandle, &g_lpuartRxEdmaHandle);
  
  g_receiveXfer.data = (uint8*)audio_data;
  g_receiveXfer.dataSize = KWS_DATA_LENGTH * 2 + 1;
  LPUART_ReceiveEDMA(KWS_LPUART, &g_lpuartEdmaHandle, &g_receiveXfer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口DMA回调函数，用于接收数据
// 参数说明     void
// 使用示例     uart_dma_callback();
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
void uart_dma_callback(void)
{
    LPUART_ClearStatusFlags(KWS_LPUART, kLPUART_IdleLineFlag);
    DCACHE_CleanInvalidateByRange((uint32_t)audio_data, KWS_DATA_LENGTH * 2);  
    LPUART_TransferGetReceiveCountEDMA(KWS_LPUART, &g_lpuartEdmaHandle, (uint32_t*)&g_bufflength);
    LPUART_TransferAbortReceiveEDMA(KWS_LPUART, &g_lpuartEdmaHandle);

    //记录实际数据长度
    get_data_lenth += (g_bufflength) / 2;
    
    //判断帧头
    if( get_data_lenth >= 4 &&
        (uint16)audio_data[get_data_lenth - 4] == 0xa5a5    &&
        (uint16)audio_data[get_data_lenth - 3] == 0         &&
        (uint16)audio_data[get_data_lenth - 2] == 0         &&
        (uint16)audio_data[get_data_lenth - 1] == 0x007d )
    {
        get_data_lenth = 0;
        memset(audio_data, 0, sizeof(audio_data));
    }
    
    //如果数据接收完成则audio_data_get_finish置1
    if(get_data_lenth == KWS_DATA_LENGTH)
    {
        audio_data_get_finish = 1;
    }
    
    //如果get_data_lenth超过最大值则将长度等于0
    if(get_data_lenth > KWS_DATA_LENGTH)get_data_lenth = 0;
    
    //继续启动DMA接收
    g_receiveXfer.data = (uint8*)audio_data + get_data_lenth * 2;
    g_receiveXfer.dataSize = KWS_DATA_LENGTH * 2 + 1;
    LPUART_ReceiveEDMA(KWS_LPUART, &g_lpuartEdmaHandle, &g_receiveXfer);
    
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     tflite模型初始化
// 参数说明     void
// 使用示例     tflite_model_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void tflite_model_init(void)
{
    //初始化model
    if (MODEL_Init() != kStatus_Success)
    {
        //说明模型文件异常
        PRINTF("Failed initializing model" EOL);
        while(1);
    }
    
    inputData = MODEL_GetInputTensorData(&inputDims, &inputType);
    outputData = MODEL_GetOutputTensorData(&outputDims, &outputType);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化wifi转串口
// 参数说明     void
// 使用示例     audio_wifi_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void audio_wifi_init(void)
{
	interrupt_global_disable();
	gpio_init(WIFI_UART_RTS_PIN, GPI, 0, GPI_PULL_UP);                          // 初始化流控引脚
	gpio_init(WIFI_UART_RST_PIN, GPO, 1, GPO_PUSH_PULL);                        // 初始化复位引脚
	uart_init(WIFI_UART_INDEX, 1500000, WIFI_UART_RX_PIN, WIFI_UART_TX_PIN);    // 初始化WiFi模块所使用的串口
    //初始化wifi转串口接收
    uart_dma_init();
    uart_rx_idle_interrupt (WIFI_UART_INDEX, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     语音识别
// 参数说明     void
// 返回参数     int           语音标签值
// 使用示例     audio_predict();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
int audio_predict(void)
{
    int label_index;
    //将wifi接收的数据传入语音处理
    set_audio_data((int16_t*)&audio_data[0]);
    
    //语音预处理
    if (AUDIO_GetSpectralSample(inputData, inputDims.data[1] * inputDims.data[2]) != kStatus_Success)
    {
        PRINTF("Failed retrieving input audio" EOL);
        while(1);
    }
    //模型运行
    MODEL_RunInference();
    
    //记录完成的值，开始的值在dma回调函数中记录

    //模型识别输出
    MODEL_ProcessOutput(outputData, &outputDims, outputType, 0, &label_index);
    
    //返回语音标签值
    return label_index;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使用固定数组语音识别
// 参数说明     void
// 返回参数     int           语音标签值
// 使用示例     audio_predict();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
int audio_predict_use_data(void)
{
    int label_index;
    //将固定数据传入语音处理
    set_audio_data((int16_t*)self_sample_data);
    
    //语音预处理
    if (AUDIO_GetSpectralSample(inputData, inputDims.data[1] * inputDims.data[2]) != kStatus_Success)
    {
        PRINTF("Failed retrieving input audio" EOL);
        while(1);
    }
    //模型运行
    MODEL_RunInference();
    
    //记录完成的值，开始的值在dma回调函数中记录

    //模型识别输出
    MODEL_ProcessOutput(outputData, &outputDims, outputType, 0, &label_index);
    
    //返回语音标签值
    return label_index;
}