#include "zf_common_headfile.h"
#include "run_kws_model_demo.h"
#include "math.h"
#include <stdlib.h>
#include <string.h>
#include "user_function.h"
#include "JY901.h"

static uint8_t RxBuffer[11];
static volatile uint8_t Rxstate = 0;
static uint8_t Rxindex = 0;

volatile float roll = 0.0f;
volatile float pitch = 0.0f;
volatile float yaw = 0.0f;

// ------------------- 基准角度保存 -------------------
static float base_roll = 0.0f;
static float base_pitch = 0.0f;
static float base_yaw = 0.0f;
static uint8_t base_set = 0;   // 是否已经设置基准

// ------------------- 工具函数：角度归一化 [-180,180] -------------------
static float normalize_angle(float angle)
{
    if (angle > 180.0f)  angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

// ------------------- JY901 串口数据接收与解析 -------------------
void jy901_receivedata(uint8_t Rxdata)
{
    uint8_t i, sum = 0;
    if (Rxstate == 0)
    {
        if (Rxdata == 0x55)
        {
            RxBuffer[Rxindex] = Rxdata;
            Rxstate = 1;
            Rxindex = 1;
        }
    }
    else if (Rxstate == 1)
    {
        if (Rxdata == 0x53)
        {
            RxBuffer[Rxindex] = Rxdata;
            Rxstate = 2;
            Rxindex = 2;
        }
        else
        {
            // 错帧，重新等待 0x55
            Rxstate = 0;
            Rxindex = 0;
        }
    }
    else if (Rxstate == 2)
    {
        RxBuffer[Rxindex++] = Rxdata;
        if (Rxindex == 11)
        {
            for (i = 0; i < 10; i++)
            {
                sum += RxBuffer[i];
            }

            if (sum == RxBuffer[10])
            {
                float raw_roll, raw_pitch, raw_yaw;

                raw_roll  = ((int16_t)((int16_t)RxBuffer[3] << 8 | (int16_t)RxBuffer[2])) / 32768.0f * 180.0f;
                raw_pitch = ((int16_t)((int16_t)RxBuffer[5] << 8 | (int16_t)RxBuffer[4])) / 32768.0f * 180.0f;
                raw_yaw   = ((int16_t)((int16_t)RxBuffer[7] << 8 | (int16_t)RxBuffer[6])) / 32768.0f * 180.0f;

                // 第一次获取数据时，记录基准角度
                if (!base_set)
                {
                    base_roll  = raw_roll;
                    base_pitch = raw_pitch;
                    base_yaw   = raw_yaw;
                    base_set   = 1;
                }

                // 减去基准，得到相对角度
                roll  = normalize_angle(raw_roll  - base_roll);
                pitch = normalize_angle(raw_pitch - base_pitch);
                yaw   = normalize_angle(raw_yaw   - base_yaw);
            }

            // 重置状态机
            Rxstate = 0;
            Rxindex = 0;
        }
    }
}
