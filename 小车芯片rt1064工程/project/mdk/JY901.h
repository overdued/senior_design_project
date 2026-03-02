
#ifndef __JY901_H__
#define __JY901_H__

#include "zf_driver_uart.h"


extern volatile float roll;
extern volatile float pitch;
extern volatile float yaw;
void jy901_receivedata(uint8_t Rxdata);
#endif