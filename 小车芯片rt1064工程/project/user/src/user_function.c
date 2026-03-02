#include "zf_common_headfile.h"
#include "user_function.h"
#include "math.h"
#define num_all (target_num + 2)
#define m 40
#define alpha 2
#define beta 4
#define iter_max 3000
#define Q 100
#define rou 0.5


//extern uint16 servo1_duty;
//extern uint16 servo2_duty;

////------------------------------------------------------------------------------------------------
//// 函数简介			舵机连续控制函数
//// 参数说明			_servo1_angle			舵机1的目标角度
//// 参数说明			_servo2_angle			舵机2的目标角度
//// 参数说明			_step_count				舵机连续控制间隔次数
//// 使用示例			servo_slow_ctrl(90, 90, 100);
//// 备注信息
////------------------------------------------------------------------------------------------------
//void servo_slow_ctrl(uint16 _servo1_angle, uint16 _servo2_angle, float _step_count)
//{
//    float servo1_start = (float)servo1_duty, servo2_start = (float)servo2_duty;
//    float servo1_step = (float)(_servo1_angle - servo1_duty) / _step_count, servo2_step = (float)(_servo2_angle - servo2_duty) / _step_count;
//    while (1)
//    {
//        system_delay_ms(5);
//        if (fabsf(servo1_start - (float)_servo1_angle) >= servo1_step)servo1_start += servo1_step;
//        else servo1_start = _servo1_angle;
//        pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_start));

//        if (fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)servo2_start += servo2_step;
//        else servo2_start = _servo2_angle;
//        pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start));

//        if (fabsf(servo1_start - (float)_servo1_angle) < 1 && fabsf(servo2_start - (float)_servo2_angle) < 1)
//        {
//            servo1_duty = (uint16)_servo1_angle;
//            servo2_duty = (uint16)_servo2_angle;
//            return;
//        }
//    }
//}


//------------------------------------------------------------------------------------------------
// 函数简介			获取图像
// 参数说明			gray_image													指向存储图像的首地址的指针
// 使用示例			get_image(img);
// 备注信息			可在while(1)循环里面设定一直显示图像，直到得到想要的图像发1，接收到信息后执行此函数
// 备注信息			ips200_show_gray_image(0,0,(const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, MT9V03X_W,MT9V03X_H,threshold);
//------------------------------------------------------------------------------------------------
void get_image(uint8 *gray_image, uint8 *input_image)
{
//	while(1)
//	{
//		if(mt9v03x_finish_flag)
//    {
//			image = (uint8 *)mt9v03x_image;
//			wireless_uart_send_string("get_image success!\n");
//			mt9v03x_finish_flag = 0;
//		}
//	}
	for (int i = 0; i < MT9V03X_H; i++)
		for (int j = 0; j < MT9V03X_W; j++)
			gray_image[i * MT9V03X_W + j] = input_image[i * MT9V03X_W + j];
}


//------------------------------------------------------------------------------------------------
// 函数简介			图像二值化
// 参数说明			binary_image											指向存储二值图像的首地址的指针
// 参数说明			threshold												二值化的门限值
// 参数说明			input_image												输入需要二值化的图像
// 使用示例			binary_image(binary_image， threshold);
// 备注信息			在main函数里要首先定义一个存储空间保存binary_image，建议使用malloc动态分配内存
//------------------------------------------------------------------------------------------------
void binarize_image(bool *binary_image, uint8 threshold, uint8 *input_image)
{
	for (int i = 0; i < MT9V03X_H; i++){
		for (int j = 0; j < MT9V03X_W; j++){
//			if (i < MT9V03X_H*3/4){
//				if (*(input_image + i * MT9V03X_W + j) < threshold)
//					binary_image[i * MT9V03X_W + j] = 0;
//				else
//					binary_image[i * MT9V03X_W + j] = 1;
//			}
//			else{
//				if (*(input_image + i * MT9V03X_W + j) < threshold - 24)
//					binary_image[i * MT9V03X_W + j] = 0;
//				else
//					binary_image[i * MT9V03X_W + j] = 1;			
//			}
			if (*(input_image + i * MT9V03X_W + j) < threshold)
				binary_image[i * MT9V03X_W + j] = 0;
			else
				binary_image[i * MT9V03X_W + j] = 1;
		}
	}
//	ips200_show_string(0, 280, "binary");
	wireless_uart_send_string("binarize success!\n");
}


//------------------------------------------------------------------------------------------------
// 函数简介			移动平均阈值处理
// 参数说明			binary_image											指向存储二值图像的首地址的指针
// 参数说明			input_image												输入需要二值化的图像
// 参数说明			N																	模板的大小 按经验可取为平均宽度的5倍
// 参数说明			c																	常量系数
// 使用示例			movingThreshold(binary_image, (uint8 *)mt9v03x_image, 20, 0.9);
// 备注信息			在main函数里要首先定义一个存储空间保存binary_image，建议使用malloc动态分配内存
//------------------------------------------------------------------------------------------------
void movingThreshold(bool *binary_image, uint8 *input_image, uint8 N, float c)
{
	int thre = 0;
	uint8 count = 0;
	int *gray = (int*)malloc(sizeof(int) * N);
	for (int i = 0; i < N; i++)
		gray[i] = 0;
	
	for (int k = 0; k <= MT9V03X_W+MT9V03X_H-2; k++){
		if (k % 2 == 1){
			for (int i = 0; i < MT9V03X_H; i++){
				for (int j = 0; j < MT9V03X_W; j++){
					if (i+j == k){
						if (count < N){
							gray[count] = input_image[ind(i,j)];
							count++;
						}
						else{
							for (int t = 0; t < N-1; t++)
									gray[t] = gray[t+1];
							gray[N-1] = input_image[ind(i,j)];
						}
						for (int t = 0; t < N; t++)
								thre += gray[t];
						thre /= N;
						if (input_image[ind(i,j)] >= c * thre)
							binary_image[ind(i,j)] = 1;
						else
							binary_image[ind(i,j)] = 0;
					}
				}
			}
		}
		else{
			for (int i = MT9V03X_H-1; i > 0; i--){
				for (int j = MT9V03X_W-1; j > 0; j--){
					if (i+j == k){
						if (count < N){
							gray[count] = input_image[ind(i,j)];
							count++;
						}
						else{
							for (int t = 0; t < N-1; t++)
									gray[t] = gray[t+1];
							gray[N-1] = input_image[ind(i,j)];
						}
						for (int t = 0; t < N; t++)
								thre += gray[t];
						thre /= N;
						if (input_image[ind(i,j)] >= c * thre)
							binary_image[ind(i,j)] = 1;
						else
							binary_image[ind(i,j)] = 0;
					}
				}
			}

		}
	}
//	wireless_uart_send_string("move_threshold success!\n");
}



//------------------------------------------------------------------------------------------------
// 函数简介			图像形态学细化
// 参数说明			binary_image											指向存储二值图像的首地址的指针
// 参数说明			I																	输出的细化图像
// 使用示例			refined_image(binary_image, I);
// 备注信息			在main函数里要首先定义一个存储空间保存I，赋初值为0即可
//------------------------------------------------------------------------------------------------
void refined_image(bool *binary_image, bool *I)
{
	bool temp[MT9V03X_W * MT9V03X_H] = { 0 };
	uint8 count = 0;
	for (int i = 0; i < MT9V03X_H; i++){
		for (int j = 0; j < MT9V03X_W; j++){
			I[ind(i,j)] = !binary_image[ind(i,j)];
		}
	}
	while(count < 3){
		// SE 1
		for (int i = 1; i < MT9V03X_H - 1; i++){
			for (int j = 1; j < MT9V03X_W - 1; j++){
				if (I[ind(i-1,j-1)]==0 && I[ind(i-1,j)]==0 && I[ind(i-1,j+1)]==0 && I[ind(i,j)]==1 && I[ind(i+1,j-1)]==1 && I[ind(i+1,j)]==1 && I[ind(i+1,j+1)]==1)
					temp[ind(i,j)] = 1;
			}
		}
		for (int i = 0; i < MT9V03X_H; i++){
			for (int j = 0; j < MT9V03X_W; j++){
				I[ind(i,j)] -= temp[ind(i,j)];
				temp[ind(i,j)] = 0;
			}
		}
		
		// SE 2
		for (int i = 1; i < MT9V03X_H - 1; i++){
			for (int j = 1; j < MT9V03X_W - 1; j++){
				if (I[ind(i-1,j)]==0 && I[ind(i-1,j+1)]==0 && I[ind(i,j+1)]==0 && I[ind(i,j)]==1 && I[ind(i,j-1)]==1 && I[ind(i+1,j-1)]==1 && I[ind(i+1,j)]==1)
					temp[ind(i,j)] = 1;
			}
		}
		for (int i = 0; i < MT9V03X_H; i++){
			for (int j = 0; j < MT9V03X_W; j++){
				I[ind(i,j)] -= temp[ind(i,j)];
				temp[ind(i,j)] = 0;
			}
		}

		// SE 3
		for (int i = 1; i < MT9V03X_H - 1; i++){
			for (int j = 1; j < MT9V03X_W - 1; j++){
				if (I[ind(i-1,j+1)]==0 && I[ind(i,j+1)]==0 && I[ind(i+1,j+1)]==0 && I[ind(i,j)]==1 && I[ind(i-1,j-1)]==1 && I[ind(i,j-1)]==1 && I[ind(i+1,j-1)]==1)
					temp[ind(i,j)] = 1;
			}
		}
		for (int i = 0; i < MT9V03X_H; i++){
			for (int j = 0; j < MT9V03X_W; j++){
				I[ind(i,j)] -= temp[ind(i,j)];
				temp[ind(i,j)] = 0;
			}
		}
		
		// SE 4
		for (int i = 1; i < MT9V03X_H - 1; i++){
			for (int j = 1; j < MT9V03X_W - 1; j++){
				if (I[ind(i,j+1)]==0 && I[ind(i+1,j+1)]==0 && I[ind(i+1,j)]==0 && I[ind(i,j)]==1 && I[ind(i,j-1)]==1 && I[ind(i-1,j-1)]==1 && I[ind(i-1,j)]==1)
					temp[ind(i,j)] = 1;
			}
		}
		for (int i = 0; i < MT9V03X_H; i++){
			for (int j = 0; j < MT9V03X_W; j++){
				I[ind(i,j)] -= temp[ind(i,j)];
				temp[ind(i,j)] = 0;
			}
		}
		
		// SE 5
		for (int i = 1; i < MT9V03X_H - 1; i++){
			for (int j = 1; j < MT9V03X_W - 1; j++){
				if (I[ind(i+1,j-1)]==0 && I[ind(i+1,j)]==0 && I[ind(i+1,j+1)]==0 && I[ind(i,j)]==1 && I[ind(i-1,j-1)]==1 && I[ind(i-1,j)]==1 && I[ind(i-1,j+1)]==1)
					temp[ind(i,j)] = 1;
			}
		}
		for (int i = 0; i < MT9V03X_H; i++){
			for (int j = 0; j < MT9V03X_W; j++){
				I[ind(i,j)] -= temp[ind(i,j)];
				temp[ind(i,j)] = 0;
			}
		}

		// SE 6
		for (int i = 1; i < MT9V03X_H - 1; i++){
			for (int j = 1; j < MT9V03X_W - 1; j++){
				if (I[ind(i,j-1)]==0 && I[ind(i+1,j-1)]==0 && I[ind(i+1,j)]==0 && I[ind(i,j)]==1 && I[ind(i,j+1)]==1 && I[ind(i-1,j)]==1 && I[ind(i-1,j+1)]==1)
					temp[ind(i,j)] = 1;
			}
		}
		for (int i = 0; i < MT9V03X_H; i++){
			for (int j = 0; j < MT9V03X_W; j++){
				I[ind(i,j)] -= temp[ind(i,j)];
				temp[ind(i,j)] = 0;
			}
		}

		// SE 7
		for (int i = 1; i < MT9V03X_H - 1; i++){
			for (int j = 1; j < MT9V03X_W - 1; j++){
				if (I[ind(i-1,j-1)]==0 && I[ind(i,j-1)]==0 && I[ind(i+1,j-1)]==0 && I[ind(i,j)]==1 && I[ind(i-1,j+1)]==1 && I[ind(i,j+1)]==1 && I[ind(i+1,j+1)]==1)
					temp[ind(i,j)] = 1;
			}
		}
		for (int i = 0; i < MT9V03X_H; i++){
			for (int j = 0; j < MT9V03X_W; j++){
				I[ind(i,j)] -= temp[ind(i,j)];
				temp[ind(i,j)] = 0;
			}
		}

		// SE 8
		for (int i = 1; i < MT9V03X_H - 1; i++){
			for (int j = 1; j < MT9V03X_W - 1; j++){
				if (I[ind(i-1,j-1)]==0 && I[ind(i-1,j)]==0 && I[ind(i,j-1)]==0 && I[ind(i,j)]==1 && I[ind(i,j+1)]==1 && I[ind(i+1,j)]==1 && I[ind(i+1,j+1)]==1)
					temp[ind(i,j)] = 1;
			}
		}
		for (int i = 0; i < MT9V03X_H; i++){
			for (int j = 0; j < MT9V03X_W; j++){
				I[ind(i,j)] -= temp[ind(i,j)];
				temp[ind(i,j)] = 0;
			}
		}

		count++;
	}
	
	for (int i = 0; i < MT9V03X_H; i++){
		for (int j = 0; j < MT9V03X_W; j++){
			I[ind(i,j)] = !I[ind(i,j)];
		}
	}
//	wireless_uart_send_string("refinement success!\n");
}



//------------------------------------------------------------------------------------------------
// 函数简介			Otsu阈值分割
// 参数说明			input_image												输入需要二值化的图像
// 参数说明			threshold												  输出的二值化的门限值
// 使用示例			threshold = otsu((uint8 *)mt9v03x_image);
// 备注信息			类似matlab的graythresh函数，用就行了
//------------------------------------------------------------------------------------------------
uint8 otsu(uint8 *input_image)
{
	float numPix = MT9V03X_W * MT9V03X_H;
	float gray_all = 0;
	float avr_all = 0;
	float grayA = 0;
	float grayB = 0;
	float numA = 0;
	float numB = 0;
	float PA = 0;
	float PB = 0;
	float avrA = 0;
	float avrB = 0;
	
	uint8 threshold = 0;
	float ICV_t = 0;
	float ICV = 0;
	
	for (int i = 0; i < MT9V03X_H; i++)
		for (int j = 0; j < MT9V03X_W; j++)
			gray_all += input_image[i * MT9V03X_W + j];
	avr_all = gray_all / numPix;
	
	for (int thre = 0; thre < 255; thre++){
		grayA = 0;
		grayB = 0;
		numA = 0;
		numB = 0;
		for (int i = 0; i < MT9V03X_H; i++){
			for (int j = 0; j < MT9V03X_W; j++){
				if (input_image[i * MT9V03X_W + j] >= thre){
					numA++;
					grayA += input_image[i * MT9V03X_W + j];
				}
				else{
					numB++;
					grayB += input_image[i * MT9V03X_W + j];
				}
			}
		}
		PA = numA / numPix;
		PB = numB / numPix;
		avrA = grayA / numA;
		avrB = grayB / numB;
		ICV = PA * pow((avrA - avr_all), 2) + PB * pow((avrB - avr_all), 2);
		if (ICV > ICV_t){
			ICV_t = ICV;
			threshold = thre;
		}
	}
//	wireless_uart_send_string("Otsu success!\n");
	return threshold;
}

//------------------------------------------------------------------------------------------------
// 函数简介			寻找边框的四个角点
// 参数说明			corner[4][2]										保存四个角点的数组
// 参数说明			binary_image										要查找角点的二值图像
// 参数说明			width												得到的实际场地的宽度
// 参数说明			height												得到的实际场地的高度
// 使用示例			find_corners(corner);
// 备注信息			在main函数里要首先定义一个存储空间保存corners，可以使用malloc动态分配内存
// 备注信息			其中corner[][0]存储行数，corner[][1]存储列数
// 备注信息			使用完可在main里显示图像，并用draw_line查看角点寻找是否正确
//------------------------------------------------------------------------------------------------
void find_corners(int corner[4][2], bool *binary_image)
{
	uint8 cnt = 0;
	uint8 maxnum = 0;
	
	// 清零
	for (int i = 0; i < 4; i++){
		corner[i][0] = 0;
		corner[i][1] = 1;
	}
	
// left & up
	for (int i = 5; i <= MT9V03X_H/5 - 5; i++)												// int i = 5; i <= MT9V03X_H/2 - 5; i++
	{
		for (int j = 5; j <= MT9V03X_W/4 - 5; j++)											// int j = 5; j <= MT9V03X_W/2 - 5; j++
		{
			if (binary_image[ind(i,j)] == 0 && binary_image[ind(i+1,j)] == 0 && binary_image[ind(i,j+1)] == 0 && binary_image[ind(i+1,j+1)] == 1){				// && binary_image[ind(i+1,j+1)] == 1
				cnt = 0;
				for (int u = -1; u<= 1; u++)
					for (int v = -4; v <= -2; v++){
						if (i+u >= 1 && j+v >= 1){
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 1)
								cnt++;
							else
								cnt--;
						}
					}
				for (int u = -4; u<= -2; u++)
					for (int v = -1; v <= 1; v++){
						if (i+u >= 1 && j+v >= 1){
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 1)
								cnt++;
							else
								cnt--;
						}
					}
				for (int u = -1; u<= 1; u++)
					for (int v = -1; v <= 1; v++)
						if (i+u >= 1 && j+v >= 1)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = 2; u<= 4; u++)
					for (int v = -1; v <= 1; v++)
						if (i+u >= 1 && j+v >= 1)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = -1; u<= 1; u++)
					for (int v = 2; v <= 4; v++)
						if (i+u >= 1 && j+v >= 1)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
							
				for (int v = 0; v < 10; v++)
						if (binary_image[i * MT9V03X_W + j+v] == 0)
							cnt++;
				for (int u = 0; u < 10; u++)
						if (binary_image[(i+u) * MT9V03X_W + j] == 0)
							cnt++;
				if (cnt > maxnum){
					corner[0][0] = i;
					corner[0][1] = j;
					maxnum = cnt;
				}
			}
		}
	}
	
// right & up
	maxnum = 0;
	for (int i = 1; i <= MT9V03X_H/5 - 5; i++)												// int i = 5; i <= MT9V03X_H/4 - 5; i++
	{
		for (int j = MT9V03X_W - 1; j >= MT9V03X_W*3/4 + 5; j--)				// int j = MT9V03X_W - 5; j >= MT9V03X_W/2 + 5; j--
		{
			if (binary_image[ind(i,j)] == 0 && binary_image[ind(i+1,j)] == 0 && binary_image[ind(i,j-1)] == 0  && binary_image[ind(i+1,j-1)] == 1){			//  && binary_image[ind(i+1,j-1)] == 1
				cnt = 0;
				for (int u = -1; u<= 1; u++)
					for (int v = -4; v <= -2; v++)
						if (i+u > 1 && j+v < MT9V03X_W)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = -4; u<= -2; u++)
					for (int v = -1; v <= 1; v++){
						if (i+u > 1 && j+v < MT9V03X_W){
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 1)
								cnt++;
							else
								cnt--;
						}
					}
				for (int u = -1; u<= 1; u++)
					for (int v = -1; v <= 1; v++)
						if (i+u > 1 && j+v < MT9V03X_W)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = 2; u<= 4; u++)
					for (int v = -1; v <= 1; v++)
						if (i+u > 1 && j+v < MT9V03X_W)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = -1; u<= 1; u++)
					for (int v = 2; v <= 4; v++){
						if (i+u > 1 && j+v < MT9V03X_W){
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 1)
								cnt++;
							else
								cnt--;
						}
					}
					
				for (int v = 0; v > -10; v--)
					if (binary_image[i * MT9V03X_W + j+v] == 0)
						cnt++;
				for (int u = 0; u < 10; u++)
					if (binary_image[(i+u) * MT9V03X_W + j] == 0)
						cnt++;
				if (cnt > maxnum){
					corner[1][0] = i;
					corner[1][1] = j;
					maxnum = cnt;
				}
			}
		}
	}
	
// right & down
	maxnum = 0;
	for (int i = MT9V03X_H - 1; i >= MT9V03X_H*4/5 + 5; i--)					// int i = MT9V03X_H - 5; i >= MT9V03X_H/2 + 5; i--
	{
		for (int j = MT9V03X_W - 1; j >= MT9V03X_W*3/4 + 5; j--)				// int j = MT9V03X_W - 5; j >= MT9V03X_W/2 + 5; j--
		{
			if (binary_image[ind(i,j)] == 0 && binary_image[ind(i-1,j)] == 0 && binary_image[ind(i,j-1)] == 0  && binary_image[ind(i-1,j-1)] == 1){			//  && binary_image[ind(i-1,j-1)] == 1
				cnt = 0;
				for (int u = -1; u<= 1; u++)
					for (int v = -4; v <= -2; v++)
						if (i+u < MT9V03X_H && j+v < MT9V03X_W)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = -4; u<= -2; u++)
					for (int v = -1; v <= 1; v++)
						if (i+u < MT9V03X_H && j+v < MT9V03X_W)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = -1; u<= 1; u++)
					for (int v = -1; v <= 1; v++)
						if (i+u < MT9V03X_H && j+v < MT9V03X_W)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = 2; u<= 4; u++)
					for (int v = -1; v <= 1; v++){
						if (i+u < MT9V03X_H && j+v < MT9V03X_W){
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 1)
								cnt++;
							else
								cnt--;
						}
					}
				for (int u = -1; u<= 1; u++)
					for (int v = 2; v <= 4; v++){
						if (i+u < MT9V03X_H && j+v < MT9V03X_W){
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 1)
								cnt++;
							else
								cnt--;
						}
					}
				
				for (int v = 0; v > -10; v--)
					if (binary_image[i * MT9V03X_W + j+v] == 0)
						cnt++;
				for (int u = 0; u > -10; u--)
					if (binary_image[(i+u) * MT9V03X_W + j] == 0)
						cnt++;
				if (cnt > maxnum){
					corner[3][0] = i;
					corner[3][1] = j;
					maxnum = cnt;
				}
			}
		}
	}
	
// left & down
	maxnum = 0;
	for (int i = MT9V03X_H - 1; i >= MT9V03X_H*4/5 + 1; i--)					// int i = MT9V03X_H - 5; i >= MT9V03X_H/2 + 5; i--
	{
		for (int j = 1; j <= MT9V03X_W/4 - 1; j++)											// int j = 5; j <= MT9V03X_W/2 - 5; j++
		{
			if (binary_image[ind(i,j)] == 0 && binary_image[ind(i-1,j)] == 0 && binary_image[ind(i,j+1)] == 0  && binary_image[ind(i-1,j+1)] == 1){			//  && binary_image[ind(i-1,j+1)] == 1
				cnt = 0;
				for (int u = -1; u<= 1; u++)
					for (int v = -4; v <= -2; v++){
						if (i+u < MT9V03X_H && j+v >= 1){
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 1)
								cnt++;
							else
								cnt--;
						}
					}
				for (int u = -4; u<= -2; u++)
					for (int v = -1; v <= 1; v++)
						if (i+u < MT9V03X_H && j+v >= 1)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = -1; u<= 1; u++)
					for (int v = -1; v <= 1; v++)
						if (i+u < MT9V03X_H && j+v >= 1)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				for (int u = 2; u<= 4; u++)
					for (int v = -1; v <= 1; v++){
						if (i+u < MT9V03X_H && j+v >= 1){
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 1)
								cnt++;
						}
					}
				for (int u = -1; u<= 1; u++)
					for (int v = 2; v <= 4; v++)
						if (i+u < MT9V03X_H && j+v >= 1)
							if (binary_image[(i+u) * MT9V03X_W + j+v] == 0)
								cnt++;
				
				for (int v = 0; v < 10; v++)
						if (binary_image[i * MT9V03X_W + j+v] == 0)
							cnt++;
				for (int u = 0; u > -10; u--)
						if (binary_image[(i+u) * MT9V03X_W + j] == 0)
							cnt++;
				if (cnt > maxnum){
					corner[2][0] = i;
					corner[2][1] = j;
					maxnum = cnt;
				}
			}
		}
	}

//	*width = (sqrt(pow(corner[0][0] - corner[1][0], 2) + pow(corner[0][1] - corner[1][1], 2)) + sqrt(pow(corner[2][0] - corner[3][0], 2) + pow(corner[2][1] - corner[3][1], 2))) / 2;
//	*height = (sqrt(pow(corner[0][0] - corner[2][0], 2) + pow(corner[0][1] - corner[2][1], 2)) + sqrt(pow(corner[1][0] - corner[3][0], 2) + pow(corner[1][1] - corner[3][1], 2))) / 2;
//	wireless_uart_send_string("find_corners success!\n");
	ips200_show_int(0,		280, corner[0][1], 3);
	ips200_show_int(30,		280, corner[0][0], 3);
	ips200_show_int(70,		280, corner[1][1], 3);
	ips200_show_int(100,	280, corner[1][0], 3);
	ips200_show_int(0,		280, corner[2][1], 3);
	ips200_show_int(30,		280, corner[2][0], 3);
	ips200_show_int(70,		280, corner[3][1], 3);
	ips200_show_int(100,	280, corner[3][0], 3);
}


//------------------------------------------------------------------------------------------------
// 函数简介			透视变换
// 参数说明			perspective_img									保存透视后图像首地址
// 参数说明			binary_image										输入的二值化图像
// 参数说明			corner[4][2]										保存四个角点的数组
// 参数说明			width														目标宽度
// 参数说明			height													目标高度
// 使用示例			perspective_tran(perspective_img, corner[4][2], width, height)
//
// 备注信息			在main函数里要首先定义一个存储空间保存perspective_img， 必须 使用malloc动态分配内存，无需初始化
// 备注信息			(uint8 *)malloc(sizeof(uint8) * (int)(width+0.5) * (int)(height+0.5));  一维数组
// 备注信息			使用完函数可在main里显示图像，如下例所示：
// 备注信息			ips200_show_gray_image(0, 150, (const uint8 *)img_p, (int)(width+0.5), (int)(height+0.5), (int)(width+0.5), (int)(height+0.5), threshold);
//------------------------------------------------------------------------------------------------
void perspective_tran(bool* perspective_img, bool *binary_image, int corner[4][2], float width, float height)
{
// 初始化内存
//	for (int i = 0; i < (int)(height+0.5); i++){
//		for (int j = 0; j < (int)(width+0.5); j++){
//			*(perspective_img + i * (int)(width+0.5) + j) = 0;
//		}
//	}
	uint8 bias = 0;
	int y[4] = {corner[0][1]+bias, corner[1][1]-bias, corner[2][1]+bias, corner[3][1]-bias};					// 加偏置使得边框尽量不出现在图像内
	int x[4] = {corner[0][0]+bias, corner[1][0]+bias, corner[2][0]-bias, corner[3][0]-bias};			// 针对下边框较粗的问题加入较大偏置
	int Y[4] = {0, (int)(width+0.5)-1, 0, (int)(width+0.5)-1};
	int X[4] = {0, 0, (int)(height+0.5)-1, (int)(height+0.5)-1};

	int A[8][8] = 
	{x[0], y[0], 1, 0, 0, 0, -X[0] * x[0], -X[0] * y[0],
	0, 0, 0, x[0], y[0], 1, -Y[0] * x[0], -Y[0] * y[0],
	x[1], y[1], 1, 0, 0, 0, -X[1] * x[1], -X[1] * y[1],
	0, 0, 0, x[1], y[1], 1, -Y[1] * x[1], -Y[1] * y[1],
	x[2], y[2], 1, 0, 0, 0, -X[2] * x[2], -X[2] * y[2],
	0, 0, 0, x[2], y[2], 1, -Y[2] * x[2], -Y[2] * y[2],
	x[3], y[3], 1, 0, 0, 0, -X[3] * x[3], -X[3] * y[3],
	0, 0, 0, x[3], y[3], 1, -Y[3] * x[3], -Y[3] * y[3]};

	double inv_A[8][8];
	Gauss8(A, inv_A);
	
//	double C[8] = {0};
	double **C = (double**)malloc(sizeof(double*) * 3);
	for (int i = 0; i < 3; i++){
		C[i] = (double*)malloc(sizeof(double) * 3);
		for (int j = 0; j < 3; j++)
		C[i][j] = 0;
	}
	C[2][2] = 1.0;
	
	int B[8] = {X[0], Y[0], X[1], Y[1], X[2], Y[2], X[3], Y[3]};  //
	for (int i = 0; i < 8; i++)
		for (int j = 0; j < 8; j++)
			C[i/3][i%3] += inv_A[i][j] * B[j];
	
// 动态初始化一个3×3的数组D
//	double **D = (double**)malloc(sizeof(double*) * 3);
//		for (int i = 0; i < 3; i++){
//			D[i] = (double*)malloc(sizeof(double) * 3);
//			for (int j = 0; j < 3; j++)
//				D[i][j] = 0;
//		}
//	
//	for (int i = 0; i < 3; i++){
//		for (int j = 0; j < 3; j++){
//			if (i == 2 && j == 2)
//				D[i][j] = 1.0;
//			else
//				D[i][j] = C[i*3 + j];
//		}
//	}

//	double **inv_D = Matrix_inver(D, 3, 3);
	double **inv_D = Matrix_inver(C, 3, 3);
//	wireless_uart_send_string("inver success!\n");
//	free(D);
	double pix[3] = { 0 };
	
	for (int i = 0; i < (int)(height+0.5); i++){
		for (int j = 0; j < (int)(width+0.5); j++){
			for (int u = 0; u < 3; u++){
				pix[u] = inv_D[u][0] * i + inv_D[u][1] * j + inv_D[u][2];
			}
			pix[0] /= pix[2];
			pix[1] /= pix[2];
			if (pix[0] < MT9V03X_H && pix[1] < MT9V03X_W){
//				if (binary_image[(int)pix[0] * MT9V03X_W + (int)pix[1]] == 1)
//					perspective_img[i * (int)(width+0.5) + j] = 1;
//				else
//					perspective_img[i * (int)(width+0.5) + j] = 0;
				perspective_img[i * (int)(width) + j] = binary_image[(int)pix[0] * MT9V03X_W + (int)pix[1]];
			}
			else
				perspective_img[i * (int)(width) + j] = 1;
		}
	}
	
//	wireless_uart_send_string("perspective success!\n");
}


//------------------------------------------------------------------------------------------------
// 函数简介			辅助判断透视变换结果是否正确
// 参数说明			perspective_img									保存透视后图像首地址
// 参数说明			corner[4][2]										保存四个角点的数组
// 参数说明			width														目标宽度
// 参数说明			height													目标高度
// 使用示例			perspect_correct = perspective_is_cor(perspective_image, corner, width, height);
//
// 备注信息			便利图像顶部与底部中为1的像素点，并计算他们的总数、平均数与标准差，并据此判断透视变换结果是否正确
// 备注信息			并根据具体情况对corner的右上or右下角点进行矫正，最大矫正次数通过.h文件中的MAX_ITERATE_NUM修改
//------------------------------------------------------------------------------------------------
bool perspective_is_cor(bool* perspective_img, int corner[4][2], float width, float height)
{
	double mean_top = 0.0;
	double mean_bot = 0.0;
	double sigma_top = 0.0;
	double sigma_bot = 0.0;
	int sum_top = 0;
	int sum_bot = 0;
	for (int j = 0; j < width; j++){
		for (int  i = 0; i < 5; i++){
			if (perspective_img[i * (int)width + j] == 0){
				mean_top += i;
				sum_top++;
			}
		}
		for (int i = (int)height-1; i > (int)height - 6; i--){
			if (perspective_img[i * (int)width + j] == 0){
				mean_bot += (int)height-1 - i;
				sum_bot++;
			}
		}
	}
	mean_top  = mean_top / sum_top;
	mean_bot = mean_bot / sum_bot;
	
	for (int j = 0; j < width; j++){
		for (int  i = 0; i < 5; i++){
			if (perspective_img[i * (int)width + j] == 0){
				sigma_top += pow((i - mean_top), 2);
			}
		}
		for (int i = (int)height-1; i > (int)height - 6; i--){
			if (perspective_img[i * (int)width + j] == 0){
				sigma_bot += pow(((int)height-1 - i - mean_bot), 2);
			}
		}
	}
	sigma_top = sqrt(sigma_top / sum_top);
	sigma_bot = sqrt(sigma_bot / sum_bot);

	
//	ips200_show_int(0, 280, sum_top, 3);
//	ips200_show_int(0, 300, sum_bot, 3);
//	ips200_show_float(40, 280, mean_top, 3, 3);
//	ips200_show_float(40, 300, mean_bot, 3, 3);
//	ips200_show_float(100, 280, sigma_top, 3, 3);
//	ips200_show_float(100, 300, sigma_bot, 3, 3);


	if (sum_top > 0.8 * (int)width && (mean_top > 1.0 || sigma_top > 0.65)){
		if (MAX_ITERATE_NUM > 1){
			corner[1][0] += 4;
			corner[3][0] += 4;
		}
//		ips200_show_string(180, 300, "false");
		return false;
	}
	if (sum_bot > 0.8 * (int)width && (mean_top > 1.0 || sigma_bot > 0.65)){
		if (MAX_ITERATE_NUM > 1){		
			corner[1][0] -= 4;
			corner[3][0] -= 4;
		}
//		ips200_show_string(180, 300, "false");
		return false;
	}
//	ips200_show_string(180, 300, "true");
	return true;
}


//------------------------------------------------------------------------------------------------
// 函数简介			种子算法
// 参数说明			target[20][2]										保存图像内目标点的坐标
// 参数说明			sum															保存图像内目标点的个数
// 参数说明			perspective_img								  透视后图像首地址
// 参数说明			width														目标宽度
// 参数说明			height												 	目标高度
// 使用示例			seed_function(target,  &sum, perspective_img, width, height)
//
// 备注信息			在main函数里要首先定义一个存储空间保存target，初始化为0
// 备注信息			运行完函数可在main里显示目标个数以及各点坐标
//------------------------------------------------------------------------------------------------
bool seed_function(uint8 target[20][2], uint8 *sum, bool* perspective_img, float width, float height)
{
	uint8 threshold = 1;
	uint8 count = 0;
	double row_avr = 0;
	double col_avr = 0;
	double rc_cnt = 0;
	
	// 清零
	for (int i = 0; i < 20; i++){
			target[i][0] = 0;
			target[i][1] = 0;
	}
	*sum = 0;
	
	for (int i = 7; i <= (int)(height+0.5) - 7; i++){											// int i = 6; i <= (int)(height+0.5) - 10; i++
		for (int j = 7; j <= (int)(width+0.5) - 7; j++){											// int j = 6; j <= (int)(width+0.5) - 10; j++
			count = 0;
			if (perspective_img[i*(int)(width+0.5) + j] == 0){
				row_avr = 0;
				col_avr = 0;
				rc_cnt = 0;
//				if (perspective_img[(i - 1) * (int)(width+0.5) + j - 1] == 0) 	count++;
//				if (perspective_img[(i - 1) * (int)(width+0.5) + j] == 0) 		 	count++;
//				if (perspective_img[(i - 1) * (int)(width+0.5) + j + 1] == 0) 	count++;
//				if (perspective_img[i * (int)(width+0.5) + j - 1] == 0) 			 	count++;
				if (perspective_img[i * (int)(width+0.5) + j + 1] == 0) 				count++;
				if (perspective_img[(i + 1) * (int)(width+0.5) + j - 1] == 0) 	count++;
				if (perspective_img[(i + 1) * (int)(width+0.5) + j] == 0) 			count++;
				if (perspective_img[(i + 1) * (int)(width+0.5) + j + 1] == 0) 	count++;								
			}
			if (count >= threshold){
				for (int u = 0; u <= 7; u++){
					for (int v = -7; v <= 7; v++){
						if (perspective_img[(i+u) * (int)(width+0.5) + j+v] == 0){
							row_avr += i+u;
							col_avr += j+v;
							rc_cnt = rc_cnt + 1;
						}
						perspective_img[(i+u) * (int)(width+0.5) + j+v] = 1;
					}
				}
				target[*sum][0] = (int)(row_avr/rc_cnt + 0.5);
				target[*sum][1] = (int)(col_avr/rc_cnt + 0.5);
				*sum = *sum + 1;
				if (*sum > MAX_TARGET_NUM)
					return false;
			}
		}
	}
	
	
	for (int i = 0; i < *sum; i++)
		ips200_draw_line((int)(width+0.5)/2, 130, target[i][1], 130 + target[i][0], RGB565_CYAN);

	coordinate_cal(target, *sum, width, height);
	return true;
//	for (int i = 0; i < *sum && i < 15; i++){
//		ips200_show_int(130, 20*i, target[i][1], 3);		
//		ips200_show_int(160, 20*i, target[i][0], 3);
//	}
//	wireless_uart_send_string("seed success!\n");
}


//------------------------------------------------------------------------------------------------
// 函数简介			坐标计算
// 参数说明			target[20][2]										保存图像内目标点的坐标
// 参数说明			sum															目标点的个数
// 参数说明			width														实际场地宽度 单位m
// 参数说明			height													实际场地高度 单位m
// 参数说明			width														目标宽度
// 参数说明			height													目标高度
// 使用示例			coordinate_cal(target, W, H, width, height)
//
// 备注信息			将先前计算得到的target坐标值量化为实际的场地上的坐标点，坐标点间距离为0.2m
//------------------------------------------------------------------------------------------------
void coordinate_cal(uint8 target[20][2], uint8 sum, float width, float height)
{
	for (int i = 0; i < sum; i++){
//		target[i][0] = H * 5 - (int)(target[i][0] * H * 5 / height);
//		target[i][1] = ceil(target[i][1] * W * 5 / width);
		// 取出质心后 坐标计算方式要重新修改
		target[i][0] = ceil((float)H * 5.0 - target[i][0] * (float)H * 5.0 / height);
		target[i][1] = ceil(target[i][1] * (float)W * 5.0 / width);
		
		
//		target[i][0] = (int)((height - target[i][0]) * H * 5 / height + 0.5);
//		target[i][1] = (int)(target[i][1] * W * 5 / width + 0.5);
	}
//	wireless_uart_send_string("coordinate success!\n");
}


//------------------------------------------------------------------------------------------------
// 函数简介			路径规划--贪心算法
// 参数说明			target[20][2]										保存图像内目标点的坐标
// 参数说明			route														保存路径坐标
// 参数说明			sum													保存图像内目标点的个数
// 使用示例			greedy_plan(target, route, sum);
//
// 备注信息			在main函数里要首先定义一个存储空间保存route，初始化为0
// 备注信息			运行完函数可在main里显示目标个数以及各点坐标
//------------------------------------------------------------------------------------------------
void greedy_plan(uint8 target[20][2], uint8 route[20][2], uint8 sum)
{
	uint8 **target_temp = (uint8**)malloc(sizeof(uint8*) * sum);
	for (int i = 0; i < sum; i++){
		target_temp[i] = (uint8*)malloc(sizeof(uint8) * 2);
		target_temp[i][0] = target[i][0];
		target_temp[i][1] = target[i][1];
	}
	
	uint16 dis[20] = { 0 };
	uint16 minnum = 5000;
	uint8 index = 0;
	for (int i = 0; i < sum; i++){
		dis[i] = pow(target[i][0], 2) + pow(target[i][1], 2);
		if (dis[i] < minnum){
			minnum = dis[i];
			index = i;
			route[0][0] = target_temp[i][0];
			route[0][1] = target_temp[i][1];
		}
	}
	target_temp[index][0] = 100;
	target_temp[index][1] = 100;
	
	for (int i = 1; i < sum; i++){
		minnum = 5000;
		index = 0;
		for (int j = 0; j < sum; j++){
			dis[j] = pow(target_temp[j][0] - route[i-1][0], 2) + pow(target_temp[j][1] - route[i-1][1], 2);
			if (dis[j] < minnum){
				minnum = dis[j];
				index = j;
				route[i][0] = target_temp[j][0];
				route[i][1] = target_temp[j][1];
			}
		}
		target_temp[index][0] = 100;
		target_temp[index][1] = 100;
	}
	
	
	for (int i = 0; i < sum; i++)
			free(target_temp[i]);
	free(target_temp);
	wireless_uart_send_string("planning success!\n");
}


//------------------------------------------------------------------------------------------------
// 函数简介			路径规划--pipe算法
// 参数说明			target[20][2]										保存图像内目标点的坐标
// 参数说明			route														保存路径坐标
// 参数说明			sum													保存图像内目标点的个数
// 参数说明			W														实际宽度
// 参数说明			H														实际高度
// 使用示例			pipe_plan(target, route, sum, W, H);
//
// 备注信息			在main函数里要首先定义一个存储空间保存route，初始化为0
// 备注信息			运行完函数可在main里显示路径各点坐标
//------------------------------------------------------------------------------------------------
void pipe_plan(uint8 target[20][2], uint8 route[20][2], uint8 sum)
{
	  uint8 temp[20][2] = { 0 };
		uint8 bin = 0;
		uint8 count = 0;
		uint8 index = 0;
		bool flag = 0;
		
		for (int i = 5; i <= 5*W; i += 5){
		    count = 0;
				// temp清零
			  for (int j = 0; j < 20; j++){
						temp[j][0] = 0;
					  temp[j][1] = 0;
				}
				// 把在管道内的点放进temp，count代表管道内点的个数
				for (int j = 0; j < sum; j++){
						if (target[j][1] <= i && target[j][1] > i-5){
								temp[count][1] = target[j][1];
								temp[count][0] = target[j][0];
								count++;
						}
				}
				
				for (int f = 0; f <= count - 2; f++){
						for (int h = 0; h <= count - 2; h++){
								if (flag == 0){
										if (temp[h][0] > temp[h+1][0]){
												bin = temp[h][0];
												temp[h][0] = temp[h+1][0];
												temp[h+1][0] = bin;
												bin = temp[h][1];
												temp[h][1] = temp[h+1][1];
												temp[h+1][1] = bin;
										}
								}
								else{
										if (temp[h][0] < temp[h+1][0]){
												bin = temp[h][0];
												temp[h][0] = temp[h+1][0];
												temp[h+1][0] = bin;
												bin = temp[h][1];
												temp[h][1] = temp[h+1][1];
												temp[h+1][1] = bin;
										}
								}
						}
				}
				// 把排好序的点放进route内
				for (int g = 0; g < count; g++){
						route[index][0] = temp[g][0];
						route[index][1] = temp[g][1];
						index++;
				}
				// 根据上一个点的y值判定下一次从上还是下遍历
				if (route[index-1][0] <= H*5/2)
						flag = 0;
				else
						flag = 1;
		}
		wireless_uart_send_string("pipe planning success!\n");
}


//------------------------------------------------------------------------------------------------
// 函数简介			蚁群算法所内部调用的函数
// 参数说明			A[][8]												需要求逆的八维矩阵
// 参数说明			B[][8]												输出的逆矩阵
// 使用示例			Gauss8(A, inv_A);
// 备注信息			用于在透视变换里对八维矩阵A进行求逆操作
//------------------------------------------------------------------------------------------------
double uniform_data(double a, double b,long int * seed)
{
	double t;
	*seed = 2045.0 * (*seed) + 1;
	*seed = *seed - (*seed / 1048576) * 1048576;
	t = (*seed) / 1048576.0;
	t = a + (b - a) * t;
	return t;
}

//------------------------------------------------------------------------------------------------
// 函数简介			路径规划——蚁群算法
// 参数说明			A[][8]												需要求逆的八维矩阵
// 参数说明			B[][8]												输出的逆矩阵
// 使用示例			Gauss8(A, inv_A);
// 备注信息			用于在透视变换里对八维矩阵A进行求逆操作
//------------------------------------------------------------------------------------------------
void greedy_antColonyAlgorithm(uint8 target[20][2],uint8 target_num,uint8 route[20][2]) {
	// wireless_uart_send_string("debug_start\n");
  //srand(time(0)); 
	// double result_temp[n+1][3];
    double TARGET[num_all+1][3];
    int i, j, k;
    double weight[num_all+1][num_all+1];
    double now[3] = {1, 1, 1};
    double weight_temp[num_all][num_all];
    double length_greedy = 0;
		double MIN;
    int index;		
	
    int Table[m+1][num_all+1];
    double Tau[num_all+1][num_all+1];
    double Eta[num_all+1][num_all+1];
    double Length_best;
    // double Length_ave[iter_max+1];
    int Route_best[num_all+1];
    int iter = 1;

		for (i = 0; i < target_num && i < 16; i++){
					ips200_show_int(190, 20*i, target[i][1], 2);
					ips200_show_int(210, 20*i, target[i][0], 2);
				}


    // result_temp initialization
    // result_temp[1][0] = 1;
    // result_temp[1][1] = 1;
    // result_temp[1][2] = 1;

    // TARGET initialization
    TARGET[1][0] = 1;
    TARGET[1][1] = 1;
		TARGET[1][2] = 1;
    for (i = 2; i <= num_all - 1; i++) {
        TARGET[i][0] = i;
        TARGET[i][1] = target[i-2][0];
        TARGET[i][2] = target[i-2][1];
    }
    TARGET[num_all][0] = num_all;
    TARGET[num_all][1] = 3;
    TARGET[num_all][2] = 33;

    // weight initialization
    for (i = 1; i <= num_all; i++) {
        for (j = i + 1; j <= num_all; j++) {
            weight[i][j] = sqrt(pow(TARGET[i][1] - TARGET[j][1], 2) + pow(TARGET[i][2] - TARGET[j][2], 2));
            weight[j][i] = weight[i][j];
        }
        weight[i][i] = 1000;
        // weight[i][1] = 1000;
    }

    double speed_max = 105.0;
    double speed_5_forward = 100.0 / speed_max;
    double speed_5_backward = speed_max / speed_max;
    double speed_3_5 = 90.0 / speed_max;
    double speed_3 = 65.0 / speed_max;

    // 计算等效距离
    // double weight_equal[n][n] = {0.0};

    for (i = 1; i <= num_all; i++) {
        for (j = 1; j <= num_all; j++) {

            if (i != j) {    // 不可以走到自己

                double line[2] = {TARGET[j][2] - TARGET[i][2], TARGET[j][1] - TARGET[i][1]};
                double Angle = acos(line[0] / sqrt(pow(line[0], 2) + pow(line[1], 2)));

                double speed;

                if (weight[i][j] > 5) {
                    if (TARGET[i][1] > TARGET[j][1]) {
                        speed = speed_5_backward / (1 + fabs(tan(Angle))) / fabs(cos(Angle));
                    } else {
                        speed = speed_5_forward / (1 + fabs(tan(Angle))) / fabs(cos(Angle));
                    }

                } else if (weight[i][j] > 3 && weight[i][j] <= 5) {
                    speed = speed_3_5 / (1 + fabs(tan(Angle))) / fabs(cos(Angle));

                } else if (weight[i][j] <= 3) {
                    speed = speed_3 / (1 + fabs(tan(Angle))) / fabs(cos(Angle));
                }

                speed = fabs(speed);
                weight[i][j] = weight[i][j] / speed;
            }
        }
    }
		
    for (i = 1; i < num_all; i++) {
        for (j = 1; j < num_all; j++) {
            weight_temp[i][j] = weight[i][j];
        }
        weight_temp[i][1] = 1000;
    }
		
//	wireless_uart_send_string("debug0\n");
	

    for (i = 2; i < num_all; i++) {
        MIN = 1000;
        index = 0;	
        for (j = 1; j < num_all; j++) {
			int now_num = now[0];
            if (weight_temp[now_num][j] < MIN) {
                MIN = weight_temp[now_num][j];
                index = j;
            }
        }
        length_greedy += MIN;
        // result_temp[i][0] = TARGET[index][0];
        // result_temp[i][1] = TARGET[index][1];
        // result_temp[i][2] = TARGET[index][2];
        now[0] = TARGET[index][0];
		now[1] = TARGET[index][1];
		now[2] = TARGET[index][2];
		for (j = 1 ;j < num_all;j++)
		{
			weight_temp[j][index] = 1000;
		}

    }

    // result_temp[n][0] = TARGET[n][0];
    // result_temp[n][1] = TARGET[n][1];
    // result_temp[n][2] = TARGET[n][2];
    length_greedy += weight[index][num_all];

    // printf("贪心算法\n");
    // for (i = 1; i <= n; i++) {
    //     printf("%f %f\n", result_temp[i][1], result_temp[i][2]);
    // }
//    printf("长度：%f\n\n", length_greedy);
	// wireless_uart_send_string("debug1\n");

    // Eta initialization
    for (i = 1; i <= num_all; i++) {
        for (j = 1; j <= num_all; j++) {
            Eta[i][j] = 1 / weight[i][j];
        }
    }

    // Tau initialization
    for (i = 1; i <= num_all; i++) {
        for (j = 1; j <= num_all; j++) {
            if (i == j )
                Tau[i][j] = 0;

            else
                Tau[i][j] = m / length_greedy;
                // Tau[i][j] = 0.9;
        }
    }

    while (iter <= iter_max) {
			
				if (iter % 100 == 0)
						ips200_show_int(30, 260, iter,4);
				
        int point_index[num_all];      //构建解空间
        for (j = 1; j <= num_all-1; j++) {
            point_index[j] = j;
        }

        for (i = 1; i <= m; i++) {
            Table[i][1] = 1;
            Table[i][num_all] = num_all;

            for (j = 2; j <= num_all-1; j++) {//确定蚂蚁所走的每一个城市
                int visited[j];
                for (k = 1; k < j; k++) {
                    visited[k] = Table[i][k];
                }

								
                int allow_index[num_all];// 得到允许访问的点的编号
                for (k = 1; k <= num_all-1; k++) {
                    allow_index[k] = 1;
                }
                for (k = 1; k < j; k++) {
                    allow_index[visited[k]] = 0;
                }

								
                int allow_count = 1;
                int allow[num_all];//待访问
                for (k = 1; k <= num_all-1; k++) {
                    if (allow_index[k] == 1) {
                        allow[allow_count] = point_index[k];
                        allow_count++;
                    }
                }

                double P[allow_count];//轮盘赌方法访问下一个城市
                double sumP = 0;
                for (k = 1; k < allow_count; k++) {
                    int a = visited[j-1];
                    int b = allow[k];
                    P[k] = pow(Tau[a][b], alpha) * pow(Eta[a][b], beta);
                    sumP += P[k];
                }
                for (k = 1; k < allow_count; k++) {
                    P[k] /= sumP;
                }

                double Pc[allow_count];
                Pc[1] = P[1];
                for (k = 2; k < allow_count; k++) {
                    Pc[k] = Pc[k-1] + P[k];
                }
								
								long int s;
								if(iter == 1)
										s = rand();
								
                double randomNum = uniform_data(0.0,1.0,&s);
//								double randomNum = rand()/RAND_MAX;
                int target_now;
                for (k = 1; k < allow_count; k++) {
                    if (Pc[k] >= randomNum) {
                        target_now = allow[k];
                        break;
                    }
                }

                Table[i][j] = target_now;
            }
        }

        double Length[m+1];//计算各个蚂蚁的路径距离
        for (i = 1; i <= m; i++) {
            Length[i] = 0;
            for (j = 1; j <= num_all-1; j++) {
                Length[i] += weight[Table[i][j]][Table[i][j+1]];
            }
        }

        double min_Length = Length[1];//计算最短路径
        int min_index = 1;
        for (i = 1; i <= m; i++) {
            if (Length[i] < min_Length) {
                min_Length = Length[i];
                min_index = i;
            }
        }

        if (iter == 1) {
            Length_best = min_Length;

            for (j = 1; j <= num_all; j++) {
                Route_best[j] = Table[min_index][j];
            }
        } else {
            if (min_Length < Length_best) {
                Length_best = min_Length;

                for (j = 1; j <= num_all; j++) {
                    Route_best[j] = Table[min_index][j];
                }
            } 
            // else {
            //     // Length_best = Length_best;

            //     for (j = 1; j <= n; j++) {
            //         Route_best[j] = Route_best[j];
            //     }
            // }
        }

        double Delta_Tau[num_all+1][num_all+1];//更新信息素初始化
        for (i = 1; i <= num_all; i++) {
            for (j = 1; j <= num_all; j++) {
                Delta_Tau[i][j] = 0;
            }
        }

        for (i = 1; i <= m; i++) {//逐个蚂蚁进行计算
            for (j = 1; j <= num_all-1; j++) {
                Delta_Tau[Table[i][j]][Table[i][j+1]] += Q / Length[i];
            }
        }

        for (i = 1; i <= num_all; i++) {
            for (j = 1; j <= num_all; j++) {
                Tau[i][j] = (1 - rou) * Tau[i][j] + Delta_Tau[i][j];
							
                if (Tau[i][j] > 2)
                    Tau[i][j] = 2;
                else if (Tau[i][j] < 0.5)
                    Tau[i][j] = 0.5;
            }
        }

        iter++;
        // for (i = 1;i <= m;i++){
        //     for(j = 1;j <= n;j++){
        //         Table[i][j] = 0;
        //     }
        // }
    }
		// wireless_uart_send_string("debug2\n");

    int point[num_all+1][2];
    for (i = 1; i <= num_all; i++) {
        point[i][0] = TARGET[i][1];
        point[i][1] = TARGET[i][2];
    }

    uint8 Shortest_Length = (uint8)Length_best;
    // int Shortest_Route[n+1];
    // for (j = 1; j <= n; j++) {
    //     Shortest_Route[j] = Route_best[j];
    // }
		
	  for (j = 0; j < target_num + 2; j++) {
			route[j][0] = point[Route_best[j+2]][0];
			route[j][1] = point[Route_best[j+2]][1];	
    }
	
//		for (i = 0; i < target_num && i < 16; i++){
//					ips200_show_int(190, 20*i, route[i][1], 2);
//					ips200_show_int(210, 20*i, route[i][0], 2);A
//				}
//		ips200_show_int(190, 20*target_num, Shortest_Length, 3);
//    printf("最短距离： %f\n", Shortest_Length);
//    printf("最短路径：\n");
//    for (i = 1; i <= n; i++) {
//        printf("%d ", Route_best[i]);
//    }
//    printf("\n");
//    printf("最短路径的坐标：\n");
//    for (i = 0; i < n-2; i++) {
//        printf("%d %d\n", route[i][1], route[i][0]);
//    }

//		for (int i = 0; i<target_num;i++)
//		{
//				route[i][0] = target[i][0];
//				route[i][1] = target[i][1];
//		}
		
}


void findPoint2House(uint8 point_end[2], uint8 point[3][2]) {
		double x0 = 36;
		double y0 = 26;
    double end_x = point_end[0];
    double end_y = point_end[1];
    double theta = atan((2 * y0 - 4 - end_y) / (2 * x0 - end_x));
    
    double point_second = round((y0 - 4) * tan(PI / 2 - theta));
    double point_first = round((x0 - end_x) * tan(theta)) + end_y;

		if (point_second > 25) {
				point_second = 25;
				if(point_first > 20){
						point_first = 20;
				}
	}
		
		point[0][0] = 36;
		point[0][1] = (uint8)point_first;
		point[1][0] = (uint8)point_second;
		point[1][1] = 26;
		point[2][0] = 0;
		point[2][1] = 8;
}


//------------------------------------------------------------------------------------------------
// 函数简介			八维矩阵的求逆
// 参数说明			A[][8]												需要求逆的八维矩阵
// 参数说明			B[][8]												输出的逆矩阵
// 使用示例			Gauss8(A, inv_A);
// 备注信息			用于在透视变换里对八维矩阵A进行求逆操作
//------------------------------------------------------------------------------------------------
void Gauss8(int A[][8], double B[][8])
{
	int n = 8;
	int i, j, k;
	double max, temp;
	double t[8][8]; 

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			t[i][j] = A[i][j];
		}
	}

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			B[i][j] = (i == j) ? (double)1 : 0;
		}
	}
	for (i = 0; i < n; i++)
	{

		max = t[i][i];
		k = i;
		for (j = i + 1; j < n; j++)
		{
			if (fabs(t[j][i]) > fabs(max))
			{
				max = t[j][i];
				k = j;
			}
		}

		if (k != i)
		{
			for (j = 0; j < n; j++)
			{
				temp = t[i][j];
				t[i][j] = t[k][j];
				t[k][j] = temp;

				temp = B[i][j];
				B[i][j] = B[k][j];
				B[k][j] = temp;
			}
		}

		temp = t[i][i];
		for (j = 0; j < n; j++)
		{
			t[i][j] = t[i][j] / temp;
			B[i][j] = B[i][j] / temp;
		}
		for (j = 0; j < n; j++)
		{
			if (j != i)
			{
				temp = t[j][i];
				for (k = 0; k < n; k++)
				{
					t[j][k] = t[j][k] - t[i][k] * temp;
					B[j][k] = B[j][k] - B[i][k] * temp;
				}
			}
		}
	}
}


//------------------------------------------------------------------------------------------------
// 函数简介			double矩阵的求逆
// 参数说明			src													需要求逆的double矩阵
// 参数说明			row													矩阵的行数
// 参数说明			col													矩阵的列数
// 参数说明			res2												返回的逆矩阵
// 使用示例			Matrix_inver(D, 3, 3);
// 备注信息			用于在透视变换里对double矩阵D进行求逆操作，保证精度
//------------------------------------------------------------------------------------------------
double** Matrix_inver(double** src, int row, int col)
{
	wireless_uart_send_string("step0\n");
	//step 1
	int i, j, k, n;
	double** res, ** res2, tmp;
	int count = 0;
	//step 2
	res = (double**)malloc(sizeof(double*) * row);
	res2 = (double**)malloc(sizeof(double*) * row);
	wireless_uart_send_string("step1\n");
	n = 2 * row;
	for (i = 0; i < row; i++)
	{
		res[i] = (double*)malloc(sizeof(double) * n);
		res2[i] = (double*)malloc(sizeof(double) * col);
		memset(res[i], 0, sizeof(res[0][0]) * n);
		memset(res2[i], 0, sizeof(res2[0][0]) * col);
	}
	wireless_uart_send_string("step2\n");
	//step 3
	for (i = 0; i < row; i++)
	{
		memcpy(res[i], src[i], sizeof(res[0][0]) * n);
	}
	for (i = 0; i < row; i++)
	{
		for (j = col; j < n; j++)
		{
			if (i == (j - row))
				res[i][j] = 1.0;
		}
	}
	wireless_uart_send_string("step3\n");
	for (j = 0; j < col; j++)
	{
		//step 4
		count = j;
		double Max = fabs(res[count][j]);
		for (i = j; i < row; i++)
		{
			if (fabs(res[i][j]) > Max)
			{
				count = i;
				Max = fabs(res[i][j]);
			}
		}
		if (i == j && i != count)
		{
			for (k = 0; k < n; k++)
			{
				tmp = res[count][k];
				res[count][k] = res[i][k];
				res[i][k] = tmp;
			}
		}
		wireless_uart_send_string("step4\n");
		//step 5
		for (i = 0; i < row; i++)
		{
			if (i == j || res[i][j] == 0)continue;
			double b = res[i][j] / res[j][j];
			for (k = 0; k < n; k++)
			{
				res[i][k] += b * res[j][k] * (-1);
			}
		}
		double a = 1.0 / res[j][j];
		for (i = 0; i < n; i++)
		{
			res[j][i] *= a;
		}
	}
	wireless_uart_send_string("step5\n");
	//step 6
	for (i = 0; i < row; i++)
	{
		memcpy(res2[i], res[i] + row, sizeof(res[0][0]) * row);
	}
	wireless_uart_send_string("step6\n");
	free(res);
	return res2;
}

