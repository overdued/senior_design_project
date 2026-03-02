#pragma once
#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"

# define ind(i, j)											((i)*MT9V03X_W + (j))
# define MAX_TARGET_NUM								  16													// 限定最多能找到的目标点个数
# define MAX_ITERATE_NUM								1														// 限定最大能重新透视的次数
# define W															7														// 场地真实宽度 单位m
# define H															5														// 场地真实高度 单位m  ->  代表场地真实大小为7×5


//void servo_slow_ctrl(uint16 _servo1_angle, uint16 _servo2_angle, float _step_count);
void get_image(uint8 *gray_image, uint8 *input_image);
void binarize_image(bool *binary_image, uint8 threshold, uint8 *input_image);
void movingThreshold(bool *binary_image, uint8 *input_image, uint8 N, float c);
void refined_image(bool *binary_image, bool *I);
uint8 otsu(uint8 *input_image);
void find_corners(int corner[4][2], bool *binary_image);
void perspective_tran(bool* perspective_img, bool *binary_image, int corner[4][2], float width, float height);
bool perspective_is_cor(bool* perspective_img, int corner[4][2], float width, float height);
bool seed_function(uint8 target[20][2], uint8 *sum, bool* perspective_img, float width, float height);
void coordinate_cal(uint8 target[20][2], uint8 sum, float width, float height);
void greedy_plan(uint8 target[20][2], uint8 route[20][2], uint8 sum);
void pipe_plan(uint8 target[20][2], uint8 route[20][2], uint8 sum);

double uniform_data(double a, double b,long int * seed);
void greedy_antColonyAlgorithm(uint8 target[20][2],uint8 target_num,uint8 route[20][2]);
void findPoint2House(uint8 point_end[2], uint8 point[3][2]);

void Gauss8(int A[][8], double B[][8]);
double** Matrix_inver(double** src, int row, int col);

