#include "zf_common_headfile.h"
#include "run_kws_model_demo.h"
#include "math.h"
#include <stdlib.h>
#include <string.h>
#include "user_function.h"
#include "JY901.h"

extern volatile float roll;//x轴角度
extern volatile float pitch;//y轴角度
extern volatile float yaw;//z轴角度
float gy;
float angle1 = 0;
float last_angle1 = 0;
int Tp_count = 0;


// ----------------- 状态定义 -----------------
#define STATE_SEARCHING        0   // 搜索物体
#define STATE_AVOIDING         1   // 避障
#define STATE_FOUND            2   // 停车等待抓取
#define STATE_MOVING_TO_TARGET 3   // 前进到物体
#define STATE_RETURNING        4   // 掉头后搜索起点标志物
#define STATE_FIND_BEGIN       5   // 检测到起点标志物
#define STATE_MOVING_TO_BEGIN  6   // 前进到起点
#define STATE_WAIT_MOVE        7   // 等待启动命令
#define STATE_TURNING_AT_BEGIN 8 
#define STATE_TURNING					 9


#define PIT_PRIORITY            (PIT_IRQn)
#define IPS200_TYPE     (IPS200_TYPE_SPI)                                         // 单排排针 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI
//openart摄像头串口
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_1
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           
#define UART_PRIORITY           (LPUART1_IRQn)                                  // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体
uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区
uint8 get_data = 0;                                                             // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数
fifo_struct uart_data_fifo;
int art_rflag = 0;//openart串口信息执行标志
int operate_count = 0; //openart通讯操作计数
int timeout_count = 0;//超时计数 
int en_count = 0;
int dis_count = 0;
int test_count = 0;
int lr_flag = 0; //视觉校正·使能标记
int box_num = 0;//货箱序号
int box_pre = 3;//前次货箱序号，初始置为3号箱
int box_angle[6] = {225,180,0,270,90,45};
int cmu_count = 0;//单次矫正通信次数
int cmu_test = 0;
int fifo_count = 0;//实际读取fifo个数，防止fifo数组首位为0而跳过有效信息
int fifo_random = 0;


float speedy_k = 1; //1.06
int32 motor1 = 0 ,motor2 = 0,motor3 = 0,motor4 = 0;   //电机转速
int speed_x = 0,speed_y = 0;
double target_theta;
int set_speed1,set_speed2,set_speed3,set_speed4 = 0;
int tset_speed1,tset_speed2,tset_speed3,tset_speed4 = 0;
float speed = 120;
int standard_speed = 75; //75
int motor1_min,motor1_max,motor2_min,motor2_max,motor3_min,motor3_max,motor4_min,motor4_max = 0;
int standard_pulse = 12447;
int pre_pulse_x = 0;
int pre_pulse_y = 0;
int target_pulse_x = 0;
int target_pulse_y = 0;
int inertia_pulse = 120; //120 376
int dist_flag = 0;
int16 debug_speed = 1500; //3000
int16 debug_speed_back = 1500; //3000
int16 speed1,speed2,speed3,speed4;
int motorstate = 0; //运动状态 1为前进 2为后退 3为左平移 4为右平移





float angle_k = 0.5;
int err1,err2,err3,err4 = 0;
uint8 data_buffer[32];
uint8 data_len;
bool first = 0;
//----------------------跑点-------------------
//uint8 x[17] = {1,10,13,14,10,5,5,5,14,14,18,22,20,28,33,33,33};
//uint8 y[17] = {1,4,7,10,11,10,14,18,16,23,23,18,14,12,16,23,11};
//uint8 x[13] = {1,5,12,18,17,15,26,31,32,24,27,26,32};
//uint8 y[13] = {1,8,9,9,13,21,18,18,13,13,10,6,6};
uint8 x[20] = {1};
uint8 y[20] = {1};
//uint8 y[13] = {1,11,10,14,23,21,22,17,12,5,8,7,4};
//uint8 y[13] = {1,11,10,14,23,21,22,17,12,5,8,7,4};
//uint8 x[21] = {1,6,9,14,15,10,7,5,14,19,21,21,21,25,26,24,26,27,29,33,32};
//uint8 y[21] = {1,6,7,3,11,13,11,20,18,19,15,11,5,5,8,12,15,18,22,16,12};
//uint8 x[21] = {1,3,9,12,12,16,16,13,7,16,22,21,26,29,32,30,32,26,21,29,33};
//uint8 y[21] = {1,5,7,5,9,7,12,13,21,22,19,15,18,21,22,16,9,7,5,5,5};
//int x[16] = {1,4,8,8,3,9,13,16,21,23,20,32,31,28,27,30};
//int y[16] = {1,5,4,10,13,17,12,7,5,9,13,16,9,10,7,4};
//int x[16] = {1,4,8,8,3,9,13,16,21,23,20,28,27,30,31,32};
//int y[16] = {1,5,4,10,13,17,12,7,5,9,13,10,7,4,9,16};
int run = 0; //启动跑点
int first_start = 1;
int xy = 0; //坐标迭代
int back_enable = 0; //入库使能
int back_flag = 0; //后退避让目标过程取消超时后退
int wayback_flag = 0; //避让后回到目标位置
double way_count = 0; //返回目标移动距离
int stop_speed = 0;
int zero_count = 0;
int point_num = 0; //已知点个数
int random_num = 0;//随机点个数
int task_finish = 0;//已知点搜索与拾取完成标志
float pulse_yt = 0.0; //记录搜索随机目标时的前进里程
float pulse_xt = 0.0; //记录向左搜索时的里程
int right_flag = 0;//向右搜索使能标志
int up_flag = 0; //标志进入上半区域
int up_stop_flag = 0;//进入上半场后先停止运动等待搜索信息
int left_limit = 0;//下半区域搜索时向左运行到达边界
int limit_xpulse = 0; //达到左边界时记录视觉矫正X方向的里程
int limit_count = 0;//达到左边界时记录视觉矫正X方向的计数标记
int back_count = 0; //下半场后退计数标记
int down_speed = 60;// 下半场巡航速度
int openart4_1 = 0;//随机点拾取后开启水平openart的计时标志
int openart4_1_count = 0; //随机点拾取后水平openart的计时
int limit_counter = 0;//到达边界后超时并进行角度校正计数
int whole_pulsex = 0; //全场x坐标记录
int whole_pulsey = 0; //全场y坐标记录
int pulse_first = 0; //记录随机目标前进搜索时底线的水平位移，避免跳点
int pulse_first_count = 0;//记录跳点返回计数
int pulse_first_counter = 0;//记录随机搜索时位于底线的目标标计
uint8 home_route[3][2] = { 0 };
uint8 point_end[2] = { 0, 0 };
//------------------------------------------------
#define UART4_RX_BUFFER_SIZE 256///定义大小
#define UART4_INDEX              (UART_4)    
fifo_struct uart_data_fifo4;
uint32 fifo_data_count4 = 0;                                                     // fifo 数据个数
//----------------------------------------------模仿写入-------------------------------//
volatile uint8_t uart4_rx_buf[64];
volatile int uart4_rx_index = 0;
volatile int uart4_rx_ready = 0;

volatile uint8_t uart4_rx_buffer[UART4_RX_BUFFER_SIZE];
volatile uint16_t uart4_rx_buffer_index = 0;
volatile uint16_t uart4_disp_pos = 0;  // 当前显示位置（横坐标）

void uart4_rx_interrupt_handler_1(void)
{
    uint8_t byte;
    if (uart_query_byte(UART4_INDEX, &byte))
    {
        if (uart4_rx_index < sizeof(uart4_rx_buf) - 1)
        {
            uart4_rx_buf[uart4_rx_index++] = byte;

            if (byte == '\r' || byte == '\n')
            {
							 if (uart4_rx_index > 1)  // 说明除了换行还有数据
							 {
							  uart4_rx_buf[uart4_rx_index] = '\0';
                uart4_rx_ready = 1;  // 设置主循环标志
							 }

                uart4_rx_index = 0;
            }
        }
        else
        {
            uart4_rx_index = 0;
        }

        fifo_write_buffer(&uart_data_fifo4, &byte, 1);
        fifo_data_count4 = fifo_used(&uart_data_fifo4);
    }
}


float angle;     //当前角度
float last_angle;    //上一时刻角度
float ag_dt = 0.005;    //采样周期    
float Tp = 0; //温补系数
float delta_Tp = 0; //温补系数

int interupt_count = 1;
int time_count = 0;

int eff_angle = 2; //最小范围时可调角度 5 10 15
int eff_angle1 = 2;

int debug_count = 0;
int debug_count1 = 0;
int debug_count2 = 0;



//----------角度-------------前行--------//
volatile int16_t target_angle = 0;        // 目标角度
volatile uint8_t rotating_flag = 0;

volatile int32_t pulse_count = 0;         // 当前累计脉冲
volatile int32_t target_pulse = 0;        // 目标脉冲
volatile uint8_t moving_flag = 0;
float lock_angle = 0;            // 当前目标角度（由当前角度 + 偏移得到）
volatile uint8 angle_turn_flag = 0;       // 是否正在执行角度旋转动作

float normalize_angle(float angle)
{
    while (angle > 180)  angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}
void motor_ctr()
{
//后轮	 
			  if(motor1>0)
			  {
					if(motor1 > debug_speed_back)
					{
						motor1 = debug_speed_back;
					}			
				  gpio_set_level(C7, 1);
					pwm_set_duty(PWM2_MODULE0_CHA_C6, motor1);
			  }
				else
				{
					if(motor1 < -debug_speed_back)
					{
						motor1 = -debug_speed_back;
					}
					gpio_set_level(C7, 0);
					pwm_set_duty(PWM2_MODULE0_CHA_C6, -motor1);
				}
				
				 if(motor2>0)
			  {
					if(motor2 > debug_speed_back)
					{
						motor2 = debug_speed_back;
					}
				  gpio_set_level(C9, 1);
					pwm_set_duty(PWM2_MODULE1_CHA_C8, motor2);
			  }
				else
				{
					if(motor2 < -debug_speed_back)
					{
						motor2 = -debug_speed_back;
					}
					gpio_set_level(C9, 0);
					pwm_set_duty(PWM2_MODULE1_CHA_C8, -motor2);
				}

//前轮				
			  if(motor3>0)
			  {
					if(motor3 > debug_speed)
					{
						motor3 = debug_speed;
					}
				  gpio_set_level(C10, 1);
					pwm_set_duty(PWM2_MODULE2_CHB_C11, motor3);
			  }
				else
				{
					if(motor3 < -debug_speed)
					{
						motor3 = -debug_speed;
					}
					gpio_set_level(C10, 0);
					pwm_set_duty(PWM2_MODULE2_CHB_C11, -motor3);
				}

			  if(motor4>0)
			  {
					if(motor4 > debug_speed)
					{
						motor4 = debug_speed;
					}
				  gpio_set_level(D2, 1);
					pwm_set_duty(PWM2_MODULE3_CHB_D3, motor4);
			  }
				else
				{
					if(motor4 < -debug_speed)
					{
						motor4 = -debug_speed;
					}
					gpio_set_level(D2, 0);
					pwm_set_duty(PWM2_MODULE3_CHB_D3, -motor4);
				}				
}


//--------------------------------speed change------------------------------------------------

volatile uint8_t flag_display = 0;
volatile uint8_t flagtest= 0;
float kp1 = 9; //2 1
float kp2 = 4; //2 1
float kp3 = 4; //2 1
float kp4 = 7;

void turn_move_handler()
{
    flag_display = 1; // 表示进入中断，主函数刷新显示用


    // 2. 读取四个轮子的速度
    speed2 = -encoder_get_count(QTIMER1_ENCODER2);  // motor1(+)
    speed1 = encoder_get_count(QTIMER1_ENCODER1); // motor2(-)
    speed3 = encoder_get_count(QTIMER2_ENCODER2); // motor3(-)
    speed4 = -encoder_get_count(QTIMER2_ENCODER1);  // motor4(+)

    // 3. 清除编码器计数
    encoder_clear_count(QTIMER1_ENCODER1);
    encoder_clear_count(QTIMER1_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER1);

    // 5. 计算速度差值
    err1 = tset_speed1 - speed1;
    err2 = tset_speed2 - speed2;
    err3 = tset_speed3 - speed3;
    err4 = tset_speed4 - speed4;

			tset_speed1 = set_speed1 - normalize_angle(yaw - lock_angle)  * angle_k;
			tset_speed2 = set_speed2 + normalize_angle(yaw - lock_angle)  * angle_k;
			tset_speed3 = set_speed3 - normalize_angle(yaw - lock_angle)  * angle_k;
			tset_speed4 = set_speed4 + normalize_angle(yaw - lock_angle)  * angle_k;
		
		
    // 6. 加入角度误差补偿


    // 7. 限幅保护
    if (tset_speed1 < motor1_min) tset_speed1 = motor1_min;
    if (tset_speed1 > motor1_max) tset_speed1 = motor1_max;
    if (tset_speed2 < motor2_min) tset_speed2 = motor2_min;
    if (tset_speed2 > motor2_max) tset_speed2 = motor2_max;
    if (tset_speed3 < motor3_min) tset_speed3 = motor3_min;
    if (tset_speed3 > motor3_max) tset_speed3 = motor3_max;
    if (tset_speed4 < motor4_min) tset_speed4 = motor4_min;
    if (tset_speed4 > motor4_max) tset_speed4 = motor4_max;

    // 8. 再次计算误差（限幅后）
    err1 = tset_speed1 - speed1;
    err2 = tset_speed2 - speed2;
    err3 = tset_speed3 - speed3;
    err4 = tset_speed4 - speed4;

    // 9. PID 调速（此处只用 P）
    if (abs(err1) >= 2) motor1 -= err1 * kp1;
    if (abs(err2) >= 2) motor2 -= err2 * kp2;
    if (abs(err3) >= 2) motor3 += err3 * kp3;//错误====
    if (abs(err4) >= 2) motor4 += err4 * kp4;

    // 10. 输出 PWM
    motor_ctr();
		

				if (moving_flag)
				{
						// 累计前进距离：使用两个前轮的平均速度估算
						pulse_count += (abs(speed3) + abs(speed4)) *0.6256;

				}

}



//------------------------避障-----------------------//
//注意要在isr.c里修改GPIO1_Combined_0_15_IRQHandler(void)代码调用中断函数
//---------------------------------------避障模块--------------------------------------------------
//先初始化串口，rt1064与避障模块通过uart通信
#define UART_BASE LPUART1
uint8_t uart_rx_data;

// 超声波模块引脚定义
#define TRIG_PIN     B9   // Trig 引脚连接 B9：输出
#define ECHO_PIN     B10   // Echo 引脚连接 D10：输入

#define TRIG2_PIN     D13      // Trig 引脚连接 B9：输出
#define ECHO2_PIN     D14   // Echo 引脚连接 D10：输入
// 假设系统时钟 600MHz
#define SYSTEM_CLOCK     (600000000U)
#define TICK_PER_US      (SYSTEM_CLOCK / 1000000U)
volatile uint32_t systick_ms_count = 0;  // 毫秒计数器

// ===================== 1. 初始化 =========================
void systick_user_init(void)
{
    // 假设系统主频为600MHz
    uint32_t core_clock = CLOCK_GetFreq(kCLOCK_CoreSysClk);  // 或者写死为 600000000
    SysTick->LOAD = core_clock / 1000 - 1;  // 1ms定时
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}


// ===================== 3. 获取毫秒时间戳 ==================
uint32_t systick_get_ms(void)
{
    return systick_ms_count;
}

uint64_t systick_get_us(void)
{
    uint32_t ms = systick_get_ms();              // 毫秒时间
    uint32_t val = SysTick->VAL;                 // 当前倒计数值（剩余计数）

    // SysTick 计数从 LOAD 到 0，因此我们需要用 LOAD - VAL
    extern uint32_t SystemCoreClock;
    uint32_t load = SysTick->LOAD;

    uint32_t us_offset = (load - val) / TICK_PER_US;
    return ((uint64_t)ms) * 1000 + us_offset;
}

//---------------------------初始化函数------------------------------//
void hcsr04_init(void)
{
	//初始化IO口与避障模块进行连接，这里 B17-Trig（接受1064给的启动信号）,D13-Echo（向1064发送距离）
	gpio_init(TRIG_PIN,GPO,0,GPO_PUSH_PULL);// 初始化Trig引脚为推挽输出，初始电平0
	exti_init(ECHO_PIN, EXTI_TRIGGER_RISING);//定义中断接口
  
	//初始化第二个避障模块
	gpio_init(TRIG2_PIN,GPO,0,GPO_PUSH_PULL);
	exti_init(ECHO2_PIN, EXTI_TRIGGER_RISING);
}
#define HC_SR04_MAX_PULSE 25000   // 最大脉冲宽度（≈23ms，约400cm）
#define HC_SR04_INVALID   -1.0f   // 无效数据标记
#define FILTER_SIZE 5             // 滤波窗口大小

// 左模块
volatile uint32_t echo_rise_time = 0;
volatile uint32_t echo_fall_time = 0;
volatile uint8_t echo_flag = 0;
volatile float distance_cm = 100;
volatile uint8_t distance_ready = 0;

// 滤波缓冲
static float filter_buf[FILTER_SIZE];
static uint8_t filter_index = 0;
static uint8_t sample_count = 0;

float hc_sr04_filter(float new_val)
{
    filter_buf[filter_index] = new_val;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    if (sample_count < FILTER_SIZE) sample_count++;

    // 计算平均值
    float sum = 0;
    for (uint8_t i = 0; i < sample_count; i++) sum += filter_buf[i];
    return sum / sample_count;
}

void HC_SR04_handler(void)
{
    if (echo_flag == 0)
    {
        echo_rise_time = systick_get_us();
        echo_flag = 1;
        exti_init(ECHO_PIN, EXTI_TRIGGER_FALLING);
    }
    else
    {
        echo_fall_time = systick_get_us();
        echo_flag = 0;
        exti_init(ECHO_PIN, EXTI_TRIGGER_RISING);

        uint32_t pulse = echo_fall_time - echo_rise_time;

        // 超时 → 无效
        if (pulse >= HC_SR04_MAX_PULSE)
        {
            distance_cm = HC_SR04_INVALID;
        }
        else
        {
						
            float d = pulse * 0.0343f / 2.0f;
            distance_cm = hc_sr04_filter(d);
        }
        distance_ready = 1;
    }
}

//第二个避障模块
// 记录开始和结束时间的变量
volatile uint32_t echo_rise_time_2 = 0;
volatile uint32_t echo_fall_time_2 = 0;
volatile uint8_t echo_flag_2 = 0;   // 0 表示等待上升沿，1 表示等待下降沿
volatile float distance_cm_2 = 100; 
volatile uint8_t distance_ready_2 = 0;  // 新增：数据就绪标志
// 中断处理函数
void HC_SR04_2_handler(void)
{
    if (echo_flag_2 == 0)
    {
        // 捕获上升沿
        echo_rise_time_2 = systick_get_us();
        echo_flag_2 = 1;

        // 改为下降沿触发
        exti_init(ECHO2_PIN, EXTI_TRIGGER_FALLING);
    }
    else
    {
        // 捕获下降沿
        echo_fall_time_2 = systick_get_us();
        echo_flag_2 = 0;

        // 改为上升沿触发（等待下一次测量）
        exti_init(ECHO2_PIN, EXTI_TRIGGER_RISING);

        // 计算脉冲宽度
        uint32_t pulse = echo_fall_time_2 - echo_rise_time_2;

        // 防止异常脉冲（超过 25ms 视为无效）
        if (pulse < 25000)
        {
            distance_cm_2 = pulse * 0.0343f / 2.0f;  // 单位：cm
            distance_ready_2 = 1;                    // 标记测量完成
        }
		
    }
}
volatile uint8_t trig_toggle = 0;

void hc_sr04_trig_send(void)
{
    if (trig_toggle == 0) {
        gpio_set_level(TRIG_PIN, 0);
        system_delay_us(2);
        gpio_set_level(TRIG_PIN, 1);
        system_delay_us(10);
        gpio_set_level(TRIG_PIN, 0);
        trig_toggle = 1;
    } else {
        gpio_set_level(TRIG2_PIN, 0);
        system_delay_us(2);
        gpio_set_level(TRIG2_PIN, 1);
        system_delay_us(10);
        gpio_set_level(TRIG2_PIN, 0);
        trig_toggle = 0;
    }
}

//----------------------------------






//---------------------------旋转-------------------//
int turn_speed = 0;
void turn_angle(float angle_num)
{
			//计算目标角度
	lock_angle = normalize_angle(yaw + angle_num);      // 当前角度 +angle_num
	
	if(angle_num!=0)
	{
		 turn_speed = 10;//用于设置旋转的初速度
	}
	else if(angle_num==0)
	{
		turn_speed = 0;
	}
	
	int turn_edge_speed = 5;//用于设置旋转的最值速度
	if(angle_num<0)
	{
		
			set_speed1 = -turn_speed;
			set_speed2 = turn_speed;
			set_speed3 = -turn_speed;
			set_speed4 = turn_speed;
			motor1_min = set_speed1 - turn_edge_speed; 
			motor1_max = set_speed1 + turn_edge_speed; 
			motor2_min = set_speed2 - turn_edge_speed; 
			motor2_max = set_speed2 + turn_edge_speed; 
			motor3_min = set_speed3 - turn_edge_speed; 
			motor3_max = set_speed3 + turn_edge_speed; 
			motor4_min = set_speed4 - turn_edge_speed; 
			motor4_max = set_speed4 + turn_edge_speed; 			
			tset_speed1 = set_speed1;
			tset_speed2 = set_speed2;
			tset_speed3 = set_speed3;
			tset_speed4 = set_speed4;	
			angle_k = 0.5;  // 控制转向灵敏度		    
			angle_turn_flag = 1;
		
		
		
	}
	else
	{
			set_speed1 = turn_speed;
			set_speed2 = -turn_speed;
			set_speed3 = turn_speed;
			set_speed4 = -turn_speed;
			//设置四个轮子初速度
			
			motor1_min = set_speed1 - turn_edge_speed; 
			motor1_max = set_speed1 + turn_edge_speed; 
			motor2_min = set_speed2 - turn_edge_speed; 
			motor2_max = set_speed2 + turn_edge_speed; 
			motor3_min = set_speed3 - turn_edge_speed; 
			motor3_max = set_speed3 + turn_edge_speed; 
			motor4_min = set_speed4 - turn_edge_speed; 
			motor4_max = set_speed4 + turn_edge_speed; 		
			tset_speed1 = set_speed1;
			tset_speed2 = set_speed2;
			tset_speed3 = set_speed3;
			tset_speed4 = set_speed4;			
			angle_k = 0.5;//灵敏度		    
			angle_turn_flag = 1;
	}
}



int move_speed = 0;
//----------------------------直行----------------------------//
void go_forward(float move_num)
{		
	if (move_num !=0)
	{
		 move_speed = 30;//用于设置前进的初速度
	}
	else if(move_num ==0)
	{
		move_speed = 0;
	}
	
	
	int move_edge_speed = 15;//用于设置前进的最值速度
	pulse_count=0;
	target_pulse=abs((standard_pulse*move_num)/100);
	
//	angle_k=0;
	if(move_num>0)
	{
		//根据麦克轮设置前进时轮子动向
			set_speed1 = move_speed;
			set_speed2 = move_speed;//
			set_speed3 = move_speed;
			set_speed4 = move_speed;
			motor1_min = set_speed1 - move_edge_speed; 
			motor1_max = set_speed1 + move_edge_speed; 
			motor2_min = set_speed2 - move_edge_speed; 
			motor2_max = set_speed2 + move_edge_speed; 
			motor3_min = set_speed3 - move_edge_speed; 
			motor3_max = set_speed3 + move_edge_speed; 
			motor4_min = set_speed4 - move_edge_speed; 
			motor4_max = set_speed4 + move_edge_speed; 		
			tset_speed1 = set_speed1;
			tset_speed2 = set_speed2;
			tset_speed3 = set_speed3;
			tset_speed4 = set_speed4;			
      angle_k = 0.5;
	}
	else
	{
			set_speed1 = -move_speed;
			set_speed2 = -move_speed;//
			set_speed3 = -move_speed;
			set_speed4 = -move_speed;
			motor1_min = set_speed1 - move_edge_speed; 
			motor1_max = set_speed1 + move_edge_speed; 
			motor2_min = set_speed2 - move_edge_speed; 
			motor2_max = set_speed2 + move_edge_speed; 
			motor3_min = set_speed3 - move_edge_speed; 
			motor3_max = set_speed3 + move_edge_speed; 
			motor4_min = set_speed4 - move_edge_speed; 
			motor4_max = set_speed4 + move_edge_speed; 	
			tset_speed1 = set_speed1;
			tset_speed2 = set_speed2;
			tset_speed3 = set_speed3;
			tset_speed4 = set_speed4;			
			angle_k = 0.5;
			
	}
	moving_flag=1;
}
int move_speed1=0;
int move_speed2=0;
//-------------------平移-------------------//
void go_left(float move_num)
{		
	if (move_num !=0)
	{                                                                                                                                                                                                                                                                                                                                                                                                     0;//用于设置前进的初速度
		move_speed2=15;//后轮
	}
	else if(move_num ==0)
	{
		move_speed1 = 0;
	}
	
	
	int move_edge_speed = 10;//用于设置前进的最值速度
	pulse_count=0;
	target_pulse=abs((standard_pulse*move_num)/100);
	
//	angle_k=0;
	if(move_num>0)
	{
		//根据麦克轮设置前进时轮子动向
			kp1=15;
			kp2=30;
			kp3=5;
			kp4=5;
			set_speed1 = -70;
			set_speed2 = 50;//
			set_speed3 = move_speed2;
			set_speed4 = -move_speed2;
			motor1_min = set_speed1 + move_edge_speed; 
			motor1_max = set_speed1 - move_edge_speed; 
			motor2_min = set_speed2 - move_edge_speed; 
			motor2_max = set_speed2 + move_edge_speed; 
			motor3_min = set_speed3 - move_edge_speed; 
			motor3_max = set_speed3 + move_edge_speed; 
			motor4_min = set_speed4 + move_edge_speed; 
			motor4_max = set_speed4 - move_edge_speed; 		
			tset_speed1 = set_speed1;
			tset_speed2 = set_speed2;
			tset_speed3 = set_speed3;
			tset_speed4 = set_speed4;		
			angle_k=0.9;
	}
	else
	{
			kp1=20;
			kp2=20;
			kp3=10;
			kp4=10;
			set_speed1 = 70;
			set_speed2 = -50;//
			set_speed3 = -move_speed2;
			set_speed4 = move_speed2;
			motor1_min = set_speed1 - move_edge_speed; 
			motor1_max = set_speed1 + move_edge_speed; 
			motor2_min = set_speed2 + move_edge_speed; 
			motor2_max = set_speed2 - move_edge_speed; 
			motor3_min = set_speed3 + move_edge_speed; 
			motor3_max = set_speed3 - move_edge_speed; 
			motor4_min = set_speed4 - move_edge_speed; 
			motor4_max = set_speed4 + move_edge_speed; 	
			tset_speed1 = set_speed1;
			tset_speed2 = set_speed2;
			tset_speed3 = set_speed3;
			tset_speed4 = set_speed4;			
			angle_k=0.9;
			
	}
	moving_flag=1;
}
//----------------------------
float global_angle = 0.0;
float received_angle = 0.0;
char angle_str[32];
float received_distance = 0;
volatile uint8_t camera_new_distance_ready = 0;
int obstacle_flag ;
//--------------------------
#define PULSES_PER_METER 12447u   // 1m 对应脉冲数（示例：你的项目中是 12447）


//------------------///
//新版备注
// ======= 全局变量 =======
// ======= 全局变量 =======
volatile int mission_state = 0;    // MOVE_BEGIN 的蛇形任务状态
volatile int mission_active = 0;   // 是否正在执行任务
int mission_cycle_complete = 0;

// ======= 全局变量 =======
volatile int mission_state = 0;    // MOVE_BEGIN 的蛇形任务状态
volatile int mission_active = 0;   // 是否正在执行任务
int mission_cycle_complete = 0;

// 避障相关变量
volatile uint8_t avoid_detected = 0;      // 是否检测到障碍物
volatile float avoid_distance = 100.0f;   // 障碍物距离
volatile uint8_t avoid_side = 0;          // 0=无, 1=左侧, 2=右侧

// ======= 避障检测函数 =======
void check_obstacle(void)
{
    static uint32_t last_check_time = 0;
    uint32_t current_time = systick_get_ms();
    
    // 每100ms检查一次避障
    if (current_time - last_check_time < 100) return;
    last_check_time = current_time;
    
    // 触发测量
    hc_sr04_trig_send();
    
    // 读取左侧避障模块距离
    if (distance_ready) {
        avoid_distance = distance_cm;
        distance_ready = 0;
        
        // 设置障碍物检测阈值 (15cm)
        if (avoid_distance < 15.0f && avoid_distance > 2.0f) {
            avoid_detected = 1;
            avoid_side = 1;  // 左侧检测到障碍物
        } else {
            avoid_detected = 0;
            avoid_side = 0;
        }
    }
}

// ======= 任务入口函数 =======
void handle_move_begin(void)
{
    static uint32_t move_start_time = 0;
    static uint32_t turn_start_time = 0;
    static uint8_t moving_forward = 0;
    
    check_obstacle();  // 检查障碍物
    
    switch (mission_state)
    {
        case 0: // 初始状态，开始前进
            go_forward(500);  // 设置一个较大的距离，实际由避障控制停止
            move_start_time = systick_get_ms();
            moving_forward = 1;
            mission_state = 1;
            break;

        case 1: // 前进中，检测障碍物
            if (avoid_detected && avoid_side == 1) {
                // 左侧检测到障碍物，停止前进并准备转弯
                go_forward(0);
                moving_forward = 0;
                mission_state = 2;
            }
            else if (systick_get_ms() - move_start_time > 10000) {
                // 10秒超时，防止卡死
                go_forward(0);
                mission_state = 0;  // 重新开始
            }
            break;

        case 2: // 向左转弯53度
            turn_angle(53);
            turn_start_time = systick_get_ms();
            mission_state = 3;
            break;

        case 3: // 检查转弯是否完成
            if (fabs(normalize_angle(lock_angle - yaw)) < 6.0f) {
                // 转弯完成
                go_forward(0);
                system_delay_ms(1000);
                mission_state = 4;
            }
            else if (systick_get_ms() - turn_start_time > 5000) {
                // 5秒转弯超时
                go_forward(0);
                mission_state = 0;  // 重新开始
            }
            break;

        case 4: // 前进一小段距离（0.5m等效）
            go_forward(100);
            move_start_time = systick_get_ms();
            moving_forward = 1;
            mission_state = 5;
            break;

        case 5: // 前进中，等待足够距离或时间
            if (pulse_count > target_pulse * 0.8 || 
                systick_get_ms() - move_start_time > 3000) {
                // 前进足够距离或时间，准备右转
                go_forward(0);
                moving_forward = 0;
                mission_state = 6;
            }
            break;

        case 6: // 向右转弯80度
            turn_angle(80);
            turn_start_time = systick_get_ms();
            mission_state = 7;
            break;

        case 7: // 检查转弯是否完成
            if (fabs(normalize_angle(lock_angle - yaw)) < 6.0f) {
                // 转弯完成
                go_forward(0);
                system_delay_ms(1000);
                mission_state = 8;
            }
            else if (systick_get_ms() - turn_start_time > 5000) {
                // 5秒转弯超时
                go_forward(0);
                mission_state = 0;  // 重新开始
            }
            break;

        case 8: // 再次前进，检测障碍物
            go_forward(500);
            move_start_time = systick_get_ms();
            moving_forward = 1;
            mission_state = 9;
            break;

        case 9: // 前进中，检测障碍物
            if (avoid_detected && avoid_side == 1) {
                // 左侧再次检测到障碍物，完成一个蛇形周期
                go_forward(0);
                moving_forward = 0;
                mission_state = 10;
            }
            else if (systick_get_ms() - move_start_time > 10000) {
                // 10秒超时
                go_forward(0);
                mission_state = 0;  // 重新开始
            }
            break;

        case 10: // 蛇形走位完成，调整最终角度
            turn_angle(-60);
            turn_start_time = systick_get_ms();
            mission_state = 11;
            break;

        case 11: // 检查最终角度调整是否完成
            if (fabs(normalize_angle(lock_angle - yaw)) < 6.0f) {
                // 调整完成
                go_forward(0);
                mission_cycle_complete = 1;
                mission_state = 0;
            }
            else if (systick_get_ms() - turn_start_time > 5000) {
                // 5秒超时
                go_forward(0);
                mission_cycle_complete = 1;
                mission_state = 0;
            }
            break;

        default:
            mission_state = 0;
            mission_active = 0;
            break;
    }
}


void main(void)
{
  clock_init(SYSTEM_CLOCK_600M);  
  debug_init();
  system_delay_ms(300);  	
	//初始化四个方向轮胎引脚
	gpio_init(C7, GPO, 0, GPO_PUSH_PULL);
	gpio_init(C9, GPO, 0, GPO_PUSH_PULL);
	pwm_init(PWM2_MODULE0_CHA_C6, 17000, 0);
	pwm_init(PWM2_MODULE1_CHA_C8, 17000, 0);
	gpio_init(C10, GPO, 0, GPO_PUSH_PULL);
	gpio_init(D2, GPO, 0, GPO_PUSH_PULL);
	pwm_init(PWM2_MODULE2_CHB_C11, 17000, 0);
	pwm_init(PWM2_MODULE3_CHB_D3, 17000, 0);

	encoder_quad_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1);
	encoder_quad_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24);
	encoder_quad_init(QTIMER2_ENCODER2, QTIMER2_ENCODER2_CH1_C5, QTIMER2_ENCODER2_CH2_C25);
	encoder_quad_init(QTIMER2_ENCODER1, QTIMER2_ENCODER1_CH1_C3, QTIMER2_ENCODER1_CH2_C4);
	gpio_init(C15, GPI, 0, GPI_PULL_UP);  // 或 GPI_PULL_NONE 取决于你电路
	gpio_init(C14, GPI, 0, GPI_PULL_UP);  // 或 GPI_PULL_NONE 取决于你电路
	gpio_init(C13, GPI, 0, GPI_PULL_UP);  // 或 GPI_PULL_NONE 取决于你电路
	gpio_init(C12, GPI, 0, GPI_PULL_UP);  // 或 GPI_PULL_NONE 取决于你电路
	interrupt_global_enable(0);
	pit_ms_init(PIT_CH0, 5); 
	
	pit_ms_init(PIT_CH1, 600); //600
	pit_disable(PIT_CH1); 	
		// 4. 启动定时器
	pit_enable(PIT_CH0);  // 启动5ms定时器
	pit_enable(PIT_CH1);  // 启动600ms定时器

	key_init(100);
	//--------------uart初始化

//	interrupt_init();  //初始化中断控制器
//	uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);
//	uart_rx_interrupt(UART_4, 1);  // 启用 UART4 接收中断
//	uart_init(UART_1, 9600, UART1_TX_B12, UART1_RX_B13);
//	
//	uart_rx_interrupt(UART_1, 1);  // 参数1: UART_1, 参数2: 1(开启)
////	NVIC_SetPriority(LPUART1_IRQn, 0);




	interrupt_init();  // 初始化中断控制器

		
		
	//			 // 2. 启用 UART4 接收中断（当接收到数据触发）
  LPUART_EnableInterrupts(LPUART4, kLPUART_RxDataRegFullInterruptEnable);
	NVIC_SetPriority(LPUART4_IRQn, 0);
	uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);
	uart_rx_interrupt(UART_4, 1);  // 启用 UART4 Rx中断
	//这里缺少了应该因为这个没有触发中断
//		 // 3. 启用 NVIC 中断通道（开启中断控制器里 UART4 的通道）	
	NVIC_EnableIRQ(LPUART4_IRQn);

	uart_init(UART_1, 9600, UART1_TX_B12, UART1_RX_B13);
	uart_rx_interrupt(UART_1, 1);  // 启用 UART1 Rx中断
	NVIC_SetPriority(LPUART1_IRQn, 1);
	NVIC_EnableIRQ(LPUART1_IRQn);

  //----------------避障初始化
	hcsr04_init();                    // 初始化 HC-SR04 模块
  
  ips200_init(IPS200_TYPE);
  systick_user_init();
	ips200_show_string(10,10,"ready");//表示初始化完成
	char dist_str[64];   // 用于格式化输出距离的字符串
  // 记录上一次按键状态（假设上拉逻辑，1 = 松开，0 = 按下）
	static uint8 key_prev[4] = {1, 1, 1, 1};
	system_delay_ms(1000);
	lock_angle=yaw;//初始化锁定的角度为一上电检测到的角度值
	int time_count_ang =0;
	int time_count_dis = 0;
	int time_count_while = 0;
	float dist;
	int angle_executed=0;
	// 在全局变量里增加
	volatile uint8_t avoid_stage = 0;  
	// 0 = 未避障
	// 1 = 左侧避障完成直行，正在右移
	// 2 = 右侧避障完成直行，正在左移
	// 3 = 右侧触发后，直行 + 左移
	// 前进一次的执行控制
	volatile uint8_t avoid_forward_active = 0;   // 0 = 未开始本次前进；1 = 前进进行中（已触发但未到达目标脉冲）
	volatile uint32_t avoid_start_pulse = 0;     // 记录开始前进时的 pulse_count
	volatile uint32_t avoid_needed_pulse = 100;    // 需要的脉冲数（从距离换算）
	//system_delay_ms(2000);
	
	int car_state = STATE_WAIT_MOVE;
	int prev_state = STATE_SEARCHING;    // 记录避障前的状态
	int search_dir = 1;                  // 1=向右平移, -1=向左平移
	float avoid_distance_cm = 8.0f;
	float forward_after_avoid_m = 50;
	float received_distance = 0;
   int time_angle =0;
	int panduan = 0;
	int tmp=0;
	
	
	int arrive_flag=0;
	int all_flag =0;
	int zhuanwan=0;
	int zhuanwan_st=0;

// ----------------- 主循环 -----------------
// ----------------- 主循环 -----------------
while (1)
{
    // … 你的按键逻辑保持不变 …
     if(arrive_flag==1) 
		 {
			 if(pulse_count>target_pulse)
			 {
					arrive_flag=0;
					uart_write_string(UART_4, "arrive");
				  go_forward(0);
			 }
		 }
		     // … 你的按键逻辑保持不变 …
     if(all_flag==1) 
		 {
			 
			 if(pulse_count>target_pulse)
			 {
					all_flag=0;
					uart_write_string(UART_4, "final");
					go_forward(0);
			 }
		 }
		 
			ips200_show_string(10,240,"angle:");
		ips200_show_int(60,240,yaw,6);
		ips200_show_string(10,260,"target_angle:");
ips200_show_int(180,260,lock_angle,6);
	 // 获取当前按键状态
		uint8 now[4] = {
				gpio_get_level(C12),  // 前进
				gpio_get_level(C13),  // 停止
				gpio_get_level(C14),  // 加速
				gpio_get_level(C15)   // 减速
		};

			key_scanner();
			if (key_prev[3] && !now[3])//模拟右转30°
		{
			mission_active=1;
			//turn_angle(55);
		}
			if (key_prev[2] && !now[2])//模拟右转30°
		{
       //turn_angle(95);
		}
			if (key_prev[1] && !now[1])//模拟右转30°
		{
       //turn_angle(-105);
		}
			if (key_prev[0] && !now[0])//模拟右转30°
		{
       //turn_angle(-65);
		}
	  // 保存当前状态用于下次判断边沿
   for (int i = 0; i < 4; i++) key_prev[i] = now[i];

		
		
//				if (mission_active ==1) {
//						handle_move_begin();  // 驱动蛇形走位任务
//					
//				}
// 主循环中
if (mission_active) {
    handle_move_begin();
    
    // 如果一轮循环完成，检查是否继续执行
    if (mission_cycle_complete) {
        mission_cycle_complete = 0;
        // 这里可以添加条件判断是否继续执行下一轮
        // 比如：if (continue_running) { mission_active = 1; }
        mission_active = 1;  // 自动开始下一轮
    }
}    
				// 角度转向误差闭环
				float angle_error = normalize_angle(lock_angle - yaw);
				if (angle_turn_flag) {
						if (fabs(angle_error) < 5.0f) {
								go_forward(0);
								angle_turn_flag = 0;
						}
				}
		// ======== UART 接收解析 ========
					if (uart4_rx_ready) {
				uart4_rx_ready = 0;

				if (strstr((char *)uart4_rx_buf, "MOVE_BEGIN")) {
						//car_state = STATE_SEARCHING;
						mission_active = 1;
						mission_state = 0;
						pulse_count = 0;
						handle_move_begin(); // 进入任务
						ips200_show_string(10, 70, "MOVE_BEGIN ");
				}
				else if (strstr((char *)uart4_rx_buf, "FIND")) {
						go_forward(0);
						//go_left(0);
						
						mission_active = 0;  // 中断蛇形任务
						ips200_show_string(10, 100, "FIND ");
				}
				else if (sscanf((char *)uart4_rx_buf, "DIST:%f", &received_distance) == 1) {
						mission_active = 0;  // 停掉蛇形
						//car_state = STATE_MOVING_TO_TARGET;
					system_delay_ms (2000);
						go_forward(received_distance * 100-10 );   // 转换成 cm
						sprintf(dist_str, "Dist_FIND: %5.2f m", received_distance);
						ips200_show_string(10, 130, dist_str);
						arrive_flag=1;
						
						//uart_write_string(UART_4, "arrive"); // 到达发送通知
				}
				else if (strstr((char *)uart4_rx_buf, "success")) {
						go_forward(0);
						turn_angle(180);               // 原地转圈（持续转）
						///car_state = STATE_RETURNING;
						
						ips200_show_string(10, 160, "success ");
				}
				else if (strstr((char *)uart4_rx_buf, "FIND_begin")) {
						go_forward(0);                 // 停下
						turn_angle(0);
						//car_state = STATE_FIND_BEGIN;
						mission_active = 0;  // 中断蛇形任务
						ips200_show_string(10, 190, "FIND_begin ");
				}
				else if (sscanf((char *)uart4_rx_buf, "DIST_begin:%f", &received_distance) == 1) 
					{
						//car_state = STATE_MOVING_TO_BEGIN;
					mission_active = 0;  // 停掉蛇形
						//car_state = STATE_MOVING_TO_TARGET;
					lock_angle = yaw;
					
						go_forward(received_distance * 100);   // 转换成 cm
					  //lock_angle = yaw;
					  
						sprintf(dist_str, "Dist_FIND_begin: %5.2f cm", received_distance);
						ips200_show_string(10, 220, dist_str);
						arrive_flag =1;
				}

				memset((void *)uart4_rx_buf, 0, sizeof(uart4_rx_buf));
		}

		
}

}
