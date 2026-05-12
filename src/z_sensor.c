#include <stdio.h>
#include <string.h>
#include "z_adc.h"
#include "z_global.h"
#include "z_gpio.h"
#include "z_timer.h"
#include "z_usart.h"
#include "z_main.h"
#include "z_delay.h"
#include "z_sensor.h"

int color_red_base, color_grn_base, color_blu_base;
uint8_t flagSoundStart=0;
static u8 xj_flag=0;
COLOR_RGBC color_rgbc;
COLOR_HSL color_hsl;
uint8_t is_tracking_updated = 0;		   // 循迹强制执行标志位
void car_move(int x, int w);
/*
	智能功能代码
*/

 // 循迹 左 A0 右 A1  
void setup_xunji(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//
void setup_csb() {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);  
	
	//初始化超声波IO口 Trig PB0  Echo PA2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure); 	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	
	
	//初始化超声波定时器
	TIM3_Int_Init(30000, 71);
}

//声音PB14  - TODO
void setup_sound() {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  
	
	//sound
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

//灰度传感器 PA1 AD
void setup_yssb() {
	TCS34725_Init(TCS34725_INTEGRATIONTIME_24MS);
}



//初始化传感器IO口
void setup_sensor(void) {
	setup_xunji();	//初始化循迹
	setup_sound();	//初始化声音
	setup_csb();	//初始化超声波
	setup_yssb();	//初始化颜色识别

}


static void bizhang_mokuai_reset(void);
static void ob_finish_and_resume(void);

//处理智能传感器功能
void loop_sensor(void) {
	static u8 AI_mode_bak;
	
	if(AI_mode == 0) {
		AI_xunji_moshi();				//循迹模式
	} else if(AI_mode == 1) {
		AI_shengkong_jiaqu();			//声控夹取
	} else if(AI_mode == 2) {
		AI_ziyou_bizhang();				//自由避障
	} else if(AI_mode == 3) {
		AI_yanse_shibie();				//颜色识别
	} else if(AI_mode == 4) {
		AI_dingju_jiaqu();				//定距夹取
	} else if(AI_mode == 5) {
		AI_gensui_moshi();				//跟随功能
	} else if(AI_mode == 6) {
		AI_xunji_bizhang();				//循迹避障
	} else if(AI_mode == 7) {
		AI_xunji_shibie();				//循迹识别
	} else if(AI_mode == 8) {
		AI_xunji_dingju();				//循迹定距
	} else if(AI_mode == 9) {	
		AI_shengkong_xunji();			//声控循迹
	} else if(AI_mode == 10) {
		AI_mode = 255;
	}
	
	if(AI_mode_bak != AI_mode) {
		if(AI_mode == 7) {
			bizhang_mokuai_reset();
		}
		AI_mode_bak = AI_mode;
		flagSoundStart=0;
		group_do_ok = 1;
	}

}


/*************************************************************
函数名称：get_adc_yssb_middle()
功能介绍：获取颜色传感器采集到的值并返回
函数参数：无
返回值：  采集的数据  
*************************************************************/
int get_adc_yssb_middle() {//读取AD值，排序，并返回中间值
	TCS34725_GetRawData(&color_rgbc);
	sprintf((char *)cmd_return, "R=%d G=%d B=%d C=%d\r\n",color_rgbc.r,color_rgbc.g,color_rgbc.b,color_rgbc.c);
	uart1_send_str(cmd_return);

	RGBtoHSL(&color_rgbc,&color_hsl);
	sprintf((char *)cmd_return, "H=%d S=%d L=%d\r\n",color_hsl.h,color_hsl.s,color_hsl.l);
	uart1_send_str(cmd_return);
	
	return 0;
}

void csb_Delay_Us(uint16_t time)  //延时函数
{ 
	uint16_t i,j;
	for(i=0;i<time;i++)
  		for(j=0;j<9;j++);
}
/*************************************************************
函数名称：get_csb_value()
功能介绍：采集超声波数据
函数参数：无
返回值：  采集的数据  
*************************************************************/
u16 get_csb_value(void) {
	u16 csb_t;
	Trig(1);
	csb_Delay_Us(20);
	Trig(0);
	while(Echo() == 0);      //等待接收口高电平输出
	TIM_SetCounter(TIM3,0);//清除计数
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
	while(Echo() == 1);
	TIM_Cmd(TIM3, DISABLE);  //使能TIMx外设      
	csb_t = TIM_GetCounter(TIM3);//获取时间,分辨率为1US
	//340m/s = 0.017cm/us
	if(csb_t < 25000) {
//		sprintf((char *)cmd_return, "csb_time=%d\r\n", (int)(csb_t*0.17));
//		uart1_send_str(cmd_return);
		csb_t = csb_t*0.017;
		return csb_t;
	}
	return 0;
}
/*************************************************************
函数名称：get_adc_csb_middle()
功能介绍：处理超声波采集到的数据，取采集到的中间值
函数参数：无
返回值：  处理后的超声波数据  
*************************************************************/

int get_adc_csb_middle() {
	u8 i;
	static int ad_value[5] = {0}, myvalue;// ad_value_bak[5] = {0}, 
	for(i=0;i<5;i++){ad_value[i] = get_csb_value();tb_delay_ms(5);}
	selection_sort(ad_value, 5);
	myvalue = ad_value[2];
// 	for(i=0;i<5;i++)ad_value[i] = ad_value_bak[i];
	return myvalue;  
}

/*************************************************************
函数名称：AI_xunji_moshi()
功能介绍：位置式PID循迹控制算法
硬件参数：
  - 黑线宽度：25mm
  - 传感器布局：x1 --27mm-- x2 --10mm-- x3 --27mm-- x4
  - 总跨度：64mm（x1到x4）
  - 检测到黑线输出0，未检测到输出1
  
算法原理：
  1. 根据传感器位置计算加权误差值
  2. 使用位置式PID算法计算转向修正量
  3. 通过差速控制实现巡线跟踪
  
定点数设计：
  - 缩放因子：1000（即1.0 = 1000）
  - 适用于STM32F1（无硬件FPU）
  - 提高运算效率，减少Flash占用
*************************************************************/

// 位置式PID宏定义
#define POS_PID_SCALE           1000
#define POS_PID_KP_INIT         5250
#define POS_PID_KI_INIT         0
#define POS_PID_KD_INIT         3000

#define POS_PID_INTEGRAL_MAX    50000
#define POS_PID_INTEGRAL_MIN    -50000
#define POS_PID_OUTPUT_MAX      25000
#define POS_PID_OUTPUT_MIN      -25000

#define POS_PID_BASE_SPEED      8
#define POS_PID_MOTOR_MAX       30
#define POS_PID_MOTOR_MIN       -30

#define POS_PID_SHARP_TURN_THRESHOLD    3
#define POS_PID_SHARP_TURN_CYCLES       8
#define POS_PID_SHARP_TURN_SPEED_L      -20//-15
#define POS_PID_SHARP_TURN_SPEED_R      25//20

#define POS_PID_DEBUG_ENABLE    0

// 位置式PID控制器结构体
typedef struct {
	int Kp;
	int Ki;
	int Kd;
	int error;
	int prev_error;
	int integral;
	int integral_max;
	int integral_min;
	int output_max;
	int output_min;
	int prev_derivative;
} Pos_PID_Controller;

// 状态变量
static int8_t s_pos_last_error = 0;
static uint8_t s_pos_sharp_turn_cycles = 0;
static uint8_t s_pos_sharp_turn_direction = 0;

// PID初始化
static void Pos_PID_Init(Pos_PID_Controller *pid, int kp, int ki, int kd) {
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->error = 0;
	pid->prev_error = 0;
	pid->integral = 0;
	pid->integral_max = POS_PID_INTEGRAL_MAX;
	pid->integral_min = POS_PID_INTEGRAL_MIN;
	pid->output_max = POS_PID_OUTPUT_MAX;
	pid->output_min = POS_PID_OUTPUT_MIN;
	pid->prev_derivative = 0;
}

// PID计算（位置式）
static int Pos_PID_Compute(Pos_PID_Controller *pid, int error, int dt_ms) {
	int P, I, D, output;
	int derivative, derivative_filtered;
	
	pid->error = error;
	
	P = (pid->Kp * pid->error) / POS_PID_SCALE;
	
	pid->integral += pid->error;
	if(pid->integral > pid->integral_max) pid->integral = pid->integral_max;
	if(pid->integral < pid->integral_min) pid->integral = pid->integral_min;
	I = (pid->Ki * pid->integral) / POS_PID_SCALE;
	
	derivative = (pid->error - pid->prev_error) * POS_PID_SCALE / dt_ms;
	derivative_filtered = (7 * derivative + 3 * pid->prev_derivative) / 10;
	D = (pid->Kd * derivative_filtered) / POS_PID_SCALE;
	pid->prev_derivative = derivative_filtered;
	
	output = P + I + D;
	
	if(output > pid->output_max) output = pid->output_max;
	if(output < pid->output_min) output = pid->output_min;
	
	pid->prev_error = pid->error;
	
	return output;
}

// PID重置
static void Pos_PID_Reset(Pos_PID_Controller *pid) {
	pid->error = 0;
	pid->prev_error = 0;
	pid->integral = 0;
	pid->prev_derivative = 0;
}

// 加权误差计算
static int Pos_Calc_Weighted_Error(void) {
	int error = 0;
	uint8_t x1_val, x2_val, x3_val, x4_val;
	
	x1_val = x1();
	x2_val = x2();
	x3_val = x3();
	x4_val = x4();
	
	if(x1_val == 0) error += 3;
	if(x2_val == 0) error += 1;
	if(x3_val == 0) error -= 1;
	if(x4_val == 0) error -= 3;
	
	return error;
}

// 直角弯处理
static void Pos_Handle_Sharp_Turn(int error) {
	if(s_pos_sharp_turn_cycles == 0) {
		if(error > 0) {
			s_pos_sharp_turn_direction = 2;
		} else {
			s_pos_sharp_turn_direction = 1;
		}
	}
	
	if(s_pos_sharp_turn_cycles < POS_PID_SHARP_TURN_CYCLES) {
		s_pos_sharp_turn_cycles++;
		if(s_pos_sharp_turn_direction == 1) {
			car_set(POS_PID_SHARP_TURN_SPEED_L, POS_PID_SHARP_TURN_SPEED_R);
		} else {
			car_set(POS_PID_SHARP_TURN_SPEED_R, POS_PID_SHARP_TURN_SPEED_L);
		}
		return;
	}
	
	s_pos_sharp_turn_cycles = 0;
	s_pos_sharp_turn_direction = 0;
}

void AI_xunji_moshi(void) {
	static Pos_PID_Controller track_pid;
	static uint8_t pid_initialized = 0;
	int error;
	int pid_output;
	int left_speed, right_speed;
	
	if(!pid_initialized) {
		Pos_PID_Init(&track_pid, POS_PID_KP_INIT, POS_PID_KI_INIT, POS_PID_KD_INIT);
		pid_initialized = 1;
	}
	
	error = Pos_Calc_Weighted_Error();
	
	if(error >= POS_PID_SHARP_TURN_THRESHOLD || error <= -POS_PID_SHARP_TURN_THRESHOLD) {
		Pos_Handle_Sharp_Turn(error);
		s_pos_last_error = error;
		return;
	}
	
	if(error == 0 && x1()==1 && x2()==1 && x3()==1 && x4()==1) {
		error = s_pos_last_error;
	}
	
	pid_output = Pos_PID_Compute(&track_pid, error * POS_PID_SCALE, 20);
	
	left_speed = POS_PID_BASE_SPEED - pid_output / POS_PID_SCALE;
	right_speed = POS_PID_BASE_SPEED + pid_output / POS_PID_SCALE;
	
	if(left_speed > POS_PID_MOTOR_MAX) left_speed = POS_PID_MOTOR_MAX;
	if(left_speed < POS_PID_MOTOR_MIN) left_speed = POS_PID_MOTOR_MIN;
	if(right_speed > POS_PID_MOTOR_MAX) right_speed = POS_PID_MOTOR_MAX;
	if(right_speed < POS_PID_MOTOR_MIN) right_speed = POS_PID_MOTOR_MIN;
	
	car_set(left_speed, right_speed);
	
	s_pos_last_error = error;
	
	#if POS_PID_DEBUG_ENABLE
	printf("err=%d pid=%d L=%d R=%d\n", error, pid_output/POS_PID_SCALE, left_speed, right_speed);
	#endif
}

/* 避障状态机：触发 -> 左转 -> 短直行离黑线 -> 差速圆弧绕障；圆弧中见黑结束避障并恢复循迹 */
#define OB_TRIG_DIST_MM           150
#define OB_TURN_LEFT_MS           350
#define OB_STRAIGHT_MS            600
#define OB_ARC_L                    16
#define OB_ARC_R                     8
#define OB_ARC_LINE_IGNORE_MS     600
#define OB_ARC_MAX_MS             4000

typedef enum {
	OB_ST_IDLE = 0,
	OB_ST_TURN_LEFT,
	OB_ST_STRAIGHT_OFF_LINE,
	OB_ST_ARC_AROUND
} ob_state_t;

static u8 s_ob_once_done = 0;
static ob_state_t s_ob_state = OB_ST_IDLE;
static u32 s_ob_phase_ms = 0;

static u8 ob_any_line(void) {
	return (x1() == 0 || x2() == 0 || x3() == 0 || x4() == 0) ? 1 : 0;
}

static void ob_finish_and_resume(void) {
	s_ob_once_done = 1;
	s_ob_state = OB_ST_IDLE;
	is_tracking_updated = 1;
}

void AI_bizhang_mokuai(void) {
	int adc_csb;

	if(s_ob_once_done) {
		return;
	}

	switch(s_ob_state) {
		case OB_ST_IDLE:
			adc_csb = get_adc_csb_middle();
			if(adc_csb > 0 && adc_csb < OB_TRIG_DIST_MM) {
				car_set(-18, 20);
				s_ob_state = OB_ST_TURN_LEFT;
				s_ob_phase_ms = millis();
			}
			break;

		case OB_ST_TURN_LEFT:
			car_set(-18, 20);
			if(millis() - s_ob_phase_ms >= OB_TURN_LEFT_MS) {
				car_set(12, 12);
				s_ob_state = OB_ST_STRAIGHT_OFF_LINE;
				s_ob_phase_ms = millis();
			}
			break;

		case OB_ST_STRAIGHT_OFF_LINE:
			car_set(12, 12);
			if(millis() - s_ob_phase_ms >= OB_STRAIGHT_MS) {
				s_ob_state = OB_ST_ARC_AROUND;
				s_ob_phase_ms = millis();
				is_tracking_updated = 1;
			}
			break;

		case OB_ST_ARC_AROUND:
			car_set(OB_ARC_L, OB_ARC_R);
			if(millis() - s_ob_phase_ms >= OB_ARC_LINE_IGNORE_MS && ob_any_line()) {
				ob_finish_and_resume();
				break;
			}
			if(millis() - s_ob_phase_ms >= OB_ARC_MAX_MS) {
				ob_finish_and_resume();
			}
			break;

		default:
			s_ob_state = OB_ST_IDLE;
			break;
	}
}

static u8 AI_bizhang_mokuai_busy(void) {
	if(s_ob_once_done) {
		return 0;
	}
	return (s_ob_state != OB_ST_IDLE) ? 1 : 0;
}

static void bizhang_mokuai_reset(void) {
	s_ob_once_done = 0;
	s_ob_state = OB_ST_IDLE;
	s_ob_phase_ms = 0;
}

/*************************************************************
函数名称：AI_xunji_bizhang()
功能介绍：在循迹的过程中，检测有障碍物则执行完整避障流程，避障优先级高于循迹
函数参数：无
返回值：  无  
*************************************************************/
void AI_xunji_bizhang(void) {
	static u32 systick_ms_bak = 0;
	int adc_csb;
	
	if(millis() - systick_ms_bak > 100) {
		systick_ms_bak = millis();
		
		if(AI_bizhang_mokuai_busy()) {
			AI_bizhang_mokuai();
		} else if(s_ob_once_done == 0) {
			adc_csb = get_adc_csb_middle();
			if(adc_csb > 0 && adc_csb < OB_TRIG_DIST_MM) {
				bizhang_mokuai_reset();
				AI_bizhang_mokuai();
			} else {
				AI_xunji_moshi();
			}
		} else {
			AI_xunji_moshi();
		}
	}
}

/*************************************************************
函数名称：AI_gensui_moshi()
功能介绍：检测物体距离，在一定距离内实现跟随功能
函数参数：无
返回值：  无  
*************************************************************/
void AI_gensui_moshi(void) {
	static u32 systick_ms_bak = 0;
	int speed = 15, adc_csb;
	if(millis() - systick_ms_bak > 100) {
		systick_ms_bak = millis();
		adc_csb = get_adc_csb_middle();//获取a0的ad值，计算出距离
		//sprintf((char *)uart_receive_buf, "adc_csb = %d\r\n", adc_csb);
		//uart1_send_str(uart_receive_buf);
		if((adc_csb > 30) && (adc_csb < 50)) {//距离30~50cm前进
			car_set(speed, speed);
		} else if(adc_csb < 20) {//距离低于20cm就后退
			car_set(-speed, -speed);
		} else {//其他情况停止
			car_set(0, 0);
		}
	}
}
/*************************************************************
函数名称：AI_ziyou_bizhang()
功能介绍：识别物体距离从而避开物体前进
函数参数：无
返回值：  无  
*************************************************************/
void AI_ziyou_bizhang(void) {
	static u32 systick_ms_bak = 0;
	static u8 ziyou_bizhangt_status =0,last_ziyou_bizhangt_status=0;
	int speed = 15, adc_csb;
	if(millis() - systick_ms_bak > 100) {
		systick_ms_bak = millis();
		adc_csb = get_adc_csb_middle();//获取a0的ad值，计算出距离
		//sprintf((char *)uart_receive_buf, "adc_csb = %d\r\n", adc_csb);
		//uart1_send_str(uart_receive_buf);
		if(adc_csb < 50) {//距离低于50cm就右转
			
			ziyou_bizhangt_status=1;
			
		} else {
			ziyou_bizhangt_status=2;
			
			
		}
		if (ziyou_bizhangt_status != last_ziyou_bizhangt_status ){
			switch (ziyou_bizhangt_status){
				case 1:
					car_set(speed, -speed);
					break;
			  case 2:
					
					car_set(speed, speed);
					break;
			}
		
		last_ziyou_bizhangt_status=ziyou_bizhangt_status;
		
		}
		
		
		
	}
}
/*************************************************************
函数名称：AI_dingju_jiaqu()
功能介绍：识别物体距离夹取物体
函数参数：无
返回值：  无  
*************************************************************/
void AI_dingju_jiaqu(void) {
	static u32 systick_ms_bak = 0;
	
	int adc_csb;
	if(group_do_ok == 0)return;

	//每20ms计算一次
	if(millis() - systick_ms_bak > 100) {
		systick_ms_bak = millis();
		adc_csb = get_adc_csb_middle();//获取a0的ad值，计算出距离
// 		sprintf((char *)uart_receive_buf, "adc_csb = %d\r\n", adc_csb);
// 		uart1_send_str(uart_receive_buf);
		if(adc_csb == 10) {//距离10cm左右就夹取
			car_set(0,0);
			beep_on_times(1, 100);
			parse_cmd((u8 *)"$DGT:3-11,1!");
			xj_flag=1;
		} 
	}
}
void AI_shengkong_jiaqu(void) {
	static u8 placeLeft = 0;
	
	if(group_do_ok == 0)return;//有动作执行，直接返回
	if(sound() == 0) {
		while(sound() == 0);
		if(placeLeft){
			parse_cmd((u8 *)"$DGT:19-27,1!");
			beep_on_times(1, 100);
		} else {
			parse_cmd((u8 *)"$DGT:28-36,1!");
			beep_on_times(2, 100);
		}
		placeLeft = !placeLeft;
	}
	
}
/*************************************************************
函数名称：AI_yanse_shibie()
功能介绍：识别木块颜色，夹取分别放到不同位置
函数参数：无
返回值：  无  
******************************************************** *****/
void AI_yanse_shibie() {
	static u32 systick_ms_yanse = 0;
  
  if (group_do_ok && millis() - systick_ms_yanse > 20) {
      systick_ms_yanse = millis();
		  TCS34725_GetRawData(&color_rgbc);//获取RGB
    	YSSB_LED(0);//关闭LED
    if (color_rgbc.c < 1) {
			//sprintf((char *)cmd_return, "R=%d G=%d B=%d C=%d\r\n",color_rgbc.r,color_rgbc.g,color_rgbc.b,color_rgbc.c);
	    //uart1_send_str(cmd_return);
			  car_set(0,0);
      	YSSB_LED(1);//打开LED
        tb_delay_ms(800);
        TCS34725_GetRawData(&color_rgbc);//获取RGB			
      if (color_rgbc.r > color_rgbc.g && color_rgbc.r  > color_rgbc.b ) {
        	car_set(0,0);
        	parse_cmd("$DGT:12-18,1!"); //执行脱机存储动作组

      } else if (color_rgbc.g > color_rgbc.r && color_rgbc.g  > color_rgbc.b) {
        	car_set(0,0);
        	parse_cmd("$DGT:19-27,1!"); //执行脱机存储动作组
				
      } else if (color_rgbc.b > color_rgbc.g && color_rgbc.b  > color_rgbc.r) {
        	car_set(0,0);
        	parse_cmd("$DGT:28-36,1!"); //执行脱机存储动作组
      }
      xj_flag=1;//识别之后为下次循迹做准备，即开始循迹再次判断条件
    }

  }
}

/*************************************************************
函数名称：AI_xunji_dingju()
功能介绍：在循迹的过程中实现定距夹取功能
函数参数：无
返回值：  无  
*************************************************************/
void AI_xunji_dingju() {
	if(group_do_ok == 1) {
		if(xj_flag==1) {
			is_tracking_updated = 1;
			xj_flag=0;
		}	
	  AI_xunji_moshi();
	}

	AI_dingju_jiaqu();
}
/*************************************************************
函数名称：AI_xunji_shibie()
功能介绍：在循迹的过程中实现颜色识别功能，并将颜色模块移开
函数参数：无
返回值：  无  
*************************************************************/
typedef enum {
	XJ_COLOR_NONE = 0,
	XJ_COLOR_RED,
	XJ_COLOR_GREEN,
	XJ_COLOR_BLUE
} xj_color_t;

typedef enum {
	XJ_TASK_SEARCH_BLOCK = 0,   // 循迹+找木块
	XJ_TASK_WAIT_GRAB,          // 等待夹取动作完成
	XJ_TASK_TO_CROSS,           // 携带木块去十字路口
	XJ_TASK_BRANCH_TURN,        // 十字路口分流转向
	XJ_TASK_TO_DROP_LINE,       // 分流后循迹，找投放黑线
	XJ_TASK_WAIT_DROP           // 等待放置动作完成
} xj_task_t;

#define XJ_FULL_BLACK_DEBOUNCE     2
#define XJ_BRANCH_TURN_MS          320
#define XJ_DROP_LINE_IGNORE_MS_RGB 700
#define XJ_DROP_LINE_IGNORE_MS_GRN 350

static xj_color_t detect_block_color_once(void) {
	TCS34725_GetRawData(&color_rgbc);
	YSSB_LED(0);
	if(color_rgbc.c >= 1) {
		return XJ_COLOR_NONE;
	}

	car_set(0,0);
	YSSB_LED(1);
	tb_delay_ms(800);
	TCS34725_GetRawData(&color_rgbc);
	YSSB_LED(0);

	if(color_rgbc.r >= color_rgbc.g && color_rgbc.r >= color_rgbc.b) {
		return XJ_COLOR_RED;
	} else if(color_rgbc.g >= color_rgbc.r && color_rgbc.g >= color_rgbc.b) {
		return XJ_COLOR_GREEN;
	} else if(color_rgbc.b >= color_rgbc.g && color_rgbc.b >= color_rgbc.r) {
		return XJ_COLOR_BLUE;
	}

	return XJ_COLOR_NONE;
}

void AI_xunji_shibie(void) {	
	static xj_task_t task_state = XJ_TASK_SEARCH_BLOCK;
	static xj_color_t carry_color = XJ_COLOR_NONE;
	static u32 state_time_ms = 0;
	static u32 color_check_ms = 0;
	static u8 full_black_count = 0;

	u8 full_black = (x1() == 0 && x2() == 0 && x3() == 0 && x4() == 0);

	switch(task_state) {
		case XJ_TASK_SEARCH_BLOCK:
			AI_bizhang_mokuai();
			if(!AI_bizhang_mokuai_busy() && group_do_ok == 1) {
				AI_xunji_moshi();
			}

			if(!AI_bizhang_mokuai_busy() && millis() - color_check_ms > 80 && group_do_ok == 1) {
				xj_color_t detected = XJ_COLOR_NONE;
				color_check_ms = millis();
				detected = detect_block_color_once();
				if(detected != XJ_COLOR_NONE) {
					carry_color = detected;
					parse_cmd((u8 *)"$DGT:12-15,1!");
					task_state = XJ_TASK_WAIT_GRAB;
					state_time_ms = millis();
					full_black_count = 0;
				}
			}
			break;

		case XJ_TASK_WAIT_GRAB:
			if(group_do_ok == 1) {
				task_state = XJ_TASK_TO_CROSS;
				state_time_ms = millis();
				full_black_count = 0;
				is_tracking_updated = 1;
			}
			break;

		case XJ_TASK_TO_CROSS:
			AI_bizhang_mokuai();
			if(!AI_bizhang_mokuai_busy()) {
				AI_xunji_moshi();
			}
			if(full_black) {
				full_black_count++;
			} else {
				full_black_count = 0;
			}

			if(full_black_count >= XJ_FULL_BLACK_DEBOUNCE) {
				task_state = XJ_TASK_BRANCH_TURN;
				state_time_ms = millis();
				full_black_count = 0;
			}
			break;

		case XJ_TASK_BRANCH_TURN:
			if(carry_color == XJ_COLOR_RED) {
				car_set(-18, 20);
			} else if(carry_color == XJ_COLOR_BLUE) {
				car_set(20, -18);
			} else {
				car_set(10, 10);
			}

			if(millis() - state_time_ms > XJ_BRANCH_TURN_MS) {
				task_state = XJ_TASK_TO_DROP_LINE;
				state_time_ms = millis();
				full_black_count = 0;
				is_tracking_updated = 1;
			}
			break;

		case XJ_TASK_TO_DROP_LINE:
		{
			u32 drop_line_ignore_ms = XJ_DROP_LINE_IGNORE_MS_RGB;
			AI_bizhang_mokuai();
			if(!AI_bizhang_mokuai_busy()) {
				AI_xunji_moshi();
			}
			if(carry_color == XJ_COLOR_GREEN) {
				drop_line_ignore_ms = XJ_DROP_LINE_IGNORE_MS_GRN;
			}
			if(millis() - state_time_ms < drop_line_ignore_ms) {
				break;
			}

			if(full_black) {
				full_black_count++;
			} else {
				full_black_count = 0;
			}

			if(full_black_count >= XJ_FULL_BLACK_DEBOUNCE) {
				car_set(0,0);
				parse_cmd((u8 *)"$DGT:17-18,1!");
				task_state = XJ_TASK_WAIT_DROP;
				state_time_ms = millis();
				full_black_count = 0;
			}
			break;
		}

		case XJ_TASK_WAIT_DROP:
			if(group_do_ok == 1) {
				car_set(0,0);
				carry_color = XJ_COLOR_NONE;
				task_state = XJ_TASK_SEARCH_BLOCK;
				state_time_ms = millis();
				full_black_count = 0;
				xj_flag = 1;
				is_tracking_updated = 0;
				AI_mode = 255;
			}
			break;

		default:
			task_state = XJ_TASK_SEARCH_BLOCK;
			carry_color = XJ_COLOR_NONE;
			full_black_count = 0;
			break;
	}
}

/*************************************************************
函数名称：AI_shengkong_xunji()
功能介绍：声控循迹函数
函数参数：无
返回值：  无  
*************************************************************/
void AI_shengkong_xunji(void) {
	
	if(flagSoundStart==0){
		if(sound() == 0) {
			while(sound() == 0);
			flagSoundStart=1;
			is_tracking_updated = 1;	
		}
	}
	if(flagSoundStart) {
	  //flagSoundStart=0;
		AI_xunji_moshi();
	}
}

