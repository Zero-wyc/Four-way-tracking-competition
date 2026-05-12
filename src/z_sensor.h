#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f10x_conf.h"
#include "z_color.h"	//颜色拾取传感器


#define XJ_ON 0
#define XJ_OFF 1
#define x1() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)
#define x2() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#define x3() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)
#define x4() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)

#define Trig(x) gpioB_pin_set(0, x);
#define Echo() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)

#define sound() GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)

#define ADC_YSSB	0	//A5 数组所在位置是0 
#define COLOR_RED 215	//红色基准色
#define COLOR_GRN 310	//蓝色基准色
#define COLOR_BLU 283	//绿色基准色
#define YSSB_LED(x) TCS34725_LedON(x); //颜色识别的LED灯

#define COLOR_VERIFY   0x42
#define COLOR_RED_BASE 119 //红色基准色
#define COLOR_GRN_BASE 135 //绿色基准色
#define COLOR_BLU_BASE 141 //蓝色基准色

extern uint8_t is_tracking_updated;		   // 循迹强制执行标志位
extern uint8_t flagSoundStart;


//处理智能传感器功能
void setup_sensor(void);	//初始化所有传感器
void loop_sensor(void);		//传感器大循环

void AI_xunji_moshi(void); 			//循迹功能
void AI_shengkong_jiaqu(void);		//静态声音识别夹取
void AI_yanse_shibie(void);			//静态颜色识别夹取
void AI_xunji_shibie(void);			//循迹颜色夹取
void AI_dingju_jiaqu(void);			//静态超声波夹取
void AI_xunji_bizhang(void);		//循迹超声波避障
void AI_gensui_moshi(void);			//超声波跟随功能
void AI_ziyou_bizhang(void);		//超声波自由避障
void AI_xunji_dingju(void);			//循迹超声波夹取
void AI_shengkong_xunji(void);		//声控循迹

#endif


