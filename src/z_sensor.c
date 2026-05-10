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
		//if(AI_mode == 3)
		   //car_set(0, 0);
		//else 
			 
		
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
功能介绍：实现循迹功能
函数参数：无
返回值：  无
*************************************************************/

static uint8_t s_tracking_status = 1;	   // 当前循迹状态
static uint8_t s_last_tracking_status = 0; // 上一次循迹状态

#define TRACKING_STATUS_FORWARD 1	 // 直行
#define TRACKING_STATUS_TURN_LEFT 2	 // 左转
#define TRACKING_STATUS_TURN_RIGHT 3 // 右转
#define TRACKING_STATUS_STOP 4		 // 停止
// 检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW/0
// 未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH/1


void AI_xunji_moshi(void)
{
	uint8_t x1, x2, x3, x4;

	//	sprintf((char *)cmd_return, "s_tracking_status=%d s_last_tracking_status = %d \r\n",s_tracking_status,s_last_tracking_status);
	//	uart1_send_str(cmd_return);
	// 获取传感器值
	x1 = x1();
	x2 = x2();
	x3 = x3();
	x4 = x4();

	// 根据传感器值判断循迹状态
//	if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 1) // 中间检测到黑线
//	{
//		s_tracking_status = TRACKING_STATUS_FORWARD;
//	}
//	else if ((x3 == 0 && x2 == 1) || (x2 == 1 && x1 == 1 && x4 == 0)) // 右边出去，左转
//	{
//		s_tracking_status = TRACKING_STATUS_TURN_LEFT;
//	}
//	else if ((x3 == 1 && x2 == 0) || (x3 == 1 && x1 == 0 && x4 == 1)) // 左边出去，右转
//	{
//		s_tracking_status = TRACKING_STATUS_TURN_RIGHT;
//	}
//	else if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0) // 所有传感器都检测到黑线
//	{
//		s_tracking_status = TRACKING_STATUS_FORWARD;
//	}
//	else // 其他情况，默认直行
//	{
//		s_tracking_status = TRACKING_STATUS_FORWARD;
//	}

  if ((x4 == 0) || (x2 == 1 && x3 == 0)) // 右边出去，左转
	{
		s_tracking_status = TRACKING_STATUS_TURN_LEFT;
	}
	else if ((x1 == 0)||(x2 == 0 && x3 == 1))   // 左边出去，右转
	{
		s_tracking_status = TRACKING_STATUS_TURN_RIGHT;
	}
  else if ((x1 == 1)&&(x2 == 1) && (x3 == 1)&& (x4 == 1))   // 左边出去，右转
	{
		s_tracking_status = TRACKING_STATUS_STOP;
	}
	else if (x2 == 0 && x3 == 0 ) // 中间检测到黑线
	{
		s_tracking_status = TRACKING_STATUS_FORWARD;
	}
		 
	 
	
//	else if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0) // 所有传感器都检测到黑线
//	{
//		s_tracking_status = TRACKING_STATUS_FORWARD;
//	}
	else // 其他情况，默认直行
	{
		//s_tracking_status = TRACKING_STATUS_FORWARD;
	}
	// 如果状态发生变化，则更新动作
	if (s_tracking_status != s_last_tracking_status || is_tracking_updated)
	{

		switch (s_tracking_status)
		{
		case TRACKING_STATUS_FORWARD:
			
			car_set(11,11); // 默认直行
			break;
		case TRACKING_STATUS_TURN_LEFT:
			car_set(-15,17); // 左转
			break;
		case TRACKING_STATUS_TURN_RIGHT:
			car_set(17,-15); // 右转
			break;
		case TRACKING_STATUS_STOP:
			car_set(0,0); // 停止
			break;
		}
		s_last_tracking_status = s_tracking_status; // 更新上一次状态
		is_tracking_updated = 0;					// 强制执行标志位刷新
	}
}
/*************************************************************
函数名称：AI_xunji_bizhang()
功能介绍：在循迹的过程中，检测有障碍物，则停止，否则继续循迹
函数参数：无
返回值：  无  
*************************************************************/
void AI_xunji_bizhang(void) {
	static u32 systick_ms_bak = 0;
	static bool flag=0;
	int adc_csb;
	
	if(millis() - systick_ms_bak > 100) {
		systick_ms_bak = millis();
		//避障处理
		adc_csb = get_adc_csb_middle();//获取a0的ad值，计算出距离
		//sprintf((char *)uart_receive_buf, "adc_csb = %d\r\n", adc_csb);
		//uart1_send_str(uart_receive_buf);
		if(adc_csb < 30) {//距离低于30mm就停止
			car_set(0, 0);
			mdelay(100);
			flag=1;
		} else if(flag==1){
		  is_tracking_updated = 1;
		  flag=0;
		}else {
			//循迹处理
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
void AI_xunji_shibie(void) {	
	if(group_do_ok == 1) {
		if(xj_flag==1) {
			is_tracking_updated = 1;
			xj_flag=0;
		}	
		AI_xunji_moshi();
	}
	AI_yanse_shibie();
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

