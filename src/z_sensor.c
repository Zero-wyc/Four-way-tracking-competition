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
#include "z_sorting.h"
#include "z_tracking.h"

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
	
	tracking_init(); //初始化巡线模块

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
	} else if(AI_mode == 11) {
		// 自动分拣模式 - 红色
		if (!sorting_is_busy()) {
			sorting_start(PART_RED);
		}
		sorting_task();
	} else if(AI_mode == 12) {
		// 自动分拣模式 - 绿色
		if (!sorting_is_busy()) {
			sorting_start(PART_GREEN);
		}
		sorting_task();
	} else if(AI_mode == 13) {
		// 自动分拣模式 - 蓝色
		if (!sorting_is_busy()) {
			sorting_start(PART_BLUE);
		}
		sorting_task();
	} else if(AI_mode == 14) {
		// 自动分拣模式 - 自动识别颜色
		if (!sorting_is_busy()) {
			sorting_start(PART_RED);  // 传入任意颜色，实际按检测分类
		}
		sorting_task();
	} else if(AI_mode == 15) {
		// 发送统计信息
		sorting_send_stats();
		AI_mode = 255;  // 执行一次后退出
	} else if(AI_mode == 16) {
		// 停止分拣
		sorting_stop();
		AI_mode = 255;
	} else if(AI_mode == 20) {
		// 颜色传感器白平衡校准
		sorting_color_calibrate();
		AI_mode = 255;
	} else if(AI_mode == 21) {
		// 颜色识别准确率测试
		sorting_test_color_accuracy();
		AI_mode = 255;
	} else if(AI_mode == 22) {
		// 机械臂定位精度测试
		sorting_test_arm_precision();
		AI_mode = 255;
	} else if(AI_mode == 23) {
		// 系统自检
		sorting_run_self_test();
		AI_mode = 255;
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

// ========== 修改：AI_xunji_moshi（增强版循迹函数）==========

/*************************************************************
 * 函数名称：AI_xunji_moshi
 * 功能介绍：新版循迹主函数（使用z_tracking模块）
 * 参数：无
 * 返回值：无
 * 说明：调用tracking_update()实现所有功能
 *************************************************************/
void AI_xunji_moshi(void)
{
    tracking_update();
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

// ========== 新增：单元测试函数 ==========

/*************************************************************
 * 函数名称：test_tracking_position_table
 * 功能介绍：测试位置查表的正确性
 *************************************************************/
void test_tracking_position_table(void)
{
    // 测试用例结构体定义
    struct TestCase {
        uint8_t s1, s2, s3, s4;
        int8_t expected;
        char *desc;
    };
    
    // 测试用例数组（C89标准：初始化必须在声明时）
    struct TestCase test_cases[] = {
        {1,0,0,1, 0, "Center"},
        {1,0,1,1, 5, "Slight Left"},
        {1,1,0,1, -5, "Slight Right"},
        {0,0,1,1, 20, "Left"},
        {1,1,0,0, -20, "Right"},
        {0,1,1,1, 15, "More Left"},
        {1,1,1,0, -15, "More Right"},
        {1,1,1,1, 99, "All White"},
        {0,0,0,0, 100, "All Black"},
    };
    
    // 变量声明（C89标准：必须在代码块开头）
    uint8_t pass = 0, fail = 0;
    uint8_t i;
    int8_t result;
    uint8_t ok;
    
    uart1_send_str((u8 *)"\r\n=== Position Table Test ===\r\n");
    
    for (i = 0; i < 9; i++) {
        result = tracking_get_position(
            test_cases[i].s1,
            test_cases[i].s2,
            test_cases[i].s3,
            test_cases[i].s4
        );
        
        ok = (result == test_cases[i].expected);
        if (ok) pass++; else fail++;
        
        sprintf((char*)cmd_return, "%s: %d%d%d%d -> %d (exp:%d) [%s]\r\n",
            test_cases[i].desc,
            test_cases[i].s1, test_cases[i].s2,
            test_cases[i].s3, test_cases[i].s4,
            result, test_cases[i].expected,
            ok ? "PASS" : "FAIL");
        uart1_send_str(cmd_return);
    }
    
    sprintf((char*)cmd_return, "Result: PASS=%d, FAIL=%d\r\n", pass, fail);
    uart1_send_str(cmd_return);
}

/*************************************************************
 * 函数名称：test_pid_controller
 * 功能介绍：测试PID控制器的响应
 *************************************************************/
void test_pid_controller(void)
{
    /* 变量声明（C89标准：必须在代码块开头） */
    int16_t positions[] = {-20, -15, -10, -5, 0, 0, 0, 5, 10, 5, 0};
    uint8_t num = sizeof(positions) / sizeof(positions[0]);
    uint8_t i;
    int16_t base;
    
    uart1_send_str((u8 *)"\r\n=== PID Controller Test ===\r\n");
    uart1_send_str((u8 *)"Pos\tBaseSpeed\r\n");
    
    for (i = 0; i < num; i++) {
        /* 使用新的tracking_pid_calc接口 */
        base = tracking_pid_calc(0, positions[i]);
        
        sprintf((char*)cmd_return, "%d\t%d\r\n",
            positions[i],
            base);
        uart1_send_str(cmd_return);
        
        /* 模拟10ms延迟 */
        tb_delay_ms(10);
    }
}

/*************************************************************
 * 函数名称：test_speed_mapping
 * 功能介绍：测试速度映射的正确性
 *************************************************************/
void test_speed_mapping(void)
{
    // 变量声明（C89标准：必须在代码块开头）
    int16_t pid_out;
    int16_t left, right;
    char *action;
    
    uart1_send_str((u8 *)"\r\n=== Speed Mapping Test ===\r\n");
    uart1_send_str((u8 *)"PID\tLeft\tRight\tAction\r\n");
    
    for (pid_out = -80; pid_out <= 80; pid_out += 20) {
        tracking_calc_speed(pid_out, 12, &left, &right);
        
        if (pid_out > 5) action = "Turn Left";
        else if (pid_out < -5) action = "Turn Right";
        else action = "Straight";
        
        sprintf((char*)cmd_return, "%d\t%d\t%d\t%s\r\n",
            pid_out, left, right, action);
        uart1_send_str(cmd_return);
    }
}

/*************************************************************
 * 函数名称：test_speed_smoothing
 * 功能介绍：测试速度平滑效果
 *************************************************************/
void test_speed_smoothing(void)
{
    // 变量声明（C89标准：必须在代码块开头）
    int16_t current = 0;
    int16_t targets[] = {15, 15, 15, 5, 5, -10, -10, 0};
    uint8_t num = sizeof(targets) / sizeof(targets[0]);
    uint8_t i;
    int16_t smoothed;
    
    uart1_send_str((u8 *)"\r\n=== Speed Smoothing Test ===\r\n");
    uart1_send_str((u8 *)"Target\tCurrent\tSmoothed\r\n");
    
    for (i = 0; i < num; i++) {
        smoothed = tracking_smooth_speed(targets[i], current);
        
        sprintf((char*)cmd_return, "%d\t%d\t%d\r\n",
            targets[i], current, smoothed);
        uart1_send_str(cmd_return);
        
        current = smoothed;
    }
}

/*************************************************************
 * 函数名称：run_all_tests
 * 功能介绍：运行所有单元测试
 *************************************************************/
void run_all_tests(void)
{
    uart1_send_str((u8 *)"\r\n========== Tracking Algorithm Tests ==========\r\n");
    
    test_tracking_position_table();
    tb_delay_ms(100);
    
    test_pid_controller();
    tb_delay_ms(100);
    
    test_speed_mapping();
    tb_delay_ms(100);
    
    test_speed_smoothing();
    tb_delay_ms(100);
    
    uart1_send_str((u8 *)"\r\n========== Tests Complete ==========\r\n");
}

