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

// ========== 新增：加权循迹算法全局变量 ==========

// 位置查表（16种传感器状态对应的位置估计值，×10放大）
// 索引：s1(bit3) | s2(bit2) | s3(bit1) | s4(bit0)
// 0=检测到黑线(低电平)，1=白色(高电平)
static const int8_t g_position_table[16] = {
    /* 0000 全黑 */  POS_ALL_BLACK,  // 100，特殊标记
    /* 0001     */   30,             // 仅右外
    /* 0010     */   12,             // 仅右内
    /* 0011     */   20,             // 右内+右外
    /* 0100     */  -12,             // 仅左内
    /* 0101     */    0,             // 左内+右内（理论不可能）
    /* 0110     */    0,             // 左内+右内（居中）
    /* 0111     */   15,             // 左内+右内+右外
    /* 1000     */  -30,             // 仅左外
    /* 1001     */  -20,             // 左外+右外（理论不可能）
    /* 1010     */  -10,             // 左外+右内
    /* 1011     */    5,             // 左外+右内+右外
    /* 1100     */  -20,             // 左外+左内
    /* 1101     */   -5,             // 左外+左内+右外
    /* 1110     */  -15,             // 左外+左内+右内
    /* 1111 全白*/   POS_ALL_WHITE   // 99，脱线
};

// PID控制器实例
static PID_TypeDef g_pid = {
    PID_KP_MID_ERR,  // Kp
    PID_KI,          // Ki
    PID_KD_MID_ERR,  // Kd
    0, 0, 0, 0       // err, err_last, integral, output
};

// 当前平滑后的速度
static int16_t g_current_left_speed = 0;
static int16_t g_current_right_speed = 0;

// 标记检测状态
static uint32_t g_black_start_time = 0;
static uint8_t g_black_flag = 0;
static TrackMark_t g_last_mark = MARK_NONE;

// 脱线恢复状态
static uint8_t g_lost_flag = 0;
static uint8_t g_lost_counter = 0;
static int8_t g_last_valid_dir = 0;  // 最后已知方向

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
 * 函数名称：tracking_get_position
 * 功能介绍：根据四路传感器值计算加权位置估计
 * 参数：s1~s4 - 传感器值（0=黑线，1=白色）
 * 返回：位置估计值（×10放大，-30~+30），特殊值99/100
 * 算法说明：使用预计算查表法，避免浮点运算，适合嵌入式
 *************************************************************/
int8_t tracking_get_position(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4)
{
    uint8_t idx = SENSOR_IDX(s1, s2, s3, s4);
    return g_position_table[idx];
}

/*************************************************************
 * 函数名称：tracking_pid_calc
 * 功能介绍：自适应PID控制器计算
 * 参数：pid - PID结构体指针
 *       setpoint - 目标值（0，表示居中）
 *       measured - 当前测量值（位置估计）
 * 返回：当前基础速度（根据误差大小自适应）
 * 算法说明：根据误差绝对值自动选择PID参数分段
 *************************************************************/
int16_t tracking_pid_calc(PID_TypeDef *pid, int16_t setpoint, int16_t measured)
{
    // 1. 计算误差
    pid->err = setpoint - measured;
    
    // 2. 根据误差绝对值选择参数
    int16_t err_abs = (pid->err >= 0) ? pid->err : -(pid->err);
    int16_t base_speed;
    
    if (err_abs < 5) {
        // 小误差：追求平稳，高速
        pid->Kp = PID_KP_SMALL_ERR;
        pid->Kd = PID_KD_SMALL_ERR;
        base_speed = SPEED_STRAIGHT;
    } else if (err_abs < 15) {
        // 中等误差：标准响应，中速
        pid->Kp = PID_KP_MID_ERR;
        pid->Kd = PID_KD_MID_ERR;
        base_speed = SPEED_CURVE;
    } else {
        // 大误差：快速纠偏，低速
        pid->Kp = PID_KP_LARGE_ERR;
        pid->Kd = PID_KD_LARGE_ERR;
        base_speed = SPEED_SLOW;
    }
    
    // 3. 计算PID三项
    // 比例项
    int16_t P = (pid->Kp * pid->err) / 10;
    
    // 积分项（带限幅）
    pid->integral += pid->err;
    if (pid->integral > PID_INTEGRAL_MAX) pid->integral = PID_INTEGRAL_MAX;
    if (pid->integral < -PID_INTEGRAL_MAX) pid->integral = -PID_INTEGRAL_MAX;
    int16_t I = (PID_KI * pid->integral) / 100;
    
    // 微分项
    int16_t D = (pid->Kd * (pid->err - pid->err_last)) / 10;
    pid->err_last = pid->err;
    
    // 4. 合成输出
    pid->output = P + I + D;
    
    // 5. 输出限幅
    if (pid->output > PID_OUTPUT_MAX) pid->output = PID_OUTPUT_MAX;
    if (pid->output < -PID_OUTPUT_MAX) pid->output = -PID_OUTPUT_MAX;
    
    return base_speed;
}

/*************************************************************
 * 函数名称：tracking_calc_speed
 * 功能介绍：根据PID输出计算左右轮目标速度
 * 参数：pid_output - PID控制器输出
 *       base_speed - 基础速度
 *       left_speed/right_speed - 输出速度指针
 * 算法说明：差速转向，PID>0左转（左轮慢），PID<0右转（右轮慢）
 *************************************************************/
void tracking_calc_speed(int16_t pid_output, int16_t base_speed,
                         int16_t *left_speed, int16_t *right_speed)
{
    // PID输出 > 0：偏右，需要左转 -> 左轮减速，右轮加速
    // PID输出 < 0：偏左，需要右转 -> 左轮加速，右轮减速
    
    *left_speed  = base_speed - (pid_output * STEER_FACTOR) / 10;
    *right_speed = base_speed + (pid_output * STEER_FACTOR) / 10;
    
    // 限幅保护
    if (*left_speed > 20) *left_speed = 20;
    if (*left_speed < -20) *left_speed = -20;
    if (*right_speed > 20) *right_speed = 20;
    if (*right_speed < -20) *right_speed = -20;
}

/*************************************************************
 * 函数名称：tracking_smooth_speed
 * 功能介绍：速度平滑处理，防止电机速度突变
 * 参数：target - 目标速度
 *       current - 当前速度
 * 返回：平滑后的速度
 * 算法说明：限制每周期速度变化量，类似"软启动"
 *************************************************************/
int16_t tracking_smooth_speed(int16_t target, int16_t current)
{
    int16_t diff = target - current;
    
    if (diff > SPEED_MAX_CHANGE) {
        return current + SPEED_MAX_CHANGE;
    } else if (diff < -SPEED_MAX_CHANGE) {
        return current - SPEED_MAX_CHANGE;
    }
    return target;
}

/*************************************************************
 * 函数名称：tracking_detect_mark
 * 功能介绍：检测赛道上的特殊标记（全黑区域）
 * 参数：s1~s4 - 传感器值
 * 返回：标记类型枚举
 * 算法说明：通过全黑持续时间区分不同尺寸的标记
 *************************************************************/
TrackMark_t tracking_detect_mark(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4)
{
    uint8_t is_all_black = (s1==0 && s2==0 && s3==0 && s4==0);
    uint32_t current_time = millis();
    
    if (is_all_black && !g_black_flag) {
        // 首次检测到全黑
        g_black_flag = 1;
        g_black_start_time = current_time;
        return MARK_NONE;
        
    } else if (is_all_black && g_black_flag) {
        // 持续全黑，根据时间判断类型
        uint32_t duration = current_time - g_black_start_time;
        
        if (duration >= MARK_40X40_MIN_MS) {
            return MARK_40X40;
        } else if (duration >= MARK_15X15_MIN_MS) {
            return MARK_15X15;
        } else if (duration >= MARK_CROSS_MS) {
            return MARK_CROSS;
        } else if (duration >= MARK_5X5_MIN_MS) {
            return MARK_5X5;
        }
        
    } else if (!is_all_black && g_black_flag) {
        // 全黑结束
        uint32_t duration = current_time - g_black_start_time;
        g_black_flag = 0;
        
        // 短脉冲可能是5x5标记
        if (duration >= MARK_5X5_MIN_MS && duration < MARK_5X5_MAX_MS) {
            g_last_mark = MARK_5X5;
            return MARK_5X5;
        }
        
    }
    
    return MARK_NONE;
}

/*************************************************************
 * 函数名称：tracking_handle_mark
 * 功能介绍：处理检测到的赛道标记
 * 参数：mark - 标记类型
 * 算法说明：根据标记类型执行相应动作
 *************************************************************/
void tracking_handle_mark(TrackMark_t mark)
{
    switch (mark) {
        case MARK_5X5:
            // 5x5cm标记：精确位置校准点
            // 可在此记录当前位置，用于里程计校准
            // beep_on_times(1, 50);  // 短鸣提示
            break;
            
        case MARK_15X15:
            // 15x15cm标记：任务点或校准点
            // 可根据位置执行不同动作
            // 例如：左上标记=开始计时，中心标记=准备抓取
            beep_on_times(1, 100);
            break;
            
        case MARK_40X40:
            // 40x40cm区域：作业区域
            // 减速进入，执行抓取/放置任务
            car_set(5, 5);  // 极慢速进入
            // 可触发机械臂动作
            // parse_cmd((u8 *)"$DGT:1-5,1!");  // 执行预设动作组
            break;
            
        case MARK_CROSS:
            // 十字路口：默认直行通过
            // 如需转向，可在此根据任务规划选择方向
            // car_set(8, 8);  // 中速通过
            break;
            
        default:
            break;
    }
}

/*************************************************************
 * 函数名称：tracking_recover_lost
 * 功能介绍：脱线恢复算法
 * 参数：无
 * 算法说明：根据最后已知方向进行弧线搜索
 *************************************************************/
void tracking_recover_lost(void)
{
    static uint8_t recovery_step = 0;
    static uint32_t recovery_timer = 0;
    
    switch (recovery_step) {
        case 0:  // 制动
            car_set(0, 0);
            recovery_timer = millis();
            recovery_step = 1;
            break;
            
        case 1:  // 等待稳定
            if (millis() - recovery_timer > 50) {
                recovery_step = 2;
                recovery_timer = millis();
            }
            break;
            
        case 2:  // 弧线搜索
            // 最后偏左了，线应该在右边，向右弧线搜索
            if (g_last_valid_dir < 0) {
                car_set(12, -6);   // 右转弧线
            } else if (g_last_valid_dir > 0) {
                car_set(-6, 12);   // 左转弧线
            } else {
                car_set(10, -10);  // 未知方向，原地右旋
            }
            recovery_step = 3;
            break;
            
        case 3:  // 搜索中
            {
                uint8_t s1 = x1(), s2 = x2(), s3 = x3(), s4 = x4();
                
                // 找到线了
                if (!(s1 && s2 && s3 && s4)) {
                    car_set(0, 0);
                    recovery_step = 0;
                    g_lost_flag = 0;
                    g_lost_counter = 0;
                    g_pid.integral = 0;  // 重置PID积分
                    beep_on_times(2, 50);  // 恢复提示
                }
                // 超时
                else if (millis() - recovery_timer > LOST_RECOVERY_TIME) {
                    car_set(0, 0);
                    recovery_step = 0;
                    g_lost_flag = 1;  // 保持脱线状态
                    beep_on_times(5, 200);  // 报警
                }
            }
            break;
    }
}

// ========== 修改：AI_xunji_moshi（增强版循迹函数）==========

/*************************************************************
 * 函数名称：AI_xunji_moshi
 * 功能介绍：增强型循迹主函数（加权位置估算+自适应PID+标记识别）
 * 参数：无
 * 返回值：无
 * 
 * 算法流程：
 * 1. 读取四路传感器
 * 2. 检测是否全黑（标记点）或全白（脱线）
 * 3. 查表获取位置估计值
 * 4. 自适应PID计算转向量
 * 5. 映射到左右轮速度
 * 6. 速度平滑处理
 * 7. 输出到电机
 *************************************************************/
void AI_xunji_moshi(void)
{
    // 1. 读取传感器值（0=黑线，1=白色）
    uint8_t s1 = x1();  // 左外 PA1
    uint8_t s2 = x2();  // 左内 PA0
    uint8_t s3 = x3();  // 右内 PA3
    uint8_t s4 = x4();  // 右外 PB1
    
    // 调试输出（调试用，可取消注释）
    // sprintf((char*)cmd_return, "S:%d%d%d%d\r\n", s1, s2, s3, s4);
    // uart1_send_str(cmd_return);
    
    // 2. 检查全黑（标记检测）
    uint8_t is_all_black = (s1==0 && s2==0 && s3==0 && s4==0);
    
    if (is_all_black) {
        TrackMark_t mark = tracking_detect_mark(s1, s2, s3, s4);
        if (mark != MARK_NONE) {
            tracking_handle_mark(mark);
            // 全黑时继续直行通过（或根据标记类型调整）
            if (mark == MARK_CROSS || mark == MARK_5X5) {
                // 十字路口或小标记，保持当前速度直行
                return;
            }
        }
    }
    
    // 3. 检查全白（脱线检测）
    uint8_t is_all_white = (s1==1 && s2==1 && s3==1 && s4==1);
    
    if (is_all_white) {
        g_lost_counter++;
        
        if (g_lost_counter > LOST_THRESHOLD) {  // 连续多次全白，确认脱线
            if (!g_lost_flag) {
                g_lost_flag = 1;
                // 记录最后已知方向
                if (s1 == 0) g_last_valid_dir = -1;
                else if (s4 == 0) g_last_valid_dir = 1;
            }
            tracking_recover_lost();
            return;
        }
    } else {
        // 正常状态，重置脱线计数
        g_lost_counter = 0;
        g_lost_flag = 0;
        
        // 记录方向（用于脱线恢复）
        if (s1 == 0) g_last_valid_dir = -1;  // 偏左
        else if (s4 == 0) g_last_valid_dir = 1;   // 偏右
        else g_last_valid_dir = 0;
    }
    
    // 4. 获取位置估计值
    int8_t position = tracking_get_position(s1, s2, s3, s4);
    
    // 处理特殊值
    if (position >= POS_ALL_WHITE) {
        position = 0;  // 默认居中
    }
    
    // 5. PID计算
    int16_t base_speed = tracking_pid_calc(&g_pid, 0, position);
    
    // 6. 计算目标速度
    int16_t target_left, target_right;
    tracking_calc_speed(g_pid.output, base_speed, &target_left, &target_right);
    
    // 7. 速度平滑
    g_current_left_speed = tracking_smooth_speed(target_left, g_current_left_speed);
    g_current_right_speed = tracking_smooth_speed(target_right, g_current_right_speed);
    
    // 8. 输出到电机
    car_set(g_current_left_speed, g_current_right_speed);
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
    uart1_send_str((u8 *)"\r\n=== 位置查表测试 ===\r\n");
    
    // 测试用例：{s1,s2,s3,s4, 期望位置, 描述}
    struct {
        uint8_t s1, s2, s3, s4;
        int8_t expected;
        char *desc;
    } test_cases[] = {
        {1,0,0,1, 0, "居中"},
        {1,0,1,1, 5, "微偏左"},
        {1,1,0,1, -5, "微偏右"},
        {0,0,1,1, 20, "明显偏左"},
        {1,1,0,0, -20, "明显偏右"},
        {0,1,1,1, 15, "偏左"},
        {1,1,1,0, -15, "偏右"},
        {1,1,1,1, 99, "全白脱线"},
        {0,0,0,0, 100, "全黑标记"},
    };
    
    uint8_t pass = 0, fail = 0;
    uint8_t i;
    for (i = 0; i < 9; i++) {
        int8_t result = tracking_get_position(
            test_cases[i].s1,
            test_cases[i].s2,
            test_cases[i].s3,
            test_cases[i].s4
        );
        
        uint8_t ok = (result == test_cases[i].expected);
        if (ok) pass++; else fail++;
        
        sprintf((char*)cmd_return, "%s: %d%d%d%d -> %d (期望%d) [%s]\r\n",
            test_cases[i].desc,
            test_cases[i].s1, test_cases[i].s2,
            test_cases[i].s3, test_cases[i].s4,
            result, test_cases[i].expected,
            ok ? "PASS" : "FAIL");
        uart1_send_str(cmd_return);
    }
    
    sprintf((char*)cmd_return, "结果: PASS=%d, FAIL=%d\r\n", pass, fail);
    uart1_send_str(cmd_return);
}

/*************************************************************
 * 函数名称：test_pid_controller
 * 功能介绍：测试PID控制器的响应
 *************************************************************/
void test_pid_controller(void)
{
    uart1_send_str((u8 *)"\r\n=== PID控制器测试 ===\r\n");
    
    PID_TypeDef test_pid = {30, 2, 45, 0, 0, 0, 0};
    
    // 模拟从偏左到居中的过程
    int16_t positions[] = {-20, -15, -10, -5, 0, 0, 0, 5, 10, 5, 0};
    uint8_t num = sizeof(positions) / sizeof(positions[0]);
    uint8_t i;
    
    uart1_send_str((u8 *)"Pos\tErr\tP\tI\tD\tOut\tSpeed\r\n");
    
    for (i = 0; i < num; i++) {
        int16_t base = tracking_pid_calc(&test_pid, 0, positions[i]);
        
        sprintf((char*)cmd_return, "%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
            positions[i],
            test_pid.err,
            (test_pid.Kp * test_pid.err) / 10,
            (PID_KI * test_pid.integral) / 100,
            (test_pid.Kd * (test_pid.err - test_pid.err_last)) / 10,
            test_pid.output,
            base);
        uart1_send_str(cmd_return);
        
        // 模拟10ms延迟
        tb_delay_ms(10);
    }
}

/*************************************************************
 * 函数名称：test_speed_mapping
 * 功能介绍：测试速度映射的正确性
 *************************************************************/
void test_speed_mapping(void)
{
    uart1_send_str((u8 *)"\r\n=== 速度映射测试 ===\r\n");
    
    uart1_send_str((u8 *)"PID输出\t左轮\t右轮\t动作\r\n");
    
    int16_t pid_out;
    for (pid_out = -80; pid_out <= 80; pid_out += 20) {
        int16_t left, right;
        tracking_calc_speed(pid_out, 12, &left, &right);
        
        char *action;
        if (pid_out > 5) action = "左转";
        else if (pid_out < -5) action = "右转";
        else action = "直行";
        
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
    uart1_send_str((u8 *)"\r\n=== 速度平滑测试 ===\r\n");
    
    int16_t current = 0;
    int16_t targets[] = {15, 15, 15, 5, 5, -10, -10, 0};
    uint8_t num = sizeof(targets) / sizeof(targets[0]);
    uint8_t i;
    
    uart1_send_str((u8 *)"目标\t当前\t平滑后\r\n");
    
    for (i = 0; i < num; i++) {
        int16_t smoothed = tracking_smooth_speed(targets[i], current);
        
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
    uart1_send_str((u8 *)"\r\n========== 循迹算法单元测试 ==========\r\n");
    
    test_tracking_position_table();
    tb_delay_ms(100);
    
    test_pid_controller();
    tb_delay_ms(100);
    
    test_speed_mapping();
    tb_delay_ms(100);
    
    test_speed_smoothing();
    tb_delay_ms(100);
    
    uart1_send_str((u8 *)"\r\n========== 测试完成 ==========\r\n");
}

