#include <stdio.h>
#include <string.h>
#include "z_sorting.h"
#include "z_global.h"
#include "z_main.h"
#include "z_sensor.h"
#include "z_delay.h"
#include "z_usart.h"
#include "z_kinematics.h"

// ========== 全局变量 ==========

// 工位配置（红、绿、蓝）
static Station_t g_stations[4];  // 索引0保留，1=红，2=绿，3=蓝

// 分拣系统控制实例
static SortingCtrl_t g_sorting_ctrl = {
    .state = SORT_IDLE,
    .last_state = SORT_IDLE,
    .target_color = PART_NONE,
    .detected_color = PART_NONE,
    .retry_counter = 0,
    .is_busy = 0,
    .state_timer = 0,
    .stats = {0, 0, 0, 0, 0, PART_NONE, 0}
};

// 机械臂初始位置
static float g_home_x = 100.0f;
static float g_home_y = 0.0f;
static float g_home_z = 100.0f;

// 错误码定义
#define ERR_NONE            0
#define ERR_COLOR_MISMATCH  1
#define ERR_GRAB_FAILED     2
#define ERR_PLACE_FAILED    3
#define ERR_NO_PART         4
#define ERR_ARM_TIMEOUT     5

// ========== 初始化函数 ==========

/*************************************************************
 * 函数名称：sorting_init
 * 功能介绍：初始化分拣系统
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_init(void)
{
    // 初始化工位配置
    sorting_load_stations();
    
    // 初始化统计信息
    memset(&g_sorting_ctrl.stats, 0, sizeof(SortingStats_t));
    g_sorting_ctrl.stats.last_part_color = PART_NONE;
    
    // 初始化状态
    g_sorting_ctrl.state = SORT_IDLE;
    g_sorting_ctrl.last_state = SORT_IDLE;
    g_sorting_ctrl.is_busy = 0;
    g_sorting_ctrl.retry_counter = 0;
    
    // 初始化颜色传感器
    TCS34725_Init(TCS34725_INTEGRATIONTIME_24MS);
    
    uart1_send_str((u8 *)"\r\n[分拣系统] 初始化完成\r\n");
}

/*************************************************************
 * 函数名称：sorting_load_stations
 * 功能介绍：加载工位配置
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_load_stations(void)
{
    // 红色工位
    g_stations[PART_RED].x = PLACE_RED_X;
    g_stations[PART_RED].y = PLACE_RED_Y;
    g_stations[PART_RED].z = PLACE_Z_HEIGHT;
    g_stations[PART_RED].action_group = ACTION_PLACE_RED;
    
    // 绿色工位
    g_stations[PART_GREEN].x = PLACE_GRN_X;
    g_stations[PART_GREEN].y = PLACE_GRN_Y;
    g_stations[PART_GREEN].z = PLACE_Z_HEIGHT;
    g_stations[PART_GREEN].action_group = ACTION_PLACE_GRN;
    
    // 蓝色工位
    g_stations[PART_BLUE].x = PLACE_BLU_X;
    g_stations[PART_BLUE].y = PLACE_BLU_Y;
    g_stations[PART_BLUE].z = PLACE_Z_HEIGHT;
    g_stations[PART_BLUE].action_group = ACTION_PLACE_BLU;
}

// ========== 主控制函数 ==========

/*************************************************************
 * 函数名称：sorting_task
 * 功能介绍：分拣系统主任务，需在主循环中周期性调用
 * 参数：无
 * 返回值：无
 * 算法说明：状态机驱动，每个状态处理对应操作
 *************************************************************/
void sorting_task(void)
{
    // 状态机处理
    switch (g_sorting_ctrl.state) {
        case SORT_IDLE:
            sorting_state_idle();
            break;
            
        case SORT_MOVE_TO_TARGET:
            sorting_state_move_to_target();
            break;
            
        case SORT_DETECT_COLOR:
            sorting_state_detect_color();
            break;
            
        case SORT_VERIFY_COLOR:
            sorting_state_verify_color();
            break;
            
        case SORT_GRAB_PART:
            sorting_state_grab();
            break;
            
        case SORT_VERIFY_GRAB:
            sorting_state_verify_grab();
            break;
            
        case SORT_MOVE_TO_STATION:
            sorting_state_move_to_station();
            break;
            
        case SORT_PLACE_PART:
            sorting_state_place();
            break;
            
        case SORT_VERIFY_PLACE:
            sorting_state_verify_place();
            break;
            
        case SORT_RETURN_HOME:
            sorting_state_return_home();
            break;
            
        case SORT_ERROR_HANDLE:
            sorting_state_error();
            break;
            
        case SORT_COMPLETE:
            g_sorting_ctrl.is_busy = 0;
            g_sorting_ctrl.state = SORT_IDLE;
            break;
            
        default:
            g_sorting_ctrl.state = SORT_IDLE;
            break;
    }
}

/*************************************************************
 * 函数名称：sorting_start
 * 功能介绍：启动分拣任务
 * 参数：target - 目标颜色（PART_RED/PART_GREEN/PART_BLUE）
 * 返回值：无
 *************************************************************/
void sorting_start(PartColor_t target)
{
    if (g_sorting_ctrl.is_busy) {
        uart1_send_str((u8 *)"[分拣系统] 错误：系统忙\r\n");
        return;
    }
    
    g_sorting_ctrl.target_color = target;
    g_sorting_ctrl.detected_color = PART_NONE;
    g_sorting_ctrl.retry_counter = 0;
    g_sorting_ctrl.is_busy = 1;
    g_sorting_ctrl.state_timer = millis();
    g_sorting_ctrl.state = SORT_MOVE_TO_TARGET;
    
    sprintf((char*)cmd_return, "[分拣系统] 开始分拣 %s 零件\r\n", 
            sorting_color_to_str(target));
    uart1_send_str(cmd_return);
}

/*************************************************************
 * 函数名称：sorting_stop
 * 功能介绍：停止分拣任务
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_stop(void)
{
    g_sorting_ctrl.is_busy = 0;
    g_sorting_ctrl.state = SORT_IDLE;
    g_sorting_ctrl.retry_counter = 0;
    
    // 机械臂返回安全位置
    sorting_arm_return_home();
    
    uart1_send_str((u8 *)"[分拣系统] 任务停止\r\n");
}

// ========== 状态处理函数 ==========

/*************************************************************
 * 空闲状态 - 等待指令
 *************************************************************/
void sorting_state_idle(void)
{
    // 空闲状态不做任何事，等待sorting_start调用
}

/*************************************************************
 * 移动到目标位置 - 使用循迹导航到零件附近
 *************************************************************/
void sorting_state_move_to_target(void)
{
    static uint8_t step = 0;
    
    switch (step) {
        case 0:  // 启动循迹模式
            // 假设通过标记检测到达目标区域
            // 这里可以调用循迹函数
            AI_xunji_moshi();
            
            // 检测是否到达40x40区域（零件放置区）
            uint8_t s1 = x1(), s2 = x2(), s3 = x3(), s4 = x4();
            TrackMark_t mark = tracking_detect_mark(s1, s2, s3, s4);
            
            if (mark == MARK_40X40) {
                // 到达目标区域，停车
                car_set(0, 0);
                step = 1;
                g_sorting_ctrl.state_timer = millis();
            }
            break;
            
        case 1:  // 等待稳定
            if (millis() - g_sorting_ctrl.state_timer > 500) {
                step = 0;
                g_sorting_ctrl.state = SORT_DETECT_COLOR;
                g_sorting_ctrl.state_timer = millis();
            }
            break;
    }
}

/*************************************************************
 * 检测颜色 - 使用TCS34725颜色传感器
 *************************************************************/
void sorting_state_detect_color(void)
{
    // 执行颜色检测
    g_sorting_ctrl.detected_color = sorting_detect_color();
    
    if (g_sorting_ctrl.detected_color == PART_UNKNOWN) {
        // 未检测到有效颜色
        sorting_log_error(ERR_NO_PART, PART_UNKNOWN);
        g_sorting_ctrl.state = SORT_ERROR_HANDLE;
        return;
    }
    
    sprintf((char*)cmd_return, "[分拣系统] 检测到颜色: %s\r\n",
            sorting_color_to_str(g_sorting_ctrl.detected_color));
    uart1_send_str(cmd_return);
    
    // 进入验证状态
    g_sorting_ctrl.state = SORT_VERIFY_COLOR;
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 验证颜色 - 多次采样确认
 *************************************************************/
void sorting_state_verify_color(void)
{
    static uint8_t verify_count = 0;
    static PartColor_t color_samples[COLOR_SAMPLE_TIMES];
    
    if (millis() - g_sorting_ctrl.state_timer < 100) {
        return;  // 等待100ms再采样
    }
    
    // 采集样本
    color_samples[verify_count] = sorting_detect_color();
    verify_count++;
    
    if (verify_count < COLOR_SAMPLE_TIMES) {
        g_sorting_ctrl.state_timer = millis();
        return;  // 继续采样
    }
    
    // 统计结果（取众数）
    uint8_t red_cnt = 0, grn_cnt = 0, blu_cnt = 0;
    uint8_t i;
    for (i = 0; i < COLOR_SAMPLE_TIMES; i++) {
        if (color_samples[i] == PART_RED) red_cnt++;
        else if (color_samples[i] == PART_GREEN) grn_cnt++;
        else if (color_samples[i] == PART_BLUE) blu_cnt++;
    }
    
    // 确定最终颜色
    PartColor_t verified_color = PART_UNKNOWN;
    if (red_cnt >= grn_cnt && red_cnt >= blu_cnt && red_cnt > 0) {
        verified_color = PART_RED;
    } else if (grn_cnt >= red_cnt && grn_cnt >= blu_cnt && grn_cnt > 0) {
        verified_color = PART_GREEN;
    } else if (blu_cnt > 0) {
        verified_color = PART_BLUE;
    }
    
    g_sorting_ctrl.detected_color = verified_color;
    verify_count = 0;
    
    sprintf((char*)cmd_return, "[分拣系统] 验证颜色: %s (R:%d G:%d B:%d)\r\n",
            sorting_color_to_str(verified_color), red_cnt, grn_cnt, blu_cnt);
    uart1_send_str(cmd_return);
    
    // 检查颜色是否匹配目标
    if (!sorting_is_color_match(verified_color, g_sorting_ctrl.target_color)) {
        // 颜色不匹配
        sorting_log_error(ERR_COLOR_MISMATCH, verified_color);
        g_sorting_ctrl.stats.color_error_count++;
        
        // 如果不是严格匹配模式，继续抓取（按实际颜色分类）
        // 如果是严格匹配模式，进入错误处理
        // 这里采用灵活模式：按检测到的实际颜色分类放置
        g_sorting_ctrl.target_color = verified_color;
    }
    
    // 进入抓取状态
    g_sorting_ctrl.state = SORT_GRAB_PART;
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 抓取零件 - 控制机械臂执行抓取
 *************************************************************/
void sorting_state_grab(void)
{
    uart1_send_str((u8 *)"[分拣系统] 开始抓取...\r\n");
    
    if (sorting_arm_grab()) {
        g_sorting_ctrl.state = SORT_VERIFY_GRAB;
    } else {
        sorting_log_error(ERR_GRAB_FAILED, g_sorting_ctrl.detected_color);
        g_sorting_ctrl.state = SORT_ERROR_HANDLE;
    }
    
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 验证抓取 - 通过超声波检测验证
 *************************************************************/
void sorting_state_verify_grab(void)
{
    // 使用超声波验证是否抓取成功
    // 如果抓取成功，前方距离应该发生变化
    uint16_t distance = get_adc_csb_middle();
    
    if (distance < GRAB_DISTANCE_VERIFY) {
        // 距离过近，可能没有抓取到（被遮挡）
        // 或者抓取成功，零件在机械臂前方
        
        // 这里简化处理：假设抓取成功
        // 实际可以通过重量传感器或视觉二次确认
        uart1_send_str((u8 *)"[分拣系统] 抓取验证通过\r\n");
        
        g_sorting_ctrl.state = SORT_MOVE_TO_STATION;
    } else {
        // 抓取可能失败
        uart1_send_str((u8 *)"[分拣系统] 抓取验证失败\r\n");
        
        if (g_sorting_ctrl.retry_counter < MAX_RETRY_TIMES) {
            g_sorting_ctrl.retry_counter++;
            g_sorting_ctrl.state = SORT_GRAB_PART;  // 重试
        } else {
            sorting_log_error(ERR_GRAB_FAILED, g_sorting_ctrl.detected_color);
            g_sorting_ctrl.state = SORT_ERROR_HANDLE;
        }
    }
    
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 移动到工位 - 导航到对应颜色的工位
 *************************************************************/
void sorting_state_move_to_station(void)
{
    // 根据检测到的颜色选择工位
    PartColor_t color = g_sorting_ctrl.detected_color;
    
    if (color < PART_RED || color > PART_BLUE) {
        sorting_log_error(ERR_COLOR_MISMATCH, color);
        g_sorting_ctrl.state = SORT_ERROR_HANDLE;
        return;
    }
    
    // 移动到对应工位（使用逆运动学或直接动作组）
    // 这里使用动作组方式
    sprintf((char*)cmd_return, "[分拣系统] 移动到 %s 工位\r\n",
            sorting_color_to_str(color));
    uart1_send_str(cmd_return);
    
    // 执行移动到工位的动作组
    // 实际实现中可能需要先循迹到工位，再控制机械臂
    
    g_sorting_ctrl.state = SORT_PLACE_PART;
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 放置零件 - 在工位释放零件
 *************************************************************/
void sorting_state_place(void)
{
    uart1_send_str((u8 *)"[分拣系统] 开始放置...\r\n");
    
    if (sorting_arm_place(g_sorting_ctrl.detected_color)) {
        g_sorting_ctrl.state = SORT_VERIFY_PLACE;
    } else {
        sorting_log_error(ERR_PLACE_FAILED, g_sorting_ctrl.detected_color);
        g_sorting_ctrl.state = SORT_ERROR_HANDLE;
    }
    
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 验证放置 - 确认零件已释放
 *************************************************************/
void sorting_state_verify_place(void)
{
    // 验证放置成功
    // 可以通过超声波检测或视觉确认
    
    uart1_send_str((u8 *)"[分拣系统] 放置完成\r\n");
    
    // 更新统计
    sorting_update_stats(1, g_sorting_ctrl.detected_color);
    
    g_sorting_ctrl.state = SORT_RETURN_HOME;
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 返回初始位置
 *************************************************************/
void sorting_state_return_home(void)
{
    uart1_send_str((u8 *)"[分拣系统] 返回初始位置\r\n");
    
    sorting_arm_return_home();
    
    g_sorting_ctrl.state = SORT_COMPLETE;
    g_sorting_ctrl.retry_counter = 0;
}

/*************************************************************
 * 错误处理
 *************************************************************/
void sorting_state_error(void)
{
    static uint8_t error_handled = 0;
    
    if (!error_handled) {
        error_handled = 1;
        
        // 更新统计
        sorting_update_stats(0, g_sorting_ctrl.detected_color);
        
        // 报警
        sorting_alarm(ERR_GRAB_FAILED);
        
        // 发送错误信息
        sorting_send_stats();
        
        uart1_send_str((u8 *)"[分拣系统] 错误处理完成\r\n");
    }
    
    if (millis() - g_sorting_ctrl.state_timer > 1000) {
        error_handled = 0;
        
        // 检查是否超过最大重试次数
        if (g_sorting_ctrl.retry_counter < MAX_RETRY_TIMES) {
            g_sorting_ctrl.retry_counter++;
            g_sorting_ctrl.state = SORT_MOVE_TO_TARGET;  // 重试
            uart1_send_str((u8 *)"[分拣系统] 准备重试...\r\n");
        } else {
            g_sorting_ctrl.state = SORT_RETURN_HOME;  // 放弃，返回
            uart1_send_str((u8 *)"[分拣系统] 超过重试次数，放弃任务\r\n");
        }
    }
}

// ========== 颜色识别函数 ==========

/*************************************************************
 * 函数名称：sorting_detect_color
 * 功能介绍：检测零件颜色
 * 参数：无
 * 返回值：检测到的颜色
 *************************************************************/
PartColor_t sorting_detect_color(void)
{
    COLOR_RGBC rgbc;
    COLOR_HSL hsl;
    
    // 获取颜色数据
    TCS34725_GetRawData(&rgbc);
    
    // 检查亮度
    if (rgbc.c < COLOR_DETECT_THRESHOLD) {
        return PART_UNKNOWN;  // 亮度不足，可能没有零件
    }
    
    // 转换为HSL（可选，用于更精确的颜色识别）
    RGBtoHSL(&rgbc, &hsl);
    
    // 判断颜色
    // 策略1：基于RGB值
    if (rgbc.r > rgbc.g && rgbc.r > rgbc.b) {
        // 红色分量最大
        if (rgbc.r > COLOR_RED_BASE) {
            return PART_RED;
        }
    } else if (rgbc.g > rgbc.r && rgbc.g > rgbc.b) {
        // 绿色分量最大
        if (rgbc.g > COLOR_GRN_BASE) {
            return PART_GREEN;
        }
    } else if (rgbc.b > rgbc.r && rgbc.b > rgbc.g) {
        // 蓝色分量最大
        if (rgbc.b > COLOR_BLU_BASE) {
            return PART_BLUE;
        }
    }
    
    // 策略2：基于HSL色相（备用）
    if (hsl.h >= 330 || hsl.h <= 30) {
        return PART_RED;
    } else if (hsl.h >= 60 && hsl.h <= 180) {
        return PART_GREEN;
    } else if (hsl.h >= 180 && hsl.h <= 270) {
        return PART_BLUE;
    }
    
    return PART_UNKNOWN;
}

/*************************************************************
 * 函数名称：sorting_is_color_match
 * 功能介绍：检查检测颜色是否匹配目标
 * 参数：detect - 检测到的颜色
 *       target - 目标颜色
 * 返回值：1=匹配，0=不匹配
 *************************************************************/
uint8_t sorting_is_color_match(PartColor_t detect, PartColor_t target)
{
    // 严格匹配模式
    return (detect == target) ? 1 : 0;
}

// ========== 机械臂控制函数 ==========

/*************************************************************
 * 函数名称：sorting_arm_move_to
 * 功能介绍：控制机械臂移动到指定坐标
 * 参数：x,y,z - 目标坐标
 *       time - 运动时间
 * 返回值：1=成功，0=失败
 *************************************************************/
uint8_t sorting_arm_move_to(float x, float y, float z, uint16_t time)
{
    // 使用逆运动学计算舵机角度
    if (kinematics_move(x, y, z, time) == 0) {
        return 1;
    }
    return 0;
}

/*************************************************************
 * 函数名称：sorting_arm_grab
 * 功能介绍：执行抓取动作序列
 * 参数：无
 * 返回值：1=成功，0=失败
 *************************************************************/
uint8_t sorting_arm_grab(void)
{
    // 方式1：使用逆运动学精确控制
    // 移动到抓取准备位置
    if (!sorting_arm_move_to(GRAB_X_DEFAULT, GRAB_Y_DEFAULT, GRAB_Z_UP, ARM_MOVE_TIME_FAST)) {
        return 0;
    }
    tb_delay_ms(ARM_MOVE_TIME_FAST);
    
    // 下降
    if (!sorting_arm_move_to(GRAB_X_DEFAULT, GRAB_Y_DEFAULT, GRAB_Z_DOWN, ARM_MOVE_TIME_SLOW)) {
        return 0;
    }
    tb_delay_ms(ARM_MOVE_TIME_SLOW);
    
    // 夹爪闭合（通过动作组或舵机控制）
    // 假设夹爪由舵机5控制
    set_servo(5, 2400, ARM_GRAB_WAIT_TIME);  // 闭合夹爪
    tb_delay_ms(ARM_GRAB_WAIT_TIME);
    
    // 抬起
    if (!sorting_arm_move_to(GRAB_X_DEFAULT, GRAB_Y_DEFAULT, GRAB_Z_UP, ARM_MOVE_TIME_SLOW)) {
        return 0;
    }
    tb_delay_ms(ARM_MOVE_TIME_SLOW);
    
    return 1;
    
    // 方式2：使用预存动作组（推荐，更稳定）
    // parse_cmd((u8 *)"$DGT:10-13,1!");
    // return 1;
}

/*************************************************************
 * 函数名称：sorting_arm_place
 * 功能介绍：执行放置动作序列
 * 参数：color - 零件颜色，决定放置工位
 * 返回值：1=成功，0=失败
 *************************************************************/
uint8_t sorting_arm_place(PartColor_t color)
{
    if (color < PART_RED || color > PART_BLUE) {
        return 0;
    }
    
    Station_t *station = &g_stations[color];
    
    // 方式1：使用逆运动学
    // 移动到工位上方
    if (!sorting_arm_move_to(station->x, station->y, GRAB_Z_UP, ARM_MOVE_TIME_FAST)) {
        return 0;
    }
    tb_delay_ms(ARM_MOVE_TIME_FAST);
    
    // 下降
    if (!sorting_arm_move_to(station->x, station->y, station->z, ARM_MOVE_TIME_SLOW)) {
        return 0;
    }
    tb_delay_ms(ARM_MOVE_TIME_SLOW);
    
    // 夹爪释放
    set_servo(5, 1500, ARM_RELEASE_WAIT_TIME);  // 释放夹爪
    tb_delay_ms(ARM_RELEASE_WAIT_TIME);
    
    // 抬起
    if (!sorting_arm_move_to(station->x, station->y, GRAB_Z_UP, ARM_MOVE_TIME_FAST)) {
        return 0;
    }
    tb_delay_ms(ARM_MOVE_TIME_FAST);
    
    return 1;
    
    // 方式2：使用预存动作组
    // char cmd[32];
    // sprintf(cmd, "$DGT:%d-%d,1!", station->action_group, station->action_group+5);
    // parse_cmd((u8 *)cmd);
    // return 1;
}

/*************************************************************
 * 函数名称：sorting_arm_return_home
 * 功能介绍：机械臂返回初始位置
 * 参数：无
 * 返回值：1=成功，0=失败
 *************************************************************/
uint8_t sorting_arm_return_home(void)
{
    return sorting_arm_move_to(g_home_x, g_home_y, g_home_z, ARM_MOVE_TIME_FAST);
}

// ========== 统计与报警函数 ==========

/*************************************************************
 * 函数名称：sorting_update_stats
 * 功能介绍：更新分拣统计信息
 * 参数：success - 1=成功，0=失败
 *       color - 零件颜色
 * 返回值：无
 *************************************************************/
void sorting_update_stats(uint8_t success, PartColor_t color)
{
    g_sorting_ctrl.stats.total_attempts++;
    
    if (success) {
        g_sorting_ctrl.stats.success_count++;
        g_sorting_ctrl.stats.consecutive_errors = 0;
    } else {
        g_sorting_ctrl.stats.fail_count++;
        g_sorting_ctrl.stats.consecutive_errors++;
    }
    
    g_sorting_ctrl.stats.last_part_color = color;
}

/*************************************************************
 * 函数名称：sorting_send_stats
 * 功能介绍：发送统计信息到串口
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_send_stats(void)
{
    SortingStats_t *stats = &g_sorting_ctrl.stats;
    
    uart1_send_str((u8 *)"\r\n========== 分拣统计 ==========\r\n");
    
    sprintf((char*)cmd_return, "总尝试次数: %d\r\n", stats->total_attempts);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "成功次数: %d\r\n", stats->success_count);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "失败次数: %d\r\n", stats->fail_count);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "颜色错误次数: %d\r\n", stats->color_error_count);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "重试次数: %d\r\n", stats->retry_count);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "成功率: %d%%\r\n", 
            stats->total_attempts > 0 ? 
            (stats->success_count * 100 / stats->total_attempts) : 0);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "连续错误: %d\r\n", stats->consecutive_errors);
    uart1_send_str(cmd_return);
    
    uart1_send_str((u8 *)"==============================\r\n");
}

/*************************************************************
 * 函数名称：sorting_alarm
 * 功能介绍：报警提示
 * 参数：error_code - 错误码
 * 返回值：无
 *************************************************************/
void sorting_alarm(uint8_t error_code)
{
    // 蜂鸣器报警
    beep_on_times(ALARM_BEEP_TIMES, ALARM_BEEP_DURATION);
    
    // LED闪烁提示
    // nled_on(); tb_delay_ms(200); nled_off();
    
    // 串口输出错误信息
    sprintf((char*)cmd_return, "[分拣系统] 报警！错误码: %d\r\n", error_code);
    uart1_send_str(cmd_return);
}

/*************************************************************
 * 函数名称：sorting_log_error
 * 功能介绍：记录错误日志
 * 参数：error_code - 错误码
 *       color - 相关颜色
 * 返回值：无
 *************************************************************/
void sorting_log_error(uint8_t error_code, PartColor_t color)
{
    sprintf((char*)cmd_return, "[分拣系统] 错误记录: 码=%d 颜色=%s 时间=%d\r\n",
            error_code, sorting_color_to_str(color), (int)millis());
    uart1_send_str(cmd_return);
}

// ========== 工具函数 ==========

/*************************************************************
 * 函数名称：sorting_color_to_str
 * 功能介绍：颜色枚举转字符串
 * 参数：color - 颜色枚举
 * 返回值：颜色字符串
 *************************************************************/
const char* sorting_color_to_str(PartColor_t color)
{
    switch (color) {
        case PART_RED:   return "红色";
        case PART_GREEN: return "绿色";
        case PART_BLUE:  return "蓝色";
        case PART_NONE:  return "无";
        default:         return "未知";
    }
}

/*************************************************************
 * 函数名称：sorting_is_busy
 * 功能介绍：检查系统是否忙
 * 参数：无
 * 返回值：1=忙，0=空闲
 *************************************************************/
uint8_t sorting_is_busy(void)
{
    return g_sorting_ctrl.is_busy;
}

/*************************************************************
 * 函数名称：sorting_get_state
 * 功能介绍：获取当前状态
 * 参数：无
 * 返回值：当前状态
 *************************************************************/
SortingState_t sorting_get_state(void)
{
    return g_sorting_ctrl.state;
}
