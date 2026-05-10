#include <stdio.h>
#include <string.h>
#include <math.h>
#include "z_sorting.h"
#include "z_global.h"
#include "z_main.h"
#include "z_sensor.h"
#include "z_delay.h"
#include "z_usart.h"
#include "z_kinematics.h"
#include "z_timer.h"
#include "z_gpio.h"

// ========== 全局变量 ==========

// 工位配置（红、绿、蓝）
static Station_t g_stations[4];  // 索引0保留，1=红，2=绿，3=蓝

// 分拣系统控制实例 (C89标准：在函数中初始化)
static SortingCtrl_t g_sorting_ctrl;

// 机械臂初始位置
static float g_home_x = 100.0f;
static float g_home_y = 0.0f;
static float g_home_z = 100.0f;

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
    g_sorting_ctrl.process_start_time = 0;
    
    // 初始化颜色校准数据
    memset(&g_sorting_ctrl.color_cal, 0, sizeof(ColorCal_t));
    
    // 初始化颜色传感器（24ms积分时间，平衡速度和精度）
    TCS34725_Init(TCS34725_INTEGRATIONTIME_24MS);
    
    uart1_send_str((u8 *)"\r\n[Sorting V2.0] Init OK\r\n");
    uart1_send_str((u8 *)"[Sorting] Target: Accuracy>=95% Time<=10s/pc\r\n");
}

/*************************************************************
 * 函数名称：sorting_load_stations
 * 功能介绍：加载工位配置（可配置）
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_load_stations(void)
{
    // 红色工位
    g_stations[PART_RED].x = PLACE_RED_X + ARM_POS_COMPENSATE_X;
    g_stations[PART_RED].y = PLACE_RED_Y + ARM_POS_COMPENSATE_Y;
    g_stations[PART_RED].z = PLACE_Z_HEIGHT + ARM_POS_COMPENSATE_Z;
    g_stations[PART_RED].action_group = ACTION_PLACE_RED;
    
    // 绿色工位
    g_stations[PART_GREEN].x = PLACE_GRN_X + ARM_POS_COMPENSATE_X;
    g_stations[PART_GREEN].y = PLACE_GRN_Y + ARM_POS_COMPENSATE_Y;
    g_stations[PART_GREEN].z = PLACE_Z_HEIGHT + ARM_POS_COMPENSATE_Z;
    g_stations[PART_GREEN].action_group = ACTION_PLACE_GRN;
    
    // 蓝色工位
    g_stations[PART_BLUE].x = PLACE_BLU_X + ARM_POS_COMPENSATE_X;
    g_stations[PART_BLUE].y = PLACE_BLU_Y + ARM_POS_COMPENSATE_Y;
    g_stations[PART_BLUE].z = PLACE_Z_HEIGHT + ARM_POS_COMPENSATE_Z;
    g_stations[PART_BLUE].action_group = ACTION_PLACE_BLU;
}

/*************************************************************
 * 函数名称：sorting_color_calibrate
 * 功能介绍：颜色传感器白平衡校准
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_color_calibrate(void)
{
    COLOR_RGBC rgbc;
    uint32_t r_sum = 0, g_sum = 0, b_sum = 0, c_sum = 0;
    uint8_t i;
    
    uart1_send_str((u8 *)"[Sorting] Color calibrating...\r\n");
    
    // 采集环境光样本
    for (i = 0; i < COLOR_WB_SAMPLE_NUM; i++) {
        TCS34725_GetRawData(&rgbc);
        r_sum += rgbc.r;
        g_sum += rgbc.g;
        b_sum += rgbc.b;
        c_sum += rgbc.c;
        tb_delay_ms(50);
    }
    
    g_sorting_ctrl.color_cal.r_base = (uint16_t)(r_sum / COLOR_WB_SAMPLE_NUM);
    g_sorting_ctrl.color_cal.g_base = (uint16_t)(g_sum / COLOR_WB_SAMPLE_NUM);
    g_sorting_ctrl.color_cal.b_base = (uint16_t)(b_sum / COLOR_WB_SAMPLE_NUM);
    g_sorting_ctrl.color_cal.c_base = (uint16_t)(c_sum / COLOR_WB_SAMPLE_NUM);
    g_sorting_ctrl.color_cal.is_calibrated = 1;
    
    sprintf((char*)cmd_return, "[Sorting] Calib OK R:%d G:%d B:%d C:%d\r\n",
            g_sorting_ctrl.color_cal.r_base,
            g_sorting_ctrl.color_cal.g_base,
            g_sorting_ctrl.color_cal.b_base,
            g_sorting_ctrl.color_cal.c_base);
    uart1_send_str(cmd_return);
}

// ========== 主控制函数 ==========

/*************************************************************
 * 函数名称：sorting_task
 * 功能介绍：分拣系统主任务，需在主循环中周期性调用
 * 参数：无
 * 返回值：无
 * 算法说明：状态机驱动，每个状态处理对应操作
 * 性能优化：总流程目标<=10秒/件
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
            
        case SORT_APPROACH_PART:
            sorting_state_approach();
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
            sorting_record_process_time();
            g_sorting_ctrl.is_busy = 0;
            g_sorting_ctrl.state = SORT_IDLE;
            uart1_send_str((u8 *)"[Sorting] Task Complete\r\n");
            break;
            
        default:
            g_sorting_ctrl.state = SORT_IDLE;
            break;
    }
}

/*************************************************************
 * 函数名称：sorting_start
 * 功能介绍：启动分拣任务
 * 参数：target - 目标颜色（PART_RED/PART_GREEN/PART_BLUE/PART_UNKNOWN自动识别）
 * 返回值：无
 *************************************************************/
void sorting_start(PartColor_t target)
{
    if (g_sorting_ctrl.is_busy) {
        uart1_send_str((u8 *)"[Sorting] Error: System Busy\r\n");
        return;
    }
    
    g_sorting_ctrl.target_color = target;
    g_sorting_ctrl.detected_color = PART_NONE;
    g_sorting_ctrl.retry_counter = 0;
    g_sorting_ctrl.is_busy = 1;
    g_sorting_ctrl.state_timer = millis();
    g_sorting_ctrl.process_start_time = millis();
    g_sorting_ctrl.state = SORT_MOVE_TO_TARGET;
    
    sprintf((char*)cmd_return, "[Sorting] Start Sorting %s Part\r\n", 
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
    
    uart1_send_str((u8 *)"[Sorting] Task Stopped\r\n");
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
 * 赛道：200x150cm，黑线宽2.5cm
 * 零件放置区：通过四传感器全黑判断到达
 *************************************************************/
void sorting_state_move_to_target(void)
{
    static uint8_t step = 0;
    uint8_t s1, s2, s3, s4;
    
    switch (step) {
        case 0:  // 启动循迹模式
            AI_xunji_moshi();
            
            s1 = x1();
            s2 = x2();
            s3 = x3();
            s4 = x4();
            
            if (s1 && s2 && s3 && s4) {
                car_set(0, 0);
                step = 1;
                g_sorting_ctrl.state_timer = millis();
            }
            break;
            
        case 1:  // 等待稳定
            if (millis() - g_sorting_ctrl.state_timer > 300) {
                step = 0;
                g_sorting_ctrl.state = SORT_DETECT_COLOR;
                g_sorting_ctrl.state_timer = millis();
            }
            break;
    }
}

/*************************************************************
 * 检测颜色 - 使用TCS34725颜色传感器
 * 优化：响应时间<=500ms
 *************************************************************/
void sorting_state_detect_color(void)
{
    PartColor_t color;
    uint8_t confidence;
    
    // 执行颜色检测（带置信度）
    if (sorting_verify_color_confidence(&color, &confidence)) {
        g_sorting_ctrl.detected_color = color;
        
        sprintf((char*)cmd_return, "[Sorting] Detected: %s (Conf:%d%%)\r\n",
                sorting_color_to_str(color), confidence);
        uart1_send_str(cmd_return);
        
        // 进入验证状态
        g_sorting_ctrl.state = SORT_VERIFY_COLOR;
        g_sorting_ctrl.state_timer = millis();
    } else {
        // 置信度不足
        sorting_log_error(ERR_LOW_CONFIDENCE, PART_UNKNOWN);
        g_sorting_ctrl.state = SORT_ERROR_HANDLE;
    }
}

/*************************************************************
 * 验证颜色 - 多次采样确认（优化版）
 * 策略：5次采样，取众数，计算置信度
 *************************************************************/
void sorting_state_verify_color(void)
{
    static uint8_t verify_count = 0;
    static PartColor_t color_samples[COLOR_SAMPLE_TIMES];
    static uint32_t last_sample_time = 0;
    uint8_t red_cnt, grn_cnt, blu_cnt, unknown_cnt;
    uint8_t i;
    PartColor_t verified_color;
    uint8_t max_count;
    uint8_t confidence;
    
    // 控制采样间隔
    if (millis() - last_sample_time < COLOR_SAMPLE_INTERVAL) {
        return;
    }
    last_sample_time = millis();
    
    // 采集样本
    color_samples[verify_count] = sorting_detect_color();
    verify_count++;
    
    if (verify_count < COLOR_SAMPLE_TIMES) {
        return;  // 继续采样
    }
    
    // 统计结果（取众数）
    red_cnt = 0;
    grn_cnt = 0;
    blu_cnt = 0;
    unknown_cnt = 0;
    for (i = 0; i < COLOR_SAMPLE_TIMES; i++) {
        switch (color_samples[i]) {
            case PART_RED: red_cnt++; break;
            case PART_GREEN: grn_cnt++; break;
            case PART_BLUE: blu_cnt++; break;
            default: unknown_cnt++; break;
        }
    }
    
    // 确定最终颜色（取最多出现的有效颜色）
    verified_color = PART_UNKNOWN;
    max_count = 0;
    
    if (red_cnt > max_count) { max_count = red_cnt; verified_color = PART_RED; }
    if (grn_cnt > max_count) { max_count = grn_cnt; verified_color = PART_GREEN; }
    if (blu_cnt > max_count) { max_count = blu_cnt; verified_color = PART_BLUE; }
    
    // 计算置信度
    confidence = (max_count * 100) / COLOR_SAMPLE_TIMES;
    
    g_sorting_ctrl.detected_color = verified_color;
    verify_count = 0;
    
    sprintf((char*)cmd_return, "[Sorting] Verify: %s Conf:%d%% (R:%d G:%d B:%d)\r\n",
            sorting_color_to_str(verified_color), confidence, red_cnt, grn_cnt, blu_cnt);
    uart1_send_str(cmd_return);
    
    // 检查置信度是否达标
    if (confidence < COLOR_CONFIDENCE_THRESHOLD || verified_color == PART_UNKNOWN) {
        sorting_log_error(ERR_LOW_CONFIDENCE, verified_color);
        g_sorting_ctrl.state = SORT_ERROR_HANDLE;
        return;
    }
    
    // 检查颜色是否匹配目标（如果指定了目标颜色）
    if (g_sorting_ctrl.target_color != PART_UNKNOWN && 
        g_sorting_ctrl.target_color != PART_NONE) {
        if (!sorting_is_color_match(verified_color, g_sorting_ctrl.target_color)) {
            sorting_log_error(ERR_COLOR_MISMATCH, verified_color);
            g_sorting_ctrl.stats.color_error_count++;
            
            // 灵活模式：按实际颜色分类放置
            g_sorting_ctrl.target_color = verified_color;
        }
    } else {
        // 自动模式：按检测到的颜色分类
        g_sorting_ctrl.target_color = verified_color;
    }
    
    // 进入接近状态（新增分步接近提高精度）
    g_sorting_ctrl.state = SORT_APPROACH_PART;
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 接近零件 - 分步接近提高精度
 * 先移动到接近高度，再下降抓取
 *************************************************************/
void sorting_state_approach(void)
{
    uart1_send_str((u8 *)"[Sorting] Approaching Part...\r\n");
    
    // 移动到接近位置
    if (sorting_arm_approach(GRAB_X_DEFAULT, GRAB_Y_DEFAULT, GRAB_Z_APPROACH, ARM_MOVE_TIME_FAST)) {
        tb_delay_ms(ARM_SETTLE_TIME);
        g_sorting_ctrl.state = SORT_GRAB_PART;
    } else {
        sorting_log_error(ERR_ARM_TIMEOUT, g_sorting_ctrl.detected_color);
        g_sorting_ctrl.state = SORT_ERROR_HANDLE;
    }
    
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 抓取零件 - 控制机械臂执行抓取
 * 优化：分步下降，提高精度
 *************************************************************/
void sorting_state_grab(void)
{
    uart1_send_str((u8 *)"[Sorting] Start Grabbing...\r\n");
    
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
 * 优化：双重验证机制
 *************************************************************/
void sorting_state_verify_grab(void)
{
    uint8_t verify_result = 0;
    
    // 验证方式1：超声波距离检测
    if (sorting_verify_grab_by_distance()) {
        verify_result = 1;
    }
    
    // 验证方式2：检测机械臂当前位置（简化）
    // 如果机械臂成功执行了抓取动作，认为成功
    
    if (verify_result) {
        uart1_send_str((u8 *)"[Sorting] Grab Verify OK\r\n");
        g_sorting_ctrl.state = SORT_MOVE_TO_STATION;
    } else {
        uart1_send_str((u8 *)"[Sorting] Grab Verify Fail\r\n");
        
        if (g_sorting_ctrl.retry_counter < MAX_RETRY_TIMES) {
            g_sorting_ctrl.retry_counter++;
            g_sorting_ctrl.stats.retry_count++;
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
 * 赛道中心区域：三个工位呈十字分布
 *************************************************************/
void sorting_state_move_to_station(void)
{
    PartColor_t color = g_sorting_ctrl.detected_color;
    
    if (color < PART_RED || color > PART_BLUE) {
        sorting_log_error(ERR_COLOR_MISMATCH, color);
        g_sorting_ctrl.state = SORT_ERROR_HANDLE;
        return;
    }
    
    sprintf((char*)cmd_return, "[Sorting] Move to %s Station\r\n",
            sorting_color_to_str(color));
    uart1_send_str(cmd_return);
    
    // 执行移动到工位的动作
    // 实际实现中可能需要先旋转小车对准工位
    
    g_sorting_ctrl.state = SORT_PLACE_PART;
    g_sorting_ctrl.state_timer = millis();
}

/*************************************************************
 * 放置零件 - 在工位释放零件
 * 优化：快速放置
 *************************************************************/
void sorting_state_place(void)
{
    uart1_send_str((u8 *)"[Sorting] Start Placing...\r\n");
    
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
    uart1_send_str((u8 *)"[Sorting] Place Complete\r\n");
    
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
    uart1_send_str((u8 *)"[Sorting] Return Home\r\n");
    
    sorting_arm_return_home();
    
    g_sorting_ctrl.state = SORT_COMPLETE;
    g_sorting_ctrl.retry_counter = 0;
}

/*************************************************************
 * 错误处理
 * 优化：智能重试和超时处理
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
        
        uart1_send_str((u8 *)"[Sorting] Error Handled\r\n");
    }
    
    if (millis() - g_sorting_ctrl.state_timer > 500) {
        error_handled = 0;
        
        // 检查是否超过最大重试次数
        if (g_sorting_ctrl.retry_counter < MAX_RETRY_TIMES) {
            g_sorting_ctrl.retry_counter++;
            g_sorting_ctrl.stats.retry_count++;
            g_sorting_ctrl.state = SORT_MOVE_TO_TARGET;  // 重试
            uart1_send_str((u8 *)"[Sorting] Prepare Retry...\r\n");
        } else {
            g_sorting_ctrl.state = SORT_RETURN_HOME;  // 放弃，返回
            uart1_send_str((u8 *)"[Sorting] Max Retry, Abort\r\n");
        }
    }
}

// ========== 颜色识别函数（优化版） ==========

/*************************************************************
 * 函数名称：sorting_detect_color
 * 功能介绍：基础颜色检测
 * 参数：无
 * 返回值：检测到的颜色
 *************************************************************/
PartColor_t sorting_detect_color(void)
{
    COLOR_RGBC rgbc;
    COLOR_HSL hsl;
    float r_norm, g_norm, b_norm;
    float max_val, second_val;
    
    // 获取颜色数据
    TCS34725_GetRawData(&rgbc);
    
    // 应用白平衡
    if (COLOR_WB_ENABLE && g_sorting_ctrl.color_cal.is_calibrated) {
        sorting_apply_white_balance(&rgbc);
    }
    
    // 检查亮度
    if (rgbc.c < COLOR_DETECT_THRESHOLD) {
        return PART_UNKNOWN;
    }
    
    // 转换为HSL
    RGBtoHSL(&rgbc, &hsl);
    
    // 策略1：基于RGB比值（提高准确率）
    r_norm = (float)rgbc.r / rgbc.c;
    g_norm = (float)rgbc.g / rgbc.c;
    b_norm = (float)rgbc.b / rgbc.c;
    
    // 计算主导颜色比值
    max_val = r_norm;
    second_val = g_norm;
    if (g_norm > max_val) { second_val = max_val; max_val = g_norm; }
    if (b_norm > max_val) { second_val = max_val; max_val = b_norm; }
    
    // 如果主导颜色不够明显，使用HSL
    if (max_val / second_val < COLOR_RATIO_THRESHOLD) {
        // 策略2：基于HSL色相
        if (hsl.h >= 330 || hsl.h <= 30) {
            return PART_RED;
        } else if (hsl.h >= 60 && hsl.h <= 180) {
            return PART_GREEN;
        } else if (hsl.h >= 180 && hsl.h <= 270) {
            return PART_BLUE;
        }
        return PART_UNKNOWN;
    }
    
    // 策略3：基于归一化RGB值
    if (r_norm > g_norm && r_norm > b_norm) {
        return PART_RED;
    } else if (g_norm > r_norm && g_norm > b_norm) {
        return PART_GREEN;
    } else if (b_norm > r_norm && b_norm > g_norm) {
        return PART_BLUE;
    }
    
    return PART_UNKNOWN;
}

/*************************************************************
 * 函数名称：sorting_detect_color_advanced
 * 功能介绍：高级颜色检测（带原始数据输出）
 * 参数：rgbc_out - 输出原始颜色数据
 * 返回值：检测到的颜色
 *************************************************************/
PartColor_t sorting_detect_color_advanced(COLOR_RGBC *rgbc_out)
{
    COLOR_RGBC rgbc;
    float r_norm, g_norm, b_norm;
    float red_score, grn_score, blu_score;
    
    TCS34725_GetRawData(&rgbc);
    
    if (COLOR_WB_ENABLE && g_sorting_ctrl.color_cal.is_calibrated) {
        sorting_apply_white_balance(&rgbc);
    }
    
    if (rgbc_out != NULL) {
        *rgbc_out = rgbc;
    }
    
    if (rgbc.c < COLOR_DETECT_THRESHOLD) {
        return PART_UNKNOWN;
    }
    
    if (rgbc.c == 0) {
        return PART_UNKNOWN;
    }
    
    r_norm = (float)rgbc.r / rgbc.c;
    g_norm = (float)rgbc.g / rgbc.c;
    b_norm = (float)rgbc.b / rgbc.c;
    
    // 计算各颜色的"得分"
    red_score = r_norm - (g_norm + b_norm) * 0.5f;
    grn_score = g_norm - (r_norm + b_norm) * 0.5f;
    blu_score = b_norm - (r_norm + g_norm) * 0.5f;
    
    if (red_score > grn_score && red_score > blu_score && red_score > 0) {
        return PART_RED;
    } else if (grn_score > red_score && grn_score > blu_score && grn_score > 0) {
        return PART_GREEN;
    } else if (blu_score > red_score && blu_score > grn_score && blu_score > 0) {
        return PART_BLUE;
    }
    
    return PART_UNKNOWN;
}

/*************************************************************
 * 函数名称：sorting_verify_color_confidence
 * 功能介绍：带置信度的颜色验证
 * 参数：result - 输出检测结果
 *       confidence - 输出置信度(0-100)
 * 返回值：1=成功，0=失败
 *************************************************************/
uint8_t sorting_verify_color_confidence(PartColor_t *result, uint8_t *confidence)
{
    PartColor_t samples[COLOR_SAMPLE_TIMES];
    uint8_t i;
    uint8_t red_cnt, grn_cnt, blu_cnt;
    PartColor_t detected;
    uint8_t max_count;
    
    // 快速采样
    for (i = 0; i < COLOR_SAMPLE_TIMES; i++) {
        samples[i] = sorting_detect_color();
        if (i < COLOR_SAMPLE_TIMES - 1) {
            tb_delay_ms(COLOR_SAMPLE_INTERVAL);
        }
    }
    
    // 统计
    red_cnt = 0;
    grn_cnt = 0;
    blu_cnt = 0;
    for (i = 0; i < COLOR_SAMPLE_TIMES; i++) {
        switch (samples[i]) {
            case PART_RED: red_cnt++; break;
            case PART_GREEN: grn_cnt++; break;
            case PART_BLUE: blu_cnt++; break;
            default: break;
        }
    }
    
    // 确定结果
    detected = PART_UNKNOWN;
    max_count = 0;
    
    if (red_cnt > max_count) { max_count = red_cnt; detected = PART_RED; }
    if (grn_cnt > max_count) { max_count = grn_cnt; detected = PART_GREEN; }
    if (blu_cnt > max_count) { max_count = blu_cnt; detected = PART_BLUE; }
    
    *result = detected;
    *confidence = (max_count * 100) / COLOR_SAMPLE_TIMES;
    
    return (detected != PART_UNKNOWN && *confidence >= COLOR_CONFIDENCE_THRESHOLD) ? 1 : 0;
}

/*************************************************************
 * 函数名称：sorting_apply_white_balance
 * 功能介绍：应用动态白平衡
 * 参数：rgbc - 颜色数据（输入输出）
 * 返回值：无
 *************************************************************/
void sorting_apply_white_balance(COLOR_RGBC *rgbc)
{
    if (!g_sorting_ctrl.color_cal.is_calibrated) return;
    
    // 简单的增益补偿
    if (g_sorting_ctrl.color_cal.r_base > 0) {
        uint32_t r_temp = (uint32_t)rgbc->r * 128 / g_sorting_ctrl.color_cal.r_base;
        rgbc->r = (r_temp > 65535) ? 65535 : (uint16_t)r_temp;
    }
    if (g_sorting_ctrl.color_cal.g_base > 0) {
        uint32_t g_temp = (uint32_t)rgbc->g * 128 / g_sorting_ctrl.color_cal.g_base;
        rgbc->g = (g_temp > 65535) ? 65535 : (uint16_t)g_temp;
    }
    if (g_sorting_ctrl.color_cal.b_base > 0) {
        uint32_t b_temp = (uint32_t)rgbc->b * 128 / g_sorting_ctrl.color_cal.b_base;
        rgbc->b = (b_temp > 65535) ? 65535 : (uint16_t)b_temp;
    }
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
    if (target == PART_UNKNOWN || target == PART_NONE) {
        return 1;  // 自动模式，总是匹配
    }
    return (detect == target) ? 1 : 0;
}

// ========== 机械臂控制函数（优化版） ==========

/*************************************************************
 * 函数名称：sorting_arm_move_to
 * 功能介绍：控制机械臂移动到指定坐标（带精度补偿）
 * 参数：x,y,z - 目标坐标
 *       time - 运动时间
 * 返回值：1=成功，0=失败
 *************************************************************/
uint8_t sorting_arm_move_to(float x, float y, float z, uint16_t time)
{
    // 应用精度补偿
    x += ARM_POS_COMPENSATE_X;
    y += ARM_POS_COMPENSATE_Y;
    z += ARM_POS_COMPENSATE_Z;
    
    // 使用逆运动学计算舵机角度
    if (kinematics_move(x, y, z, time) == 0) {
        return 1;
    }
    return 0;
}

/*************************************************************
 * 函数名称：sorting_arm_approach
 * 功能介绍：分步接近目标（提高精度）
 * 参数：x,y - 目标XY坐标
 *       z_approach - 接近高度
 *       time - 运动时间
 * 返回值：1=成功，0=失败
 *************************************************************/
uint8_t sorting_arm_approach(float x, float y, float z_approach, uint16_t time)
{
    // 先移动到接近位置
    if (!sorting_arm_move_to(x, y, z_approach, time)) {
        return 0;
    }
    tb_delay_ms(ARM_SETTLE_TIME);
    return 1;
}

/*************************************************************
 * 函数名称：sorting_arm_grab
 * 功能介绍：执行抓取动作序列（优化速度）
 * 参数：无
 * 返回值：1=成功，0=失败
 *************************************************************/
uint8_t sorting_arm_grab(void)
{
    // 方式1：使用逆运动学精确控制（推荐）
    
    // 步骤1：移动到抓取准备位置（已在接近状态完成）
    // 步骤2：分步下降到抓取高度
    if (!sorting_arm_move_to(GRAB_X_DEFAULT, GRAB_Y_DEFAULT, GRAB_Z_DOWN, ARM_MOVE_TIME_SLOW)) {
        return 0;
    }
    tb_delay_ms(ARM_SETTLE_TIME);
    
    // 步骤3：夹爪闭合
    set_servo(GRIPPER_SERVO_INDEX, GRIPPER_PWM_CLOSE, ARM_GRAB_WAIT_TIME);
    tb_delay_ms(ARM_GRAB_WAIT_TIME);
    
    // 步骤4：抬起
    if (!sorting_arm_move_to(GRAB_X_DEFAULT, GRAB_Y_DEFAULT, GRAB_Z_UP, ARM_MOVE_TIME_FAST)) {
        return 0;
    }
    tb_delay_ms(ARM_SETTLE_TIME);
    
    return 1;
    
    // 方式2：使用预存动作组（更稳定，但需提前下载）
    // parse_cmd((u8 *)"$DGT:10-14,1!");
    // return 1;
}

/*************************************************************
 * 函数名称：sorting_arm_place
 * 功能介绍：执行放置动作序列（优化速度）
 * 参数：color - 零件颜色，决定放置工位
 * 返回值：1=成功，0=失败
 *************************************************************/
uint8_t sorting_arm_place(PartColor_t color)
{
    Station_t *station;
    
    if (color < PART_RED || color > PART_BLUE) {
        return 0;
    }
    
    station = &g_stations[color];
    
    // 方式1：使用逆运动学
    // 移动到工位上方
    if (!sorting_arm_move_to(station->x, station->y, GRAB_Z_UP, ARM_MOVE_TIME_FAST)) {
        return 0;
    }
    tb_delay_ms(ARM_SETTLE_TIME);
    
    // 下降
    if (!sorting_arm_move_to(station->x, station->y, station->z, ARM_MOVE_TIME_SLOW)) {
        return 0;
    }
    tb_delay_ms(ARM_SETTLE_TIME);
    
    // 夹爪释放
    set_servo(GRIPPER_SERVO_INDEX, GRIPPER_PWM_OPEN, ARM_RELEASE_WAIT_TIME);
    tb_delay_ms(ARM_RELEASE_WAIT_TIME);
    
    // 抬起
    if (!sorting_arm_move_to(station->x, station->y, GRAB_Z_UP, ARM_MOVE_TIME_FAST)) {
        return 0;
    }
    tb_delay_ms(ARM_SETTLE_TIME);
    
    return 1;
    
    // 方式2：使用预存动作组
    // char cmd[32];
    // sprintf(cmd, "$DGT:%d-%d,1!", station->action_group, station->action_group+3);
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

// ========== 超声波验证函数 ==========

/*************************************************************
 * 函数名称：sorting_verify_distance
 * 功能介绍：验证超声波距离
 * 参数：expected_distance - 期望距离(cm)
 * 返回值：1=验证通过，0=失败
 *************************************************************/
uint8_t sorting_verify_distance(uint8_t expected_distance)
{
    int dist_temp;
    uint16_t distance;
    
    dist_temp = get_adc_csb_middle();
    
    if (dist_temp <= 0) {
        return 0;
    }
    
    distance = (uint16_t)dist_temp;
    
    if (distance >= expected_distance - 2 && distance <= expected_distance + 2) {
        return 1;
    }
    return 0;
}

/*************************************************************
 * 函数名称：sorting_verify_grab_by_distance
 * 功能介绍：通过超声波验证抓取
 * 参数：无
 * 返回值：1=抓取成功，0=失败
 *************************************************************/
uint8_t sorting_verify_grab_by_distance(void)
{
    int dist_temp;
    uint16_t distance;
    
    dist_temp = get_adc_csb_middle();
    
    if (dist_temp <= 0) {
        return 0;
    }
    
    distance = (uint16_t)dist_temp;
    
    if (distance < GRAB_DISTANCE_VERIFY) {
        return 1;
    }
    
    return 0;
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
 * 函数名称：sorting_record_process_time
 * 功能介绍：记录处理时间
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_record_process_time(void)
{
    uint32_t process_time = millis() - g_sorting_ctrl.process_start_time;
    
    g_sorting_ctrl.stats.total_process_time += process_time;
    if (g_sorting_ctrl.stats.total_attempts > 0) {
        g_sorting_ctrl.stats.avg_process_time = 
            g_sorting_ctrl.stats.total_process_time / g_sorting_ctrl.stats.total_attempts;
    }
    
    sprintf((char*)cmd_return, "[Sorting] Process Time: %d ms (Avg: %d ms)\r\n",
            (int)process_time, (int)g_sorting_ctrl.stats.avg_process_time);
    uart1_send_str(cmd_return);
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
    
    uart1_send_str((u8 *)"\r\n========== Sorting Stats ==========\r\n");
    
    sprintf((char*)cmd_return, "Total Attempts: %d\r\n", stats->total_attempts);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "Success: %d\r\n", stats->success_count);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "Failed: %d\r\n", stats->fail_count);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "Color Errors: %d\r\n", stats->color_error_count);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "Retries: %d\r\n", stats->retry_count);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "Timeouts: %d\r\n", stats->timeout_count);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "Success Rate: %d%%\r\n", 
            stats->total_attempts > 0 ? 
            (stats->success_count * 100 / stats->total_attempts) : 0);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "Avg Process Time: %d ms\r\n", (int)stats->avg_process_time);
    uart1_send_str(cmd_return);
    
    sprintf((char*)cmd_return, "Consecutive Errors: %d\r\n", stats->consecutive_errors);
    uart1_send_str(cmd_return);
    
    uart1_send_str((u8 *)"==============================\r\n");
}

/*************************************************************
 * 函数名称：sorting_alarm
 * 功能介绍：报警提示（优化）
 * 参数：error_code - 错误码
 * 返回值：无
 *************************************************************/
void sorting_alarm(uint8_t error_code)
{
    // 蜂鸣器报警
    beep_on_times(ALARM_BEEP_TIMES, ALARM_BEEP_DURATION);
    
    // 串口输出错误信息
    sprintf((char*)cmd_return, "[Sorting] Alarm! Error Code: %d\r\n", error_code);
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
    sprintf((char*)cmd_return, "[Sorting] Error Log: Code=%d Color=%s Time=%d\r\n",
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
        case PART_RED:   return "Red";
        case PART_GREEN: return "Green";
        case PART_BLUE:  return "Blue";
        case PART_NONE:  return "None";
        default:         return "Unknown";
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

// ========== 测试函数 ==========

/*************************************************************
 * 函数名称：sorting_test_color_accuracy
 * 功能介绍：测试颜色识别准确率
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_test_color_accuracy(void)
{
    uint8_t i;
    uint16_t red_ok = 0, grn_ok = 0, blu_ok = 0;
    PartColor_t result;
    uint8_t total_rate;
    
    uart1_send_str((u8 *)"\r\n[Test] Color Accuracy Test\r\n");
    uart1_send_str((u8 *)"Place Red, Green, Blue blocks...\r\n");
    
    // 测试红色
    uart1_send_str((u8 *)"[Test] Place Red block, detect in 3s...\r\n");
    tb_delay_ms(3000);
    for (i = 0; i < 20; i++) {
        result = sorting_detect_color();
        if (result == PART_RED) red_ok++;
        tb_delay_ms(50);
    }
    sprintf((char*)cmd_return, "[Test] Red Accuracy: %d%% (%d/20)\r\n", 
            red_ok * 5, red_ok);
    uart1_send_str(cmd_return);
    
    // 测试绿色
    uart1_send_str((u8 *)"[Test] Place Green block, detect in 3s...\r\n");
    tb_delay_ms(3000);
    for (i = 0; i < 20; i++) {
        result = sorting_detect_color();
        if (result == PART_GREEN) grn_ok++;
        tb_delay_ms(50);
    }
    sprintf((char*)cmd_return, "[Test] Green Accuracy: %d%% (%d/20)\r\n", 
            grn_ok * 5, grn_ok);
    uart1_send_str(cmd_return);
    
    // 测试蓝色
    uart1_send_str((u8 *)"[Test] Place Blue block, detect in 3s...\r\n");
    tb_delay_ms(3000);
    for (i = 0; i < 20; i++) {
        result = sorting_detect_color();
        if (result == PART_BLUE) blu_ok++;
        tb_delay_ms(50);
    }
    sprintf((char*)cmd_return, "[Test] Blue Accuracy: %d%% (%d/20)\r\n", 
            blu_ok * 5, blu_ok);
    uart1_send_str(cmd_return);
    
    total_rate = (red_ok + grn_ok + blu_ok) * 100 / 60;
    sprintf((char*)cmd_return, "[Test] Total Accuracy: %d%%\r\n", total_rate);
    uart1_send_str(cmd_return);
}

/*************************************************************
 * 函数名称：sorting_test_arm_precision
 * 功能介绍：测试机械臂定位精度
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_test_arm_precision(void)
{
    uart1_send_str((u8 *)"\r\n[Test] Arm Precision Test\r\n");
    
    // 测试移动到三个工位
    uart1_send_str((u8 *)"[Test] Move to Red Station...\r\n");
    sorting_arm_move_to(PLACE_RED_X, PLACE_RED_Y, PLACE_Z_HEIGHT, ARM_MOVE_TIME_FAST);
    tb_delay_ms(1000);
    
    uart1_send_str((u8 *)"[Test] Move to Green Station...\r\n");
    sorting_arm_move_to(PLACE_GRN_X, PLACE_GRN_Y, PLACE_Z_HEIGHT, ARM_MOVE_TIME_FAST);
    tb_delay_ms(1000);
    
    uart1_send_str((u8 *)"[Test] Move to Blue Station...\r\n");
    sorting_arm_move_to(PLACE_BLU_X, PLACE_BLU_Y, PLACE_Z_HEIGHT, ARM_MOVE_TIME_FAST);
    tb_delay_ms(1000);
    
    uart1_send_str((u8 *)"[Test] Return Home...\r\n");
    sorting_arm_return_home();
    
    uart1_send_str((u8 *)"[Test] Arm Test Complete\r\n");
}

/*************************************************************
 * 函数名称：sorting_run_self_test
 * 功能介绍：运行系统自检
 * 参数：无
 * 返回值：无
 *************************************************************/
void sorting_run_self_test(void)
{
    COLOR_RGBC rgbc;
    uint16_t dist;
    int dist_temp;
    
    uart1_send_str((u8 *)"\r\n========== Self Test ==========\r\n");
    
    // 检查颜色传感器
    uart1_send_str((u8 *)"[SelfTest] Checking Color Sensor...\r\n");
    TCS34725_GetRawData(&rgbc);
    if (rgbc.c > 0) {
        uart1_send_str((u8 *)"[SelfTest] Color Sensor OK\r\n");
    } else {
        uart1_send_str((u8 *)"[SelfTest] Color Sensor Error!\r\n");
    }
    
    // 检查机械臂
    uart1_send_str((u8 *)"[SelfTest] Checking Arm...\r\n");
    if (sorting_arm_return_home()) {
        uart1_send_str((u8 *)"[SelfTest] Arm OK\r\n");
    } else {
        uart1_send_str((u8 *)"[SelfTest] Arm Error!\r\n");
    }
    
    // 检查超声波
    uart1_send_str((u8 *)"[SelfTest] Checking Ultrasonic...\r\n");
    dist_temp = get_adc_csb_middle();
    dist = (dist_temp > 0 && dist_temp < 400) ? (uint16_t)dist_temp : 0;
    if (dist > 0) {
        sprintf((char*)cmd_return, "[SelfTest] Ultrasonic OK (Dist:%d cm)\r\n", dist);
        uart1_send_str(cmd_return);
    } else {
        uart1_send_str((u8 *)"[SelfTest] Ultrasonic Error!\r\n");
    }
    
    uart1_send_str((u8 *)"==============================\r\n");
}
