#include <stdio.h>
#include <string.h>
#include "z_tracking.h"
#include "z_sensor.h"
#include "z_main.h"
#include "z_delay.h"
#include "z_usart.h"

/*******************************************************************************
 * 全局变量定义
 ******************************************************************************/

/* 位置查表（16种传感器状态对应的位置估计值） */
/* 索引：s1(bit3) | s2(bit2) | s3(bit1) | s4(bit0) */
/* 0=检测到黑线(低电平)，1=白色(高电平) */
static const int8_t g_position_table[16] = {
    /* 0000 全黑 */  POS_ALL_BLACK,  /* 100 */
    /* 0001     */   30,             /* 仅右外 */
    /* 0010     */   12,             /* 仅右内 */
    /* 0011     */   20,             /* 右内+右外 */
    /* 0100     */  -12,             /* 仅左内 */
    /* 0101     */    0,             /* 左内+右内 */
    /* 0110     */    0,             /* 左内+右内（居中） */
    /* 0111     */   15,             /* 左内+右内+右外 */
    /* 1000     */  -30,             /* 仅左外 */
    /* 1001     */  -20,             /* 左外+右外 */
    /* 1010     */  -10,             /* 左外+右内 */
    /* 1011     */    5,             /* 左外+右内+右外 */
    /* 1100     */  -20,             /* 左外+左内 */
    /* 1101     */   -5,             /* 左外+左内+右外 */
    /* 1110     */  -15,             /* 左外+左内+右内 */
    /* 1111 全白*/   POS_ALL_WHITE   /* 99 */
};

/* PID控制器实例 */
static PID_TypeDef g_pid = {
    PID_KP_MID_ERR,  /* Kp */
    PID_KI_MID_ERR,  /* Ki */
    PID_KD_MID_ERR,  /* Kd */
    0, 0, 0, 0       /* err, err_last, integral, output */
};

/* 当前平滑后的速度 */
static int16_t g_current_left_speed = 0;
static int16_t g_current_right_speed = 0;

/* 标记检测状态 */
static uint32_t g_black_start_time = 0;
static uint8_t g_black_flag = 0;

/* 脱线恢复状态 */
static uint8_t g_lost_flag = 0;
static uint8_t g_lost_counter = 0;
static int8_t g_last_valid_dir = 0;

/* 系统状态 */
static Tracking_State_t g_tracking_state = {0};

/* 可调整参数 */
static uint8_t g_speed_straight = SPEED_STRAIGHT;
static uint8_t g_speed_curve = SPEED_CURVE;
static uint8_t g_speed_slow = SPEED_SLOW;

/*******************************************************************************
 * 初始化函数
 ******************************************************************************/
void tracking_init(void)
{
    /* 初始化PID参数 */
    g_pid.Kp = PID_KP_MID_ERR;
    g_pid.Ki = PID_KI_MID_ERR;
    g_pid.Kd = PID_KD_MID_ERR;
    g_pid.err = 0;
    g_pid.err_last = 0;
    g_pid.integral = 0;
    g_pid.output = 0;
    
    /* 初始化速度 */
    g_current_left_speed = 0;
    g_current_right_speed = 0;
    
    /* 初始化脱线状态 */
    g_lost_flag = 0;
    g_lost_counter = 0;
    g_last_valid_dir = 0;
    
    /* 初始化标记检测 */
    g_black_flag = 0;
    g_black_start_time = 0;
    
    /* 初始化状态结构体 */
    memset(&g_tracking_state, 0, sizeof(g_tracking_state));
    
    /* 初始化默认速度 */
    g_speed_straight = SPEED_STRAIGHT;
    g_speed_curve = SPEED_CURVE;
    g_speed_slow = SPEED_SLOW;
}

/*******************************************************************************
 * 位置查表函数
 ******************************************************************************/
int8_t tracking_get_position(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4)
{
    uint8_t idx = SENSOR_IDX(s1, s2, s3, s4);
    return g_position_table[idx];
}

/*******************************************************************************
 * PID控制器计算
 ******************************************************************************/
int16_t tracking_pid_calc(int16_t setpoint, int16_t measured)
{
    int16_t err_abs;
    int16_t base_speed;
    int16_t P, I, D;
    
    /* 1. 计算误差 */
    g_pid.err = setpoint - measured;
    
    /* 2. 根据误差绝对值选择参数 */
    err_abs = (g_pid.err >= 0) ? g_pid.err : -(g_pid.err);
    
    if (err_abs < 5) {
        /* 小误差：追求平稳，高速 */
        g_pid.Kp = PID_KP_SMALL_ERR;
        g_pid.Kd = PID_KD_SMALL_ERR;
        base_speed = g_speed_straight;
        /* 小误差时清零积分 */
        g_pid.integral = 0;
    } else if (err_abs < 15) {
        /* 中等误差：标准响应，中速 */
        g_pid.Kp = PID_KP_MID_ERR;
        g_pid.Kd = PID_KD_MID_ERR;
        base_speed = g_speed_curve;
    } else {
        /* 大误差：快速纠偏，低速 */
        g_pid.Kp = PID_KP_LARGE_ERR;
        g_pid.Kd = PID_KD_LARGE_ERR;
        base_speed = g_speed_slow;
    }
    
    /* 3. 计算PID三项 */
    /* 比例项 */
    P = (g_pid.Kp * g_pid.err) / 10;
    
    /* 积分项（带限幅） */
    if (err_abs >= 5) {
        g_pid.integral += g_pid.err;
        if (g_pid.integral > PID_INTEGRAL_MAX) g_pid.integral = PID_INTEGRAL_MAX;
        if (g_pid.integral < PID_INTEGRAL_MIN) g_pid.integral = PID_INTEGRAL_MIN;
    }
    I = (g_pid.Ki * g_pid.integral) / 100;
    
    /* 微分项 */
    D = (g_pid.Kd * (g_pid.err - g_pid.err_last)) / 10;
    g_pid.err_last = g_pid.err;
    
    /* 4. 合成输出 */
    g_pid.output = P + I + D;
    
    /* 5. 输出限幅 */
    if (g_pid.output > PID_OUTPUT_MAX) g_pid.output = PID_OUTPUT_MAX;
    if (g_pid.output < PID_OUTPUT_MIN) g_pid.output = PID_OUTPUT_MIN;
    
    return base_speed;
}

/*******************************************************************************
 * 根据PID输出计算左右轮目标速度
 ******************************************************************************/
void tracking_calc_speed(int16_t pid_output, int16_t base_speed, 
                         int16_t *left_speed, int16_t *right_speed)
{
    /* PID输出 > 0：偏左，需要右转 -> 左轮加速，右轮减速 */
    /* PID输出 < 0：偏右，需要左转 -> 左轮减速，右轮加速 */
    *left_speed  = base_speed - (pid_output * STEER_FACTOR) / 10;
    *right_speed = base_speed + (pid_output * STEER_FACTOR) / 10;
    
    /* 限幅保护 */
    if (*left_speed > 100) *left_speed = 100;
    if (*left_speed < -100) *left_speed = -100;
    if (*right_speed > 100) *right_speed = 100;
    if (*right_speed < -100) *right_speed = -100;
}

/*******************************************************************************
 * 速度平滑处理
 ******************************************************************************/
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

/*******************************************************************************
 * 标记检测函数
 ******************************************************************************/
TrackMark_t tracking_detect_mark(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4)
{
    uint8_t is_all_black = (s1==SENSOR_BLACK && s2==SENSOR_BLACK && 
                           s3==SENSOR_BLACK && s4==SENSOR_BLACK);
    uint32_t current_time = millis();
    
    if (is_all_black && !g_black_flag) {
        /* 首次检测到全黑 */
        g_black_flag = 1;
        g_black_start_time = current_time;
        return MARK_NONE;
        
    } else if (is_all_black && g_black_flag) {
        /* 持续全黑，根据时间判断类型 */
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
        /* 全黑结束 */
        uint32_t duration = current_time - g_black_start_time;
        g_black_flag = 0;
        
        /* 短脉冲可能是5x5标记 */
        if (duration >= MARK_5X5_MIN_MS && duration < MARK_5X5_MAX_MS) {
            g_tracking_state.last_mark = MARK_5X5;
            return MARK_5X5;
        }
    }
    
    return MARK_NONE;
}

/*******************************************************************************
 * 处理检测到的赛道标记
 ******************************************************************************/
void tracking_handle_mark(TrackMark_t mark)
{
    switch (mark) {
        case MARK_5X5:
            /* 5x5cm标记 */
            break;
            
        case MARK_15X15:
            /* 15x15cm标记 */
            beep_on_times(1, 100);
            break;
            
        case MARK_40X40:
            /* 40x40cm区域 */
            car_set(5, 5);
            break;
            
        case MARK_CROSS:
            /* 十字路口 */
            break;
            
        default:
            break;
    }
}

/*******************************************************************************
 * 脱线恢复处理 - 小幅多次转向策略
 ******************************************************************************/
void tracking_handle_lost(void)
{
    static uint8_t recovery_step = 0;
    static uint32_t recovery_timer = 0;
    static uint8_t turn_count = 0;      /* 转向次数计数 */
    static int8_t turn_direction = 1;   /* 当前转向方向: 1=右, -1=左 */
    static uint8_t sub_step = 0;        /* 子步骤: 0=转向, 1=停顿检测 */
    
    uint8_t s1, s2, s3, s4;
    
    switch (recovery_step) {
        case 0:  /* 制动 */
            car_set(0, 0);
            recovery_timer = millis();
            turn_count = 0;
            sub_step = 0;
            /* 根据最后已知方向确定初始转向 */
            if (g_last_valid_dir < 0) {
                turn_direction = 1;  /* 最后偏左，先向右转 */
            } else if (g_last_valid_dir > 0) {
                turn_direction = -1; /* 最后偏右，先向左转 */
            } else {
                turn_direction = 1;  /* 未知方向，默认向右 */
            }
            recovery_step = 1;
            break;
            
        case 1:  /* 等待稳定 */
            if (millis() - recovery_timer > 30) {
                recovery_step = 2;
                recovery_timer = millis();
            }
            break;
            
        case 2:  /* 小幅多次转向搜索 */
            if (sub_step == 0) {
                /* 子步骤0: 执行小幅转向 */
                if (turn_direction > 0) {
                    /* 向右小幅转 */
                    car_set(LOST_RECOVERY_SPEED_L, LOST_RECOVERY_SPEED_R);
                } else {
                    /* 向左小幅转 */
                    car_set(-LOST_RECOVERY_SPEED_R, LOST_RECOVERY_SPEED_L);
                }
                
                /* 检查是否找到线 */
                s1 = x1(); s2 = x2(); s3 = x3(); s4 = x4();
                if (!(s1 && s2 && s3 && s4)) {
                    /* 找到线了 */
                    car_set(0, 0);
                    recovery_step = 0;
                    g_lost_flag = 0;
                    g_lost_counter = 0;
                    g_pid.integral = 0;
                    beep_on_times(2, 50);
                    break;
                }
                
                /* 转向一小段时间后停顿 */
                if (millis() - recovery_timer > 60) {
                    car_set(0, 0);  /* 停顿 */
                    sub_step = 1;
                    recovery_timer = millis();
                }
                
            } else {
                /* 子步骤1: 停顿并检测 */
                s1 = x1(); s2 = x2(); s3 = x3(); s4 = x4();
                
                /* 停顿期间检测 */
                if (!(s1 && s2 && s3 && s4)) {
                    /* 找到线了 */
                    car_set(0, 0);
                    recovery_step = 0;
                    g_lost_flag = 0;
                    g_lost_counter = 0;
                    g_pid.integral = 0;
                    beep_on_times(2, 50);
                    break;
                }
                
                /* 停顿40ms后准备下一次转向 */
                if (millis() - recovery_timer > 40) {
                    turn_count++;
                    
                    /* 每转2次后换方向，逐渐扩大搜索范围 */
                    if (turn_count % 2 == 0) {
                        turn_direction = -turn_direction;
                    }
                    
                    /* 检查是否超时 */
                    if (millis() - recovery_timer > LOST_RECOVERY_TIME) {
                        car_set(0, 0);
                        recovery_step = 0;
                        g_lost_flag = 1;
                        beep_on_times(5, 200);
                        break;
                    }
                    
                    /* 准备下一次转向 */
                    sub_step = 0;
                    recovery_timer = millis();
                }
            }
            break;
    }
}

/*******************************************************************************
 * 核心巡线更新函数（主循环调用）
 ******************************************************************************/
void tracking_update(void)
{
    uint8_t s1, s2, s3, s4;
    uint8_t is_all_black;
    uint8_t is_all_white;
    int8_t position;
    int16_t base_speed;
    int16_t target_left, target_right;
    TrackMark_t mark;
    
    /* 1. 读取传感器值 */
    s1 = x1();  /* 左外 PA1 */
    s2 = x2();  /* 左内 PA0 */
    s3 = x3();  /* 右内 PA3 */
    s4 = x4();  /* 右外 PB1 */
    
    /* 保存原始值到状态结构体 */
    g_tracking_state.sensor_raw = (s1<<3) | (s2<<2) | (s3<<1) | s4;
    
    /* 2. 检查全黑（标记检测） */
    is_all_black = (s1==SENSOR_BLACK && s2==SENSOR_BLACK && 
                   s3==SENSOR_BLACK && s4==SENSOR_BLACK);
    
    if (is_all_black) {
        mark = tracking_detect_mark(s1, s2, s3, s4);
        if (mark != MARK_NONE) {
            tracking_handle_mark(mark);
            if (mark == MARK_CROSS || mark == MARK_5X5) {
                return;  /* 保持当前速度直行 */
            }
        }
    }
    
    /* 3. 检查全白（脱线检测） */
    is_all_white = (s1==SENSOR_WHITE && s2==SENSOR_WHITE && 
                   s3==SENSOR_WHITE && s4==SENSOR_WHITE);
    
    if (is_all_white) {
        g_lost_counter++;
        
        if (g_lost_counter > LOST_THRESHOLD) {
            if (!g_lost_flag) {
                g_lost_flag = 1;
                /* 记录最后已知方向 */
                if (s1 == SENSOR_BLACK) g_last_valid_dir = -1;
                else if (s4 == SENSOR_BLACK) g_last_valid_dir = 1;
            }
            tracking_handle_lost();
            return;
        }
    } else {
        /* 正常状态，重置脱线计数 */
        g_lost_counter = 0;
        g_lost_flag = 0;
        
        /* 记录方向 */
        if (s1 == SENSOR_BLACK) g_last_valid_dir = -1;
        else if (s4 == SENSOR_BLACK) g_last_valid_dir = 1;
        else g_last_valid_dir = 0;
    }
    
    /* 4. 获取位置估计值 */
    position = tracking_get_position(s1, s2, s3, s4);
    g_tracking_state.position = position;
    
    /* 处理特殊值 */
    if (position >= POS_ALL_WHITE) {
        position = 0;
    }
    
    /* 5. PID计算 */
    base_speed = tracking_pid_calc(0, position);
    
    /* 6. 计算目标速度 */
    tracking_calc_speed(g_pid.output, base_speed, &target_left, &target_right);
    
    /* 7. 速度平滑 */
    g_current_left_speed = tracking_smooth_speed(target_left, g_current_left_speed);
    g_current_right_speed = tracking_smooth_speed(target_right, g_current_right_speed);
    
    /* 8. 保存到状态结构体 */
    g_tracking_state.left_speed = g_current_left_speed;
    g_tracking_state.right_speed = g_current_right_speed;
    g_tracking_state.is_lost = g_lost_flag;
    
    /* 9. 输出到电机 */
    car_set(g_current_left_speed, g_current_right_speed);
}

/*******************************************************************************
 * 获取当前巡线状态
 ******************************************************************************/
void tracking_get_state(Tracking_State_t *state)
{
    if (state != NULL) {
        memcpy(state, &g_tracking_state, sizeof(Tracking_State_t));
    }
}

/*******************************************************************************
 * 打印调试信息
 ******************************************************************************/
void tracking_print_debug(void)
{
    sprintf((char*)cmd_return, 
        "S:%d%d%d%d Pos:%d PID:%d L:%d R:%d %s\r\n",
        (g_tracking_state.sensor_raw>>3)&1,
        (g_tracking_state.sensor_raw>>2)&1,
        (g_tracking_state.sensor_raw>>1)&1,
        g_tracking_state.sensor_raw&1,
        g_tracking_state.position,
        g_pid.output,
        g_tracking_state.left_speed,
        g_tracking_state.right_speed,
        g_tracking_state.is_lost ? "LOST" : "OK"
    );
    uart1_send_str(cmd_return);
}

/*******************************************************************************
 * 设置PID参数
 ******************************************************************************/
void tracking_set_pid(int16_t kp, int16_t ki, int16_t kd)
{
    g_pid.Kp = kp;
    g_pid.Ki = ki;
    g_pid.Kd = kd;
}

/*******************************************************************************
 * 设置速度参数
 ******************************************************************************/
void tracking_set_speed(uint8_t straight, uint8_t curve, uint8_t slow)
{
    g_speed_straight = straight;
    g_speed_curve = curve;
    g_speed_slow = slow;
}

/*******************************************************************************
 * 运行测试
 ******************************************************************************/
void tracking_run_tests(void)
{
    /* C89标准：所有变量声明必须在代码块开头 */
    uint8_t i;
    int8_t pos;
    int8_t s1[9] = {1,1,1,0,1,0,1,1,0};
    int8_t s2[9] = {0,0,1,0,1,1,1,1,0};
    int8_t s3[9] = {0,1,0,1,0,1,1,1,0};
    int8_t s4[9] = {1,1,1,1,0,1,0,1,0};
    int8_t expected[9] = {0, 5, -5, 20, -20, 15, -15, 99, 100};
    char* names[9] = {"Center", "Slight Left", "Slight Right", "Left", "Right",
                      "More Left", "More Right", "All White", "All Black"};
    
    uart1_send_str((u8*)"\r\n========== Tracking Tests ==========\r\n");
    
    /* 测试1：位置查表 */
    uart1_send_str((u8*)"\n--- Position Table Test ---\r\n");
    
    for (i = 0; i < 9; i++) {
        pos = tracking_get_position((uint8_t)s1[i], (uint8_t)s2[i], (uint8_t)s3[i], (uint8_t)s4[i]);
        sprintf((char*)cmd_return, "%s: %d (exp:%d) [%s]\r\n",
            names[i], pos, expected[i],
            (pos == expected[i]) ? "PASS" : "FAIL");
        uart1_send_str(cmd_return);
    }
    
    uart1_send_str((u8*)"\n========== Tests Complete ==========\r\n");
}
