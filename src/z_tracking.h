#ifndef __Z_TRACKING_H__
#define __Z_TRACKING_H__

#include "stm32f10x_conf.h"
#include "z_global.h"

/*******************************************************************************
 * 四路巡线系统配置文件
 * 说明：本文件包含所有巡线算法的参数配置，可根据实际赛道情况调整
 ******************************************************************************/

/*******************************************************************************
 * 传感器配置
 ******************************************************************************/
/* 传感器数量 */
#define TRACKING_SENSOR_NUM     4

/* 传感器逻辑电平定义 */
#define SENSOR_BLACK            0   /* 检测到黑线（低电平） */
#define SENSOR_WHITE            1   /* 检测到白色（高电平） */

/* 传感器位置索引宏（用于查表）
   索引格式：s1(bit3) | s2(bit2) | s3(bit1) | s4(bit0) */
#define SENSOR_IDX(s1, s2, s3, s4)  (((s1)<<3) | ((s2)<<2) | ((s3)<<1) | (s4))

/*******************************************************************************
 * 位置估计值定义（×10放大，避免浮点运算）
 * 范围：-30 ~ +30，负值表示偏左，正值表示偏右
 ******************************************************************************/
#define POS_ALL_BLACK           100     /* 全黑标记（特殊标记点） */
#define POS_ALL_WHITE           99      /* 全白（脱线） */
#define POS_CENTER              0       /* 居中 */
#define POS_LEFT_MAX            -30     /* 最左 */
#define POS_RIGHT_MAX           30      /* 最右 */

/*******************************************************************************
 * PID控制器参数配置
 * 说明：参数已×10或×100放大，计算时需相应除法
 ******************************************************************************/
/* 小误差状态（|err| < 5）：追求平稳，高速直行 */
#define PID_KP_SMALL_ERR        25      /* 比例系数 2.5 */
#define PID_KI_SMALL_ERR        0       /* 积分系数（小误差时不用积分） */
#define PID_KD_SMALL_ERR        80      /* 微分系数 8.0 */

/* 中等误差状态（5 <= |err| < 15）：标准响应，弯道行驶 */
#define PID_KP_MID_ERR          35      /* 比例系数 3.5 */
#define PID_KI_MID_ERR          2       /* 积分系数 0.02 */
#define PID_KD_MID_ERR          60      /* 微分系数 6.0 */

/* 大误差状态（|err| >= 15）：快速纠偏，急转弯 */
#define PID_KP_LARGE_ERR        50      /* 比例系数 5.0 */
#define PID_KI_LARGE_ERR        3       /* 积分系数 0.03 */
#define PID_KD_LARGE_ERR        40      /* 微分系数 4.0 */

/* PID输出限幅 */
#define PID_OUTPUT_MAX          100     /* 最大输出 */
#define PID_OUTPUT_MIN          -100    /* 最小输出 */
#define PID_INTEGRAL_MAX        500     /* 积分限幅 */
#define PID_INTEGRAL_MIN        -500

/*******************************************************************************
 * 速度配置
 ******************************************************************************/
/* 基础速度设置（电机PWM值范围：-1000 ~ 1000，但这里用百分比） */
#define SPEED_STRAIGHT          25      /* 直行速度（小误差时） */
#define SPEED_CURVE             18      /* 弯道速度（中等误差时） */
#define SPEED_SLOW              12      /* 慢速（大误差时） */
#define SPEED_RECOVERY          10      /* 脱线恢复速度 */

/* 速度平滑参数（防止电机突变） */
#define SPEED_MAX_CHANGE        3       /* 每周期最大速度变化量 */

/* 转向因子（差速转向强度） */
#define STEER_FACTOR            8       /* 转向系数 0.8 */

/*******************************************************************************
 * 脱线检测与恢复参数
 ******************************************************************************/
/* 脱线检测阈值 */
#define LOST_THRESHOLD          5       /* 连续多少次全白确认脱线 */
#define LOST_RECOVERY_TIME      800     /* 脱线恢复超时时间(ms) */
#define LOST_RECOVERY_SPEED_L   8       /* 恢复搜索左轮速度（减小摇摆幅度） */
#define LOST_RECOVERY_SPEED_R   -4      /* 恢复搜索右轮速度（减小摇摆幅度） */

/*******************************************************************************
 * 赛道标记检测参数
 ******************************************************************************/
/* 标记类型枚举 */
typedef enum {
    MARK_NONE = 0,          /* 无标记 */
    MARK_5X5,               /* 5x5cm标记点 */
    MARK_15X15,             /* 15x15cm标记区域 */
    MARK_40X40,             /* 40x40cm作业区域 */
    MARK_CROSS              /* 十字路口 */
} TrackMark_t;

/* 标记检测时间阈值(ms) - 根据车速调整 */
#define MARK_5X5_MIN_MS         20      /* 5x5标记最小持续时间 */
#define MARK_5X5_MAX_MS         80      /* 5x5标记最大持续时间 */
#define MARK_15X15_MIN_MS       100     /* 15x15标记最小持续时间 */
#define MARK_40X40_MIN_MS       300     /* 40x40区域最小持续时间 */
#define MARK_CROSS_MS           150     /* 十字路口持续时间 */

/*******************************************************************************
 * PID控制器结构体定义
 ******************************************************************************/
typedef struct {
    int16_t Kp;             /* 比例系数 */
    int16_t Ki;             /* 积分系数 */
    int16_t Kd;             /* 微分系数 */
    int16_t err;            /* 当前误差 */
    int16_t err_last;       /* 上次误差 */
    int32_t integral;       /* 积分累积 */
    int16_t output;         /* 输出值 */
} PID_TypeDef;

/*******************************************************************************
 * 巡线系统状态结构体
 ******************************************************************************/
typedef struct {
    uint8_t sensor_raw;             /* 原始传感器值（4位） */
    int8_t position;                /* 当前位置估计 */
    int16_t left_speed;             /* 左轮目标速度 */
    int16_t right_speed;            /* 右轮目标速度 */
    uint8_t is_lost;                /* 脱线标志 */
    uint8_t lost_counter;           /* 脱线计数器 */
    int8_t last_valid_dir;          /* 最后有效方向 */
    TrackMark_t last_mark;          /* 最后检测到的标记 */
} Tracking_State_t;

/*******************************************************************************
 * 函数声明
 ******************************************************************************/

/* 初始化函数 */
void tracking_init(void);

/* 核心巡线函数 */
void tracking_update(void);                          /* 更新巡线状态（主循环调用） */
int8_t tracking_get_position(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4);
int16_t tracking_pid_calc(int16_t setpoint, int16_t measured);
void tracking_calc_speed(int16_t pid_output, int16_t base_speed, int16_t *left_speed, int16_t *right_speed);
int16_t tracking_smooth_speed(int16_t target, int16_t current);

/* 脱线恢复 */
void tracking_handle_lost(void);
void tracking_recover_line(void);

/* 标记检测 */
TrackMark_t tracking_detect_mark(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4);
void tracking_handle_mark(TrackMark_t mark);

/* 调试接口 */
void tracking_get_state(Tracking_State_t *state);
void tracking_print_debug(void);
void tracking_run_tests(void);

/* 参数调整接口 */
void tracking_set_pid(int16_t kp, int16_t ki, int16_t kd);
void tracking_set_speed(uint8_t straight, uint8_t curve, uint8_t slow);

#endif /* __Z_TRACKING_H__ */
