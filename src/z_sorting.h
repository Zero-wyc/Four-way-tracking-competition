#ifndef __Z_SORTING_H__
#define __Z_SORTING_H__

#include "stm32f10x_conf.h"
#include "z_color.h"

// ============================================================
// 自动化零件分拣系统 - 参数配置区
// 版本: V2.0
// 更新: 2026-05-09
// 说明: 根据赛道图片(200x150cm)和性能要求优化
// ============================================================

// ========== 性能指标参数 ==========
// 颜色识别准确率目标: >=95%
// 响应时间目标: <=500ms
// 定位精度目标: <=±2mm
// 平均处理时间目标: <=10秒/件

// ========== 颜色识别优化参数 ==========
// 颜色检测最小亮度阈值(环境光补偿)
#define COLOR_DETECT_THRESHOLD  80      // 降低阈值提高灵敏度
#define COLOR_VERIFY_DELAY      300     // 缩短验证延时(原800ms)
#define COLOR_SAMPLE_TIMES      5       // 增加采样次数到5次(原3次)
#define COLOR_SAMPLE_INTERVAL   50      // 采样间隔ms

// 颜色识别置信度阈值(用于提高准确率)
#define COLOR_CONFIDENCE_THRESHOLD  60  // 最小置信度(%)
#define COLOR_RATIO_THRESHOLD       1.3f // 主导颜色/次颜色比值

// 动态白平衡参数
#define COLOR_WB_ENABLE         1       // 启用动态白平衡
#define COLOR_WB_SAMPLE_NUM     10      // 白平衡采样次数

// ========== 机械臂精度优化参数 ==========
// 抓取位置参数(单位mm，根据实际标定)
#define GRAB_X_DEFAULT          120.0f  // 默认抓取X坐标
#define GRAB_Y_DEFAULT          0.0f    // 默认抓取Y坐标
#define GRAB_Z_DOWN             25.0f   // 抓取下降高度(降低5mm更精准)
#define GRAB_Z_UP               80.0f   // 抓取抬起高度
#define GRAB_Z_APPROACH         45.0f   // 接近高度(新增，分步下降)

// 放置位置参数(对应红/绿/蓝三个工位，根据赛道中心区域调整)
// 赛道中心十字区域放置三个工位
#define PLACE_RED_X             100.0f  // 红色放置区X
#define PLACE_RED_Y             -50.0f  // 红色放置区Y(左侧)
#define PLACE_GRN_X             100.0f  // 绿色放置区X
#define PLACE_GRN_Y             0.0f    // 绿色放置区Y(中间)
#define PLACE_BLU_X             100.0f  // 蓝色放置区X
#define PLACE_BLU_Y             50.0f   // 蓝色放置区Y(右侧)
#define PLACE_Z_HEIGHT          40.0f   // 放置高度

// 机械臂运动时间参数(ms) - 优化为<=10秒/件
#define ARM_MOVE_TIME_FAST      300     // 快速移动(原500ms)
#define ARM_MOVE_TIME_SLOW      500     // 慢速移动(原800ms)
#define ARM_MOVE_TIME_APPROACH  400     // 接近移动
#define ARM_GRAB_WAIT_TIME      200     // 夹爪闭合(原300ms)
#define ARM_RELEASE_WAIT_TIME   150     // 夹爪释放(原200ms)
#define ARM_SETTLE_TIME         100     // 稳定等待时间

// 定位精度补偿参数
#define ARM_POS_COMPENSATE_X    0.0f    // X方向补偿
#define ARM_POS_COMPENSATE_Y    0.0f    // Y方向补偿
#define ARM_POS_COMPENSATE_Z    0.0f    // Z方向补偿

// ========== 动作组编号 ==========
// 需提前通过上位机下载到W25Q64
#define ACTION_GRAB_PREPARE     10      // 抓取准备姿态
#define ACTION_GRAB_APPROACH    11      // 接近零件
#define ACTION_GRAB_DOWN        12      // 下降抓取
#define ACTION_GRAB_CLOSE       13      // 夹爪闭合
#define ACTION_GRAB_UP          14      // 抬起
#define ACTION_PLACE_RED        20      // 放置到红色区域
#define ACTION_PLACE_GRN        30      // 放置到绿色区域
#define ACTION_PLACE_BLU        40      // 放置到蓝色区域
#define ACTION_ALARM            50      // 报警动作

// ========== 超声波测距参数 ==========
#define GRAB_DISTANCE_THRESHOLD 15      // 抓取距离阈值(cm)
#define GRAB_DISTANCE_VERIFY    12      // 抓取验证距离(cm)
#define GRAB_DISTANCE_APPROACH  8       // 接近距离(cm)

// ========== 统计与异常参数 ==========
#define MAX_RETRY_TIMES         3       // 最大重试次数
#define ALARM_BEEP_TIMES        3       // 报警蜂鸣次数(优化)
#define ALARM_BEEP_DURATION     150     // 报警蜂鸣时长(ms)

// 错误码定义
#define ERR_NONE            0
#define ERR_COLOR_MISMATCH  1
#define ERR_GRAB_FAILED     2
#define ERR_PLACE_FAILED    3
#define ERR_NO_PART         4
#define ERR_ARM_TIMEOUT     5
#define ERR_LOW_CONFIDENCE  6
#define ERR_VERIFY_FAIL     7

// ========== 枚举与结构体定义 ==========

// 零件颜色类型
typedef enum {
    PART_NONE = 0,      // 未识别
    PART_RED,           // 红色零件
    PART_GREEN,         // 绿色零件
    PART_BLUE,          // 蓝色零件
    PART_UNKNOWN        // 未知颜色
} PartColor_t;

// 分拣状态机状态
typedef enum {
    SORT_IDLE = 0,              // 空闲/等待
    SORT_MOVE_TO_TARGET,        // 移动到目标位置
    SORT_DETECT_COLOR,          // 检测颜色
    SORT_VERIFY_COLOR,          // 验证颜色
    SORT_APPROACH_PART,         // 接近零件(新增)
    SORT_GRAB_PART,             // 抓取零件
    SORT_VERIFY_GRAB,           // 验证抓取成功
    SORT_MOVE_TO_STATION,       // 移动到工位
    SORT_PLACE_PART,            // 放置零件
    SORT_VERIFY_PLACE,          // 验证放置成功
    SORT_RETURN_HOME,           // 返回初始位置
    SORT_ERROR_HANDLE,          // 错误处理
    SORT_COMPLETE               // 完成
} SortingState_t;

// 工位位置结构体
typedef struct {
    float x;
    float y;
    float z;
    uint8_t action_group;   // 对应动作组编号
} Station_t;

// 颜色校准数据
typedef struct {
    uint16_t r_base;
    uint16_t g_base;
    uint16_t b_base;
    uint16_t c_base;
    uint8_t is_calibrated;
} ColorCal_t;

// 抓取统计结构体
typedef struct {
    uint16_t total_attempts;        // 总尝试次数
    uint16_t success_count;         // 成功次数
    uint16_t fail_count;            // 失败次数
    uint16_t color_error_count;     // 颜色识别错误次数
    uint16_t retry_count;           // 重试次数
    uint16_t timeout_count;         // 超时次数
    PartColor_t last_part_color;    // 上一个零件颜色
    uint8_t consecutive_errors;     // 连续错误次数
    uint32_t avg_process_time;      // 平均处理时间(ms)
    uint32_t total_process_time;    // 总处理时间(ms)
} SortingStats_t;

// 分拣系统控制结构体
typedef struct {
    SortingState_t state;           // 当前状态
    SortingState_t last_state;      // 上一个状态
    PartColor_t target_color;       // 目标颜色
    PartColor_t detected_color;     // 检测到的颜色
    uint8_t retry_counter;          // 当前重试计数
    uint8_t is_busy;                // 忙标志
    uint32_t state_timer;           // 状态计时器
    uint32_t process_start_time;    // 流程开始时间
    SortingStats_t stats;           // 统计信息
    ColorCal_t color_cal;           // 颜色校准数据
} SortingCtrl_t;

// ========== 函数声明 ==========

// 初始化函数
void sorting_init(void);
void sorting_load_stations(void);
void sorting_color_calibrate(void);

// 主控制函数
void sorting_task(void);
void sorting_start(PartColor_t target);
void sorting_stop(void);

// 状态处理函数
void sorting_state_idle(void);
void sorting_state_move_to_target(void);
void sorting_state_detect_color(void);
void sorting_state_verify_color(void);
void sorting_state_approach(void);
void sorting_state_grab(void);
void sorting_state_verify_grab(void);
void sorting_state_move_to_station(void);
void sorting_state_place(void);
void sorting_state_verify_place(void);
void sorting_state_return_home(void);
void sorting_state_error(void);

// 颜色识别函数(优化版)
PartColor_t sorting_detect_color(void);
PartColor_t sorting_detect_color_advanced(COLOR_RGBC *rgbc_out);
uint8_t sorting_verify_color_confidence(PartColor_t *result, uint8_t *confidence);
uint8_t sorting_is_color_match(PartColor_t detect, PartColor_t target);
void sorting_apply_white_balance(COLOR_RGBC *rgbc);

// 机械臂控制函数(优化版)
uint8_t sorting_arm_move_to(float x, float y, float z, uint16_t time);
uint8_t sorting_arm_grab(void);
uint8_t sorting_arm_place(PartColor_t color);
uint8_t sorting_arm_return_home(void);
uint8_t sorting_arm_approach(float x, float y, float z_approach, uint16_t time);

// 超声波验证函数
uint8_t sorting_verify_distance(uint8_t expected_distance);
uint8_t sorting_verify_grab_by_distance(void);

// 统计与报警函数
void sorting_update_stats(uint8_t success, PartColor_t color);
void sorting_send_stats(void);
void sorting_alarm(uint8_t error_code);
void sorting_log_error(uint8_t error_code, PartColor_t color);
void sorting_record_process_time(void);

// 工具函数
const char* sorting_color_to_str(PartColor_t color);
uint8_t sorting_is_busy(void);
SortingState_t sorting_get_state(void);

// 测试函数
void sorting_test_color_accuracy(void);
void sorting_test_arm_precision(void);
void sorting_run_self_test(void);

#endif
