#ifndef __Z_SORTING_H__
#define __Z_SORTING_H__

#include "stm32f10x_conf.h"
#include "z_color.h"

// ========== 分拣系统参数配置区 ==========

// 颜色识别阈值（根据实际环境校准）
#define COLOR_DETECT_THRESHOLD  100     // 颜色检测最小亮度阈值
#define COLOR_VERIFY_DELAY      800     // 颜色验证延时(ms)
#define COLOR_SAMPLE_TIMES      3       // 颜色采样次数（取众数）

// 抓取位置参数（机械臂坐标，单位mm）
// 根据实际机械臂安装位置调整
#define GRAB_X_DEFAULT          120.0f  // 默认抓取X坐标
#define GRAB_Y_DEFAULT          0.0f    // 默认抓取Y坐标
#define GRAB_Z_DOWN             30.0f   // 抓取下降高度
#define GRAB_Z_UP               80.0f   // 抓取抬起高度

// 放置位置参数（对应红/绿/蓝三个工位）
#define PLACE_RED_X             80.0f   // 红色放置区X
#define PLACE_RED_Y             -60.0f  // 红色放置区Y
#define PLACE_GRN_X             80.0f   // 绿色放置区X
#define PLACE_GRN_Y             0.0f    // 绿色放置区Y
#define PLACE_BLU_X             80.0f   // 蓝色放置区X
#define PLACE_BLU_Y             60.0f   // 蓝色放置区Y
#define PLACE_Z_HEIGHT          50.0f   // 放置高度

// 机械臂运动时间参数（ms）
#define ARM_MOVE_TIME_FAST      500     // 快速移动时间
#define ARM_MOVE_TIME_SLOW      800     // 慢速移动时间（抓取/放置时）
#define ARM_GRAB_WAIT_TIME      300     // 夹爪闭合等待时间
#define ARM_RELEASE_WAIT_TIME   200     // 夹爪释放等待时间

// 动作组编号（需提前通过上位机下载到W25Q64）
// 假设动作组编号：
// 10-15: 抓取动作序列
// 20-25: 放置到红色区域
// 30-35: 放置到绿色区域
// 40-45: 放置到蓝色区域
// 50: 报警动作
#define ACTION_GRAB_PREPARE     10      // 抓取准备姿态
#define ACTION_GRAB_DOWN        11      // 下降抓取
#define ACTION_GRAB_CLOSE       12      // 夹爪闭合
#define ACTION_GRAB_UP          13      // 抬起
#define ACTION_PLACE_RED        20      // 放置到红色区域
#define ACTION_PLACE_GRN        30      // 放置到绿色区域
#define ACTION_PLACE_BLU        40      // 放置到蓝色区域
#define ACTION_ALARM            50      // 报警动作

// 超声波测距参数
#define GRAB_DISTANCE_THRESHOLD 12      // 抓取距离阈值(cm)
#define GRAB_DISTANCE_VERIFY    10      // 抓取验证距离(cm)

// 统计与异常参数
#define MAX_RETRY_TIMES         3       // 最大重试次数
#define ALARM_BEEP_TIMES        5       // 报警蜂鸣次数
#define ALARM_BEEP_DURATION     200     // 报警蜂鸣时长(ms)

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

// 抓取统计结构体
typedef struct {
    uint16_t total_attempts;        // 总尝试次数
    uint16_t success_count;         // 成功次数
    uint16_t fail_count;            // 失败次数
    uint16_t color_error_count;     // 颜色识别错误次数
    uint16_t retry_count;           // 重试次数
    PartColor_t last_part_color;    // 上一个零件颜色
    uint8_t consecutive_errors;     // 连续错误次数
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
    SortingStats_t stats;           // 统计信息
} SortingCtrl_t;

// ========== 函数声明 ==========

// 初始化函数
void sorting_init(void);                    // 初始化分拣系统
void sorting_load_stations(void);           // 加载工位配置

// 主控制函数
void sorting_task(void);                    // 分拣主任务（放入主循环）
void sorting_start(PartColor_t target);     // 开始分拣指定颜色
void sorting_stop(void);                    // 停止分拣

// 状态处理函数
void sorting_state_idle(void);              // 空闲状态
void sorting_state_move_to_target(void);    // 移动到目标
void sorting_state_detect_color(void);      // 检测颜色
void sorting_state_verify_color(void);      // 验证颜色
void sorting_state_grab(void);              // 抓取
void sorting_state_verify_grab(void);       // 验证抓取
void sorting_state_move_to_station(void);   // 移动到工位
void sorting_state_place(void);             // 放置
void sorting_state_verify_place(void);      // 验证放置
void sorting_state_return_home(void);       // 返回原点
void sorting_state_error(void);             // 错误处理

// 颜色识别函数
PartColor_t sorting_detect_color(void);                     // 检测颜色
PartColor_t sorting_verify_color(PartColor_t first_detect); // 验证颜色
uint8_t sorting_is_color_match(PartColor_t detect, PartColor_t target); // 颜色匹配检查

// 机械臂控制函数
uint8_t sorting_arm_move_to(float x, float y, float z, uint16_t time); // 移动到坐标
uint8_t sorting_arm_grab(void);             // 执行抓取动作序列
uint8_t sorting_arm_place(PartColor_t color); // 执行放置动作序列
uint8_t sorting_arm_return_home(void);      // 返回初始位置

// 超声波验证函数
uint8_t sorting_verify_distance(uint8_t expected_distance); // 验证距离

// 统计与报警函数
void sorting_update_stats(uint8_t success, PartColor_t color); // 更新统计
void sorting_send_stats(void);              // 发送统计信息到串口
void sorting_alarm(uint8_t error_code);     // 报警
void sorting_log_error(uint8_t error_code, PartColor_t color); // 记录错误

// 工具函数
const char* sorting_color_to_str(PartColor_t color); // 颜色转字符串
uint8_t sorting_is_busy(void);              // 检查是否忙
SortingState_t sorting_get_state(void);     // 获取当前状态

#endif
