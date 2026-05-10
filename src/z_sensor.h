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


// ========== 新增：加权循迹算法定义 ==========

// 传感器状态编码宏
#define SENSOR_IDX(s1,s2,s3,s4) (((s1)<<3)|((s2)<<2)|((s3)<<1)|(s4))

// 位置估计特殊值
#define POS_ALL_WHITE    99   // 全白（脱线）
#define POS_ALL_BLACK   100   // 全黑（标记点）

/* ============================================================
 * 快速调节参数区 - 修改以下数值即可调整循迹性能
 * ============================================================ */

// ------------------- 速度参数 -------------------
// 说明：数值越大速度越快，但过大会导致脱线
// 建议范围：8~18
#define SPEED_STRAIGHT     14   // 直道速度（默认14）
#define SPEED_CURVE        11   // 弯道速度（默认11）
#define SPEED_SLOW          8   // 慢速/纠偏（默认8）

// 速度变化限制（防止卡顿和震荡）
// 说明：每周期最大速度变化量，越大响应越快但越抖
// 建议范围：2~5
#define SPEED_MAX_CHANGE    2   // 速度最大变化率（默认2，减小以平滑速度）

// 转向灵敏度
// 说明：越大转向越激进，过小转向不足，过大易震荡
// 建议范围：3~6
#define STEER_FACTOR        3   // 转向系数（默认3，减小以减少摇摆）

// ------------------- PID参数 -------------------
// 说明：PID控制循迹精度
// Kp: 比例系数 - 越大纠偏越强，但过大会震荡
// Ki: 积分系数 - 消除静差，过大累积误差
// Kd: 微分系数 - 抑制震荡，过大对噪声敏感

// 小误差时（直道）- 追求平稳高速
#define PID_KP_SMALL_ERR   12   // 小误差Kp（默认12，降低减少震荡）
#define PID_KD_SMALL_ERR   40   // 小误差Kd（默认40，降低减少抖动）

// 中等误差时（弯道）- 标准响应
#define PID_KP_MID_ERR     20   // 中误差Kp（默认20，降低）
#define PID_KD_MID_ERR     35   // 中误差Kd（默认35，降低）

// 大误差时（急弯/脱线恢复）- 快速纠偏
#define PID_KP_LARGE_ERR   30   // 大误差Kp（默认30，降低）
#define PID_KD_LARGE_ERR   25   // 大误差Kd（默认25，降低）

// 积分系数（全局）
#define PID_KI             2    // 积分系数（默认2）

// PID积分限幅（防止累积过大）
#define PID_INTEGRAL_MAX  300   // 积分上限（默认300）

// PID输出限幅
#define PID_OUTPUT_MAX    100   // PID输出上限（默认100）

// ------------------- 脱线检测参数 -------------------
// 说明：连续多少次检测到全白才判定脱线
// 越大越不敏感（抗干扰），越小反应越快
// 注意：赛程开始时可能全白，需要足够大的值避免误判
#define LOST_THRESHOLD      20   // 脱线判定阈值（默认20次，约200-400ms）

// 脱线恢复搜索时间（毫秒）
#define LOST_RECOVERY_TIME  2000 // 恢复超时时间（默认2000ms）

// ------------------- 标记检测时间阈值（ms） -------------------
// 说明：根据小车速度和标记尺寸调整
// 速度越快，持续时间越短，阈值应调小

#define MARK_5X5_MIN_MS     10  // 5x5标记最小时间（默认10）
#define MARK_5X5_MAX_MS     40  // 5x5标记最大时间（默认40）
#define MARK_15X15_MIN_MS   50  // 15x15标记最小时间（默认50）
#define MARK_15X15_MAX_MS   150 // 15x15标记最大时间（默认150）
#define MARK_40X40_MIN_MS   200 // 40x40标记最小时间（默认200）
#define MARK_CROSS_MS       80  // 十字路口判定时间（默认80）

/* ============================================================
 * 快速调节参数区结束
 * ============================================================ */

// 赛道标记类型枚举
typedef enum {
    MARK_NONE = 0,
    MARK_5X5,
    MARK_15X15,
    MARK_40X40,
    MARK_CROSS,
    MARK_STOP
} TrackMark_t;

// PID控制器结构体
typedef struct {
    int16_t Kp, Ki, Kd;
    int16_t err, err_last;
    int32_t integral;
    int16_t output;
} PID_TypeDef;

// ========== 原有定义保留 ==========

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

// 新增函数声明
int8_t tracking_get_position(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4);
int16_t tracking_pid_calc(PID_TypeDef *pid, int16_t setpoint, int16_t measured);
void tracking_calc_speed(int16_t pid_output, int16_t base_speed, 
                         int16_t *left_speed, int16_t *right_speed);
int16_t tracking_smooth_speed(int16_t target, int16_t current);
TrackMark_t tracking_detect_mark(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4);
void tracking_handle_mark(TrackMark_t mark);
void tracking_recover_lost(void);

// 测试函数声明
void test_tracking_position_table(void);
void test_pid_controller(void);
void test_speed_mapping(void);
void test_speed_smoothing(void);
void run_all_tests(void);

// 超声波测距函数声明
int get_adc_csb_middle(void);

#endif
