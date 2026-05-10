#ifndef __Z_SORTING_H__
#define __Z_SORTING_H__

#include "stm32f10x_conf.h"

/* 自动分拣模块头文件 */

/* 分拣部件类型 */
typedef enum {
    PART_RED = 0,
    PART_GREEN,
    PART_BLUE,
    PART_UNKNOWN
} PartType_t;

/* 函数声明 */
void sorting_start(PartType_t type);
void sorting_task(void);
uint8_t sorting_is_busy(void);
void sorting_stop(void);
void sorting_send_stats(void);
void sorting_color_calibrate(void);
void sorting_test_color_accuracy(void);
void sorting_test_arm_precision(void);
void sorting_run_self_test(void);

#endif /* __Z_SORTING_H__ */
