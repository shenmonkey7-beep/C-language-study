#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f10x.h"

/*
 * ============================================================================
 * MEIC 智能寻迹小车 - 8路红光模拟灰度寻迹模块 (74HC4051 分时复用)
 * 硬件接线映射:
 *   - 多路选择地址引脚: AD0 = PB12, AD1 = PB13, AD2 = PB14
 *   - 模拟信号输入: OUT = PB0 (ADC1_Channel_8)
 * 判定口径（★ 实测重要）：
 *   - 读数 < TRACK_THRESHOLD 表示检测到黑线（黑色反光弱，ADC 值小）
 *   - 读数 >= TRACK_THRESHOLD 表示白底（白色反光强，ADC 值大）
 * ============================================================================
 */

#define TRACK_NUM_CHANNELS      8

// 默认灰度阈值，需在现场通过 RUN_MODE=3 (抄白底与黑线中间值) 覆盖微调
#define DEFAULT_TRACK_THRESHOLD 1800 

typedef struct {
    uint16_t raw_adc[TRACK_NUM_CHANNELS];   // 8 路原始 ADC 读数
    uint8_t  is_black[TRACK_NUM_CHANNELS];  // 8 路二值化压线状态 (1:压黑, 0:白底)
    uint8_t  black_count;                   // 当前压黑线探头总数
    uint8_t  left_black_count;              // 左侧4路(Ch0~Ch3)压黑数量
    uint8_t  right_black_count;             // 右侧4路(Ch4~Ch7)压黑数量
    float    line_pos_error;                // 寻迹中心偏差 (-3.5 ~ +3.5，0为居中，>0表示黑线偏右)
    uint8_t  is_cross_or_stop;              // 是否检测到横线/停止线/十字路口 (多路同时压黑)
    uint8_t  is_right_angle_left;           // 是否检测到左直角弯特征
    uint8_t  is_right_angle_right;          // 是否检测到右直角弯特征
    uint8_t  all_white;                     // 脱线状态（8路全部为白底）
} Track_Result_t;

void Track_Init(void);
void Track_ScanAll(Track_Result_t *result, uint16_t threshold);

#endif /* __TRACK_H */
