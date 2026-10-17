#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/*
 * ============================================================================
 * MEIC 智能寻迹小车 - 编码器测速与里程统计模块
 * 硬件接线映射:
 *   - 左轮编码器 (TIM3): PA6 (A相), PA7 (B相)
 *   - 右轮编码器 (TIM4): PB6 (A相), PB7 (B相)
 * 4 倍频编码器接口模式，连续计数并用16位模差获取增量，避免读后清零丢脉冲。
 * ============================================================================
 */

// 默认脉冲当量（推车 100cm 后实测 g_sumL / 100 得到）
// MG513XP28 减速电机 1:28，霍尔 13 线，4 倍频 = 13 * 4 * 28 = 1456 脉冲/圈
// 轮径 65mm 对应周长约 20.42cm -> 理论脉冲当量约 71.3 脉冲/cm
#define DEFAULT_PULSE_PER_CM   71.3f

void Encoder_Init(void);
int16_t Encoder_GetLeftSpeed(void);
int16_t Encoder_GetRightSpeed(void);

void Encoder_ResetDistance(void);
float Encoder_GetLeftDistanceCm(float pulse_per_cm);
float Encoder_GetRightDistanceCm(float pulse_per_cm);
float Encoder_GetAverageDistanceCm(float pulse_per_cm);

extern volatile int32_t g_sumL;
extern volatile int32_t g_sumR;

#endif /* __ENCODER_H */
