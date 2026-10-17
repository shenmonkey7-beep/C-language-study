#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

/*
 * ============================================================================
 * MEIC 智能寻迹小车 - 闭环 PID 控制器模块 (转向外环 PD + 速度内环 PI)
 * 避坑重点：
 *   1. 严禁在 Speed_Loop() 中写固定常量 (如 return 3000)，必须是真正闭环
 *   2. 严格按照 10ms 控制周期整定增益参数 (CTRL_PERIOD_MS = 10)
 *   3. 支持右轮比例配平因子 RIGHT_TRIM，消除双电机机械/磨损不对称
 * ============================================================================
 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float err;
    float last_err;
    float prev_err;
    float integral;
    float max_integral;
    float max_output;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max_i, float max_out);
void PID_Reset(PID_Controller_t *pid);
void PID_ResetAll(void);
float PID_PositionCompute(PID_Controller_t *pid, float target, float current);

// 速度双环与转向闭环计算
int16_t Speed_Loop_Left(int16_t target_pulse, int16_t current_pulse);
int16_t Speed_Loop_Right(int16_t target_pulse, int16_t current_pulse, float right_trim);
float Steering_Loop(float line_error);

#endif /* __PID_H */
