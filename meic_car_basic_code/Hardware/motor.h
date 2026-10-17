#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

/*
 * ============================================================================
 * MEIC 智能寻迹小车 - 电机驱动模块 (TB6612 双路驱动板)
 * 硬件接线映射（适配实际主控板引出引脚，严禁使用 PA4/PA5）:
 *   - 左轮电机 (Motor A):
 *       PWMA: PA0 (TIM2_CH1)
 *       AIN1: PA1
 *       AIN2: PA2
 *   - 右轮电机 (Motor B):
 *       PWMB: PA3 (TIM2_CH4)
 *       BIN1: PB1  (★ 关键适配点：实测板未引出 PA4/PA5)
 *       BIN2: PB8  (★ 关键适配点)
 *   - STBY: 接 3V3（待机使能）
 * ============================================================================
 */

#define PWM_MAX_LIMIT   7199    // TIM2 重装载值 ARR (72MHz / 7200 = 10kHz PWM)
#define PWM_DEADZONE    200     // 克服机械静摩擦死区

void Motor_Init(void);
void Motor_SetSpeed(int16_t left_pwm, int16_t right_pwm);
void Motor_Stop(void);
void Motor_Brake(void);

#endif /* __MOTOR_H */
