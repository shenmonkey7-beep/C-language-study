#include "motor.h"

/*
 * 电机驱动初始化:
 * 1. 配置 GPIO:
 *    - PA0 (TIM2_CH1), PA3 (TIM2_CH4) 为复用推挽输出 (PWM)
 *    - PA1, PA2, PB1, PB8 为推挽输出 (方向控制)
 * 2. 配置 TIM2:
 *    - 预分频 PSC = 0 (72MHz 计数)
 *    - 重装载 ARR = 7199 (10kHz PWM 频率，无电机高频啸叫)
 */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 1. 方向控制 GPIO 配置
    // 左轮方向: PA1, PA2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 右轮方向: PB1, PB8 (防接错适配)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 初始制动/空闲
    GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2);
    GPIO_ResetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_8);

    // 2. PWM 引脚配置: PA0 (TIM2_CH1), PA3 (TIM2_CH4)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 定时器 TIM2 基础时基配置
    TIM_TimeBaseStructure.TIM_Period = PWM_MAX_LIMIT;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 4. PWM 模式输出配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

/*
 * 设置电机输出 (-PWM_MAX_LIMIT ~ +PWM_MAX_LIMIT)
 * 正数前进，负数后退，0滑行停止
 */
void Motor_SetSpeed(int16_t left_pwm, int16_t right_pwm)
{
    // 左轮限幅与方向控制
    if (left_pwm > PWM_MAX_LIMIT) left_pwm = PWM_MAX_LIMIT;
    if (left_pwm < -PWM_MAX_LIMIT) left_pwm = -PWM_MAX_LIMIT;

    if (left_pwm > 0) {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
        TIM_SetCompare1(TIM2, (uint16_t)left_pwm);
    } else if (left_pwm < 0) {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
        TIM_SetCompare1(TIM2, (uint16_t)(-left_pwm));
    } else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2);
        TIM_SetCompare1(TIM2, 0);
    }

    // 右轮限幅与方向控制 (BIN1=PB1, BIN2=PB8)
    if (right_pwm > PWM_MAX_LIMIT) right_pwm = PWM_MAX_LIMIT;
    if (right_pwm < -PWM_MAX_LIMIT) right_pwm = -PWM_MAX_LIMIT;

    if (right_pwm > 0) {
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        TIM_SetCompare4(TIM2, (uint16_t)right_pwm);
    } else if (right_pwm < 0) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
        GPIO_SetBits(GPIOB, GPIO_Pin_8);
        TIM_SetCompare4(TIM2, (uint16_t)(-right_pwm));
    } else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_8);
        TIM_SetCompare4(TIM2, 0);
    }
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}

void Motor_Brake(void)
{
    // AIN1=1, AIN2=1; BIN1=1, BIN2=1 刹车制动
    GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2);
    GPIO_SetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_8);
    TIM_SetCompare1(TIM2, 0);
    TIM_SetCompare4(TIM2, 0);
}
