#include "encoder.h"

volatile int32_t g_sumL = 0;
volatile int32_t g_sumR = 0;
static uint16_t s_last_l = 0;
static uint16_t s_last_r = 0;

/*
 * 初始化 TIM3 (左轮) 和 TIM4 (右轮) 的正交编码器模式
 */
void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    // 1. 左编码器引脚: PA6, PA7 (TIM3_CH1, TIM3_CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 2. 右编码器引脚: PB6, PB7 (TIM4_CH1, TIM4_CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. TIM3 基础时基配置 (最大计数 65535)
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 4. TIM3 编码器模式: TI1 和 TI2 均计数 (4倍频)
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6; // 输入数字滤波，抑制毛刺
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);

    // 5. TIM4 基础时基配置
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // 6. TIM4 编码器模式
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);

    g_sumL = 0;
    g_sumR = 0;
    s_last_l = 0;
    s_last_r = 0;
}

/*
 * 读取左轮当前周期的脉冲增量，并累加到全局总里程 g_sumL
 */
int16_t Encoder_GetLeftSpeed(void)
{
    uint16_t now = (uint16_t)TIM_GetCounter(TIM3);
    int16_t count = (int16_t)(uint16_t)(now - s_last_l);
    s_last_l = now;
    g_sumL += count;
    return count;
}

/*
 * 读取右轮当前周期的脉冲增量，并累加到全局总里程 g_sumR
 * 注意：由于两轮电机对称反向安装，若右轮转动方向与左轮相反，需在此取反保持同向
 */
int16_t Encoder_GetRightSpeed(void)
{
    // 对称安装反相补偿
    uint16_t now = (uint16_t)TIM_GetCounter(TIM4);
    int16_t count = (int16_t)(uint16_t)(s_last_r - now);
    s_last_r = now;
    g_sumR += count;
    return count;
}

void Encoder_ResetDistance(void)
{
    g_sumL = 0;
    g_sumR = 0;
}

float Encoder_GetLeftDistanceCm(float pulse_per_cm)
{
    if (pulse_per_cm <= 0.01f) return 0.0f;
    return (float)g_sumL / pulse_per_cm;
}

float Encoder_GetRightDistanceCm(float pulse_per_cm)
{
    if (pulse_per_cm <= 0.01f) return 0.0f;
    return (float)g_sumR / pulse_per_cm;
}

float Encoder_GetAverageDistanceCm(float pulse_per_cm)
{
    if (pulse_per_cm <= 0.01f) return 0.0f;
    return (((float)g_sumL + (float)g_sumR) / 2.0f) / pulse_per_cm;
}
