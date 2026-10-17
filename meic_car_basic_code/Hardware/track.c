#include "track.h"

static float s_last_valid_error = 0.0f;

static void Track_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // 地址选择引脚: PB12 (AD0), PB13 (AD1), PB14 (AD2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);

    // 模拟输入引脚: PB0 (ADC1_Channel_8)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void Track_ADC_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 72MHz / 6 = 12MHz (不超过 14MHz)

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    // 校准 ADC1
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

void Track_Init(void)
{
    Track_GPIO_Init();
    Track_ADC_Init();
    s_last_valid_error = 0.0f;
}

static void Track_SelectChannel(uint8_t ch)
{
    if (ch & 0x01) GPIO_SetBits(GPIOB, GPIO_Pin_12);
    else           GPIO_ResetBits(GPIOB, GPIO_Pin_12);

    if (ch & 0x02) GPIO_SetBits(GPIOB, GPIO_Pin_13);
    else           GPIO_ResetBits(GPIOB, GPIO_Pin_13);

    if (ch & 0x04) GPIO_SetBits(GPIOB, GPIO_Pin_14);
    else           GPIO_ResetBits(GPIOB, GPIO_Pin_14);

    // 留出 74HC4051 多路复用芯片通道切换建立时间 (~5us)
    for (volatile int i = 0; i < 60; i++);
}

static uint16_t Track_ReadADC_Channel8(void)
{
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}

/*
 * 扫描全部 8 个通道，计算循迹偏差与特征标识
 * 探头物理位置分布（自车头左到右）：
 *   Ch0: -3.5 | Ch1: -2.5 | Ch2: -1.5 | Ch3: -0.5 | Ch4: +0.5 | Ch5: +1.5 | Ch6: +2.5 | Ch7: +3.5
 */
static const float s_weights[TRACK_NUM_CHANNELS] = {
    -3.5f, -2.5f, -1.5f, -0.5f, +0.5f, +1.5f, +2.5f, +3.5f
};

void Track_ScanAll(Track_Result_t *result, uint16_t threshold)
{
    float weighted_sum = 0.0f;
    uint8_t count = 0;
    uint8_t left_cnt = 0;
    uint8_t right_cnt = 0;

    for (uint8_t i = 0; i < TRACK_NUM_CHANNELS; i++) {
        Track_SelectChannel(i);
        result->raw_adc[i] = Track_ReadADC_Channel8();

        // 坑点6：读数 < 阈值 判定为压黑线（8路红光模拟分时版）
        if (result->raw_adc[i] < threshold) {
            result->is_black[i] = 1;
            weighted_sum += s_weights[i];
            count++;
            if (i < 4) left_cnt++;
            else       right_cnt++;
        } else {
            result->is_black[i] = 0;
        }
    }

    result->black_count = count;
    result->left_black_count = left_cnt;
    result->right_black_count = right_cnt;

    if (count == 0) {
        result->all_white = 1;
        // 丢线保护：沿用上一次有效偏差方向，防止局部变量未初始化或丢线乱转
        result->line_pos_error = s_last_valid_error;
    } else {
        result->all_white = 0;
        result->line_pos_error = weighted_sum / (float)count;
        s_last_valid_error = result->line_pos_error;
    }

    // 特征判定：
    // 1. 横线/停止线/十字路口：>= 5 个探头同时压黑，且左右两侧均有压黑
    if (count >= 5 && left_cnt >= 2 && right_cnt >= 2) {
        result->is_cross_or_stop = 1;
    } else {
        result->is_cross_or_stop = 0;
    }

    // 2. 左直角弯：左侧探头压黑数量 >= 3（且最左端 Ch0 或 Ch1 压黑），同时最右端未压黑
    if (left_cnt >= 3 && (result->is_black[0] || result->is_black[1]) && !result->is_black[7]) {
        result->is_right_angle_left = 1;
    } else {
        result->is_right_angle_left = 0;
    }

    // 3. 右直角弯：右侧探头压黑数量 >= 3（且最右端 Ch6 或 Ch7 压黑），同时最左端未压黑
    if (right_cnt >= 3 && (result->is_black[6] || result->is_black[7]) && !result->is_black[0]) {
        result->is_right_angle_right = 1;
    } else {
        result->is_right_angle_right = 0;
    }
}
