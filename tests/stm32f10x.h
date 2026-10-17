/* Test-only register model. Never put this directory on the firmware include path. */
#ifndef MEIC_TEST_STM32_H
#define MEIC_TEST_STM32_H
#include <stdint.h>
#include <string.h>
typedef struct { uint16_t count, ccr1, ccr4, bits, input; } MockPeripheral;
static MockPeripheral mock_a, mock_b, mock_t1, mock_t2, mock_t3, mock_t4, mock_adc;
#define GPIOA (&mock_a)
#define GPIOB (&mock_b)
#define TIM1 (&mock_t1)
#define TIM2 (&mock_t2)
#define TIM3 (&mock_t3)
#define TIM4 (&mock_t4)
#define ADC1 (&mock_adc)
#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define Bit_RESET 0
#define TIM1_UP_IRQn 25
static uint16_t mock_adc_values[8];
static unsigned mock_adc_index;
static int mock_adc_failure;
typedef struct {
    uint32_t ADC_ContinuousConvMode;
    uint32_t ADC_DataAlign;
    uint32_t ADC_ExternalTrigConv;
    uint32_t ADC_Mode;
    uint32_t ADC_NbrOfChannel;
    uint32_t ADC_ScanConvMode;
    uint32_t GPIO_Mode;
    uint32_t GPIO_Pin;
    uint32_t GPIO_Speed;
    uint32_t NVIC_IRQChannel;
    uint32_t NVIC_IRQChannelCmd;
    uint32_t NVIC_IRQChannelPreemptionPriority;
    uint32_t NVIC_IRQChannelSubPriority;
    uint32_t TIM_Channel;
    uint32_t TIM_ClockDivision;
    uint32_t TIM_CounterMode;
    uint32_t TIM_ICFilter;
    uint32_t TIM_OCMode;
    uint32_t TIM_OCPolarity;
    uint32_t TIM_OutputState;
    uint32_t TIM_Period;
    uint32_t TIM_Prescaler;
    uint32_t TIM_Pulse;
    uint32_t TIM_RepetitionCounter;
} MockInit;
typedef MockInit ADC_InitTypeDef;
typedef MockInit GPIO_InitTypeDef;
typedef MockInit NVIC_InitTypeDef;
typedef MockInit TIM_ICInitTypeDef;
typedef MockInit TIM_OCInitTypeDef;
typedef MockInit TIM_TimeBaseInitTypeDef;
#define ADC_Channel_8 1
#define ADC_DataAlign_Right 2
#define ADC_ExternalTrigConv_None 3
#define ADC_FLAG_EOC 4
#define ADC_Mode_Independent 5
#define ADC_SampleTime_55Cycles5 6
#define GPIO_Mode_AF_PP 7
#define GPIO_Mode_AIN 8
#define GPIO_Mode_IN_FLOATING 9
#define GPIO_Mode_IPU 10
#define GPIO_Mode_Out_PP 11
#define GPIO_Pin_0 1
#define GPIO_Pin_1 2
#define GPIO_Pin_11 2048
#define GPIO_Pin_12 4096
#define GPIO_Pin_13 8192
#define GPIO_Pin_14 16384
#define GPIO_Pin_2 4
#define GPIO_Pin_3 8
#define GPIO_Pin_6 64
#define GPIO_Pin_7 128
#define GPIO_Pin_8 256
#define GPIO_Speed_50MHz 23
#define NVIC_PriorityGroup_2 24
#define RCC_APB1Periph_TIM2 25
#define RCC_APB1Periph_TIM3 26
#define RCC_APB1Periph_TIM4 27
#define RCC_APB2Periph_ADC1 28
#define RCC_APB2Periph_GPIOA 29
#define RCC_APB2Periph_GPIOB 30
#define RCC_APB2Periph_TIM1 31
#define RCC_PCLK2_Div6 32
#define TIM_CKD_DIV1 33
#define TIM_Channel_1 34
#define TIM_Channel_2 35
#define TIM_CounterMode_Up 36
#define TIM_EncoderMode_TI12 37
#define TIM_FLAG_Update 38
#define TIM_ICPolarity_Rising 39
#define TIM_IT_Update 40
#define TIM_OCMode_PWM1 41
#define TIM_OCPolarity_High 42
#define TIM_OCPreload_Enable 43
#define TIM_OutputState_Enable 44
static inline void mock_ignore(int unused, ...) { (void)unused; }
#define ADC_ClearFlag(...) mock_ignore(0, __VA_ARGS__)
#define ADC_Cmd(...) mock_ignore(0, __VA_ARGS__)
#define ADC_DeInit(...) mock_ignore(0, __VA_ARGS__)
#define ADC_Init(...) mock_ignore(0, __VA_ARGS__)
#define ADC_RegularChannelConfig(...) mock_ignore(0, __VA_ARGS__)
#define ADC_ResetCalibration(...) mock_ignore(0, __VA_ARGS__)
#define ADC_SoftwareStartConvCmd(...) mock_ignore(0, __VA_ARGS__)
#define ADC_StartCalibration(...) mock_ignore(0, __VA_ARGS__)
#define GPIO_Init(...) mock_ignore(0, __VA_ARGS__)
#define NVIC_Init(...) mock_ignore(0, __VA_ARGS__)
#define NVIC_PriorityGroupConfig(...) mock_ignore(0, __VA_ARGS__)
#define RCC_ADCCLKConfig(...) mock_ignore(0, __VA_ARGS__)
#define RCC_APB1PeriphClockCmd(...) mock_ignore(0, __VA_ARGS__)
#define RCC_APB2PeriphClockCmd(...) mock_ignore(0, __VA_ARGS__)
#define TIM_ARRPreloadConfig(...) mock_ignore(0, __VA_ARGS__)
#define TIM_ClearFlag(...) mock_ignore(0, __VA_ARGS__)
#define TIM_ClearITPendingBit(...) mock_ignore(0, __VA_ARGS__)
#define TIM_Cmd(...) mock_ignore(0, __VA_ARGS__)
#define TIM_EncoderInterfaceConfig(...) mock_ignore(0, __VA_ARGS__)
#define TIM_ICInit(...) mock_ignore(0, __VA_ARGS__)
#define TIM_ITConfig(...) mock_ignore(0, __VA_ARGS__)
#define TIM_OC1Init(...) mock_ignore(0, __VA_ARGS__)
#define TIM_OC1PreloadConfig(...) mock_ignore(0, __VA_ARGS__)
#define TIM_OC4Init(...) mock_ignore(0, __VA_ARGS__)
#define TIM_OC4PreloadConfig(...) mock_ignore(0, __VA_ARGS__)
#define TIM_TimeBaseInit(...) mock_ignore(0, __VA_ARGS__)
#define GPIO_SetBits(p,n) ((p)->bits |= (n))
#define GPIO_ResetBits(p,n) ((p)->bits &= (uint16_t)~(n))
#define GPIO_ReadInputDataBit(p,n) (!!((p)->input & (n)))
#define TIM_GetCounter(p) ((p)->count)
#define TIM_SetCounter(p,n) ((p)->count = (uint16_t)(n))
#define TIM_SetCompare1(p,n) ((p)->ccr1 = (uint16_t)(n))
#define TIM_SetCompare4(p,n) ((p)->ccr4 = (uint16_t)(n))
#define TIM_TimeBaseStructInit(p) memset((p),0,sizeof(*(p)))
#define TIM_OCStructInit(p) memset((p),0,sizeof(*(p)))
#define TIM_ICStructInit(p) memset((p),0,sizeof(*(p)))
#define TIM_GetITStatus(p,n) (0)
#define ADC_GetResetCalibrationStatus(p) (mock_adc_failure)
#define ADC_GetCalibrationStatus(p) (mock_adc_failure)
#define ADC_GetFlagStatus(p,n) (!mock_adc_failure)
#define ADC_GetConversionValue(p) (mock_adc_values[mock_adc_index++ % 8])
#endif
