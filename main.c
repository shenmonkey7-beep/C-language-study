/*
 * ============================================================================
 * MEIC 智能寻迹小车 · 招新题核心控制系统 (基础任务 + 任务二泊车 + 标定系统)
 * 主控：STM32F103C8T6 (C03C 核心板)
 * 硬件资源映射：
 *   - TB6612 电机驱动: PWMA(PA0), AIN1(PA1), AIN2(PA2), PWMB(PA3), BIN1(PB1), BIN2(PB8)
 *   - MG513XP28 编码器: 左(PA6/PA7, TIM3), 右(PB6/PB7, TIM4)
 *   - 74HC4051 8路模拟灰度: AD0(PB12), AD1(PB13), AD2(PB14), OUT(PB0, ADC1_IN8)
 *   - 板载按键: KEY1(PA11), KEY2(PA12) 或板载按键，用于运行时免烧录切换任务
 * 
 * 运行模式设定 (RUN_MODE):
 *   10: 基础任务全流程 (1m×1m正方形循迹 + 4直角弯平稳通过 + 定点停车不压线)
 *    0: 直行定速调试
 *    1: 纯巡线平衡测试
 *    2: 直角弯转弯参数调试
 *    3: 灰度传感器标定模式 (Keil Watch 查看 g_dbg[0..7])
 *    4: 电机驱动与接线自检 (交替正反转)
 *    6: 自动配平模式 (计算双轮机械差异，生成 g_trim)
 *   20: 任务二 - 倒车入库 (定距直行 -> 原地自转90度 -> 倒车入库 -> 停车入位)
 *   21: 任务二 - 侧方停车 (斜向入库 -> 回正 -> 停车入位)
 * ============================================================================
 */

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

#include "motor.h"
#include "encoder.h"
#include "track.h"
#include "pid.h"

// --------------------------- 核心参数配置 ---------------------------
#define RUN_MODE                10      // 默认基础任务全流程
#define CTRL_PERIOD_MS          10      // 控制周期严格保持 10ms (坑点4)
#define TEST_RUN0_STOP_CM       0.0f    // 定距停车强制关闭 (0=不限距离，坑点2)
#define DEFAULT_SPEED_CMPS      12.0f   // 验收推荐巡线线速度: 10~15 cm/s (稳且不易脱线)

// --------------------------- 全局调试变量 (Keil Watch 可直接观测) ---------------------------
volatile uint16_t g_dbg[8];             // 8路灰度原始读数
volatile uint8_t  g_dbg_bin[8];         // 8路二值化状态 (1=压黑, 0=白底)
volatile int16_t  g_encL = 0;           // 左轮瞬时脉冲/10ms
volatile int16_t  g_encR = 0;           // 右轮瞬时脉冲/10ms
volatile int16_t  g_pwmL = 0;           // 左轮输出 PWM
volatile int16_t  g_pwmR = 0;           // 右轮输出 PWM
volatile float    g_trim = 1.00f;       // 右轮配平因子 (坑点3)
volatile float    g_pulse_per_cm = 71.3f; // 脉冲当量 (冲车100cm实测校准)
volatile uint16_t g_track_threshold = DEFAULT_TRACK_THRESHOLD; // 灰度阈值
volatile uint8_t  g_task = RUN_MODE;    // 当前执行任务/模式 (可在线修改)
volatile uint8_t  g_corner_count = 0;   // 弯道/直角计数

// --------------------------- 状态机枚举 ---------------------------
typedef enum {
    STATE_IDLE = 0,
    STATE_START_ARMING,     // 起步武装：防起跑线误判直角 (坑点5)
    STATE_TRACKING,         // 闭环寻迹巡线
    STATE_CORNER_TURNING,   // 直角弯强制转向过渡
    STATE_APPROACHING_STOP, // 最后一圈减速准备停车
    STATE_STOPPED,          // 停车刹车制动
    // 泊车状态
    STATE_PARK_FORWARD,
    STATE_PARK_ROTATE,
    STATE_PARK_BACKWARD,
    STATE_PARK_FINISH
} CarState_t;

volatile CarState_t g_state = STATE_IDLE;

// 系统滴答定时器延时与时基
volatile uint32_t g_system_ticks_ms = 0;

void SysTick_Handler(void)
{
    g_system_ticks_ms++;
}

static void Delay_Ms(uint32_t ms)
{
    uint32_t start = g_system_ticks_ms;
    while ((g_system_ticks_ms - start) < ms);
}

// 板载按键初始化 (PA11, PA12 上拉输入)
static void Button_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 检查按键切换模式 (坑点7: 验收中途不能重烧程序，按键切换任务)
static void Check_Button(void)
{
    static uint32_t last_press = 0;
    if (g_system_ticks_ms - last_press < 250) return;

    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET) {
        last_press = g_system_ticks_ms;
        // KEY1: 在 基础巡线(10) -> 倒车入库(20) -> 侧方停车(21) 轮换
        if (g_task == 10) g_task = 20;
        else if (g_task == 20) g_task = 21;
        else g_task = 10;
        g_state = STATE_IDLE;
    }
}

// --------------------------- 任务一：基础任务全流程状态机 ---------------------------
static void Process_Basic_Task(Track_Result_t *track)
{
    static uint32_t arm_start_ms = 0;
    static uint32_t turn_start_ms = 0;
    static uint8_t  s_armed = 0;
    static int8_t   turn_direction = 0; // -1: 左, +1: 右

    // 将目标速度转换为每个控制周期(10ms)的目标脉冲数
    // 脉冲数 = 速度(cm/s) * pulse_per_cm * (10ms / 1000ms)
    float base_pulse = (DEFAULT_SPEED_CMPS * g_pulse_per_cm) * 0.01f;

    switch (g_state) {
        case STATE_IDLE:
            Encoder_ResetDistance();
            s_armed = 0;
            g_corner_count = 0;
            arm_start_ms = g_system_ticks_ms;
            g_state = STATE_START_ARMING;
            break;

        case STATE_START_ARMING:
            // 坑点5：起步时压在线上，必须等检测到正常的中间单线且持续 > 300ms 后才激活直角判定
            {
                float steer = Steering_Loop(track->line_pos_error);
                int16_t target_l = (int16_t)(base_pulse - steer);
                int16_t target_r = (int16_t)(base_pulse + steer);
                g_pwmL = Speed_Loop_Left(target_l, g_encL);
                g_pwmR = Speed_Loop_Right(target_r, g_encR, g_trim);
                Motor_SetSpeed(g_pwmL, g_pwmR);

                // 判断是否看到纯净单线 (居中且压线探头数 1~2 个)
                if (track->black_count >= 1 && track->black_count <= 2) {
                    if (g_system_ticks_ms - arm_start_ms > 300) {
                        s_armed = 1; // 武装完毕，进入正式巡线
                        g_state = STATE_TRACKING;
                    }
                }
            }
            break;

        case STATE_TRACKING:
            // 正常循迹
            {
                // 检测是否到达直角弯
                if (s_armed && (track->is_right_angle_left || track->is_right_angle_right || track->is_cross_or_stop)) {
                    g_corner_count++;
                    turn_start_ms = g_system_ticks_ms;
                    turn_direction = track->is_right_angle_left ? -1 : 1;
                    
                    // 1m×1m 正方形赛道共 4 个直角。转过 4 个直角回到原点后停车
                    if (g_corner_count >= 4) {
                        g_state = STATE_APPROACHING_STOP;
                        break;
                    }

                    g_state = STATE_CORNER_TURNING;
                    break;
                }

                // 正常外环转向控制
                float steer = Steering_Loop(track->line_pos_error);
                int16_t target_l = (int16_t)(base_pulse - steer);
                int16_t target_r = (int16_t)(base_pulse + steer);

                g_pwmL = Speed_Loop_Left(target_l, g_encL);
                g_pwmR = Speed_Loop_Right(target_r, g_encR, g_trim);
                Motor_SetSpeed(g_pwmL, g_pwmR);
            }
            break;

        case STATE_CORNER_TURNING:
            // 直角弯强制差速旋转过渡 (保证车身大角度迅速切回切线)
            {
                if (turn_direction == -1) {
                    // 左转直角：左轮反转或低速，右轮加速
                    g_pwmL = Speed_Loop_Left((int16_t)(-base_pulse * 0.4f), g_encL);
                    g_pwmR = Speed_Loop_Right((int16_t)(base_pulse * 1.5f), g_encR, g_trim);
                } else {
                    // 右转直角
                    g_pwmL = Speed_Loop_Left((int16_t)(base_pulse * 1.5f), g_encL);
                    g_pwmR = Speed_Loop_Right((int16_t)(-base_pulse * 0.4f), g_encR, g_trim);
                }
                Motor_SetSpeed(g_pwmL, g_pwmR);

                // 转动持续 350ms 且探头重新咬住中心线后退出
                if ((g_system_ticks_ms - turn_start_ms > 350) && 
                    (track->is_black[3] || track->is_black[4])) {
                    g_state = STATE_TRACKING;
                }
            }
            break;

        case STATE_APPROACHING_STOP:
            // 最后一圈返回起点停车区：减速进入并刹车，确保不压线 (学长验收第1条)
            {
                float slow_pulse = base_pulse * 0.4f;
                float steer = Steering_Loop(track->line_pos_error);
                int16_t target_l = (int16_t)(slow_pulse - steer);
                int16_t target_r = (int16_t)(slow_pulse + steer);

                g_pwmL = Speed_Loop_Left(target_l, g_encL);
                g_pwmR = Speed_Loop_Right(target_r, g_encR, g_trim);
                Motor_SetSpeed(g_pwmL, g_pwmR);

                // 探测到横向停止线 (多路探头压黑)
                if (track->is_cross_or_stop) {
                    Motor_Brake();
                    g_state = STATE_STOPPED;
                }
            }
            break;

        case STATE_STOPPED:
            Motor_Brake();
            break;
    }
}

// --------------------------- 任务二：泊车 (倒车入库与侧方停车) ---------------------------
static void Process_Parking_Task(uint8_t park_type)
{
    static uint32_t step_time = 0;
    static uint8_t park_step = 0;

    switch (park_step) {
        case 0: // 准备
            Encoder_ResetDistance();
            step_time = g_system_ticks_ms;
            park_step = 1;
            break;

        case 1: // 直行出库或调整
            if (park_type == 20) {
                // 倒车入库：先直行 40cm 到库前起始点
                if (Encoder_GetAverageDistanceCm(g_pulse_per_cm) < 40.0f) {
                    g_pwmL = Speed_Loop_Left(10, g_encL);
                    g_pwmR = Speed_Loop_Right(10, g_encR, g_trim);
                    Motor_SetSpeed(g_pwmL, g_pwmR);
                } else {
                    Motor_Brake();
                    Encoder_ResetDistance();
                    step_time = g_system_ticks_ms;
                    park_step = 2;
                }
            } else {
                // 侧方停车
                if (Encoder_GetAverageDistanceCm(g_pulse_per_cm) < 30.0f) {
                    g_pwmL = Speed_Loop_Left(10, g_encL);
                    g_pwmR = Speed_Loop_Right(10, g_encR, g_trim);
                    Motor_SetSpeed(g_pwmL, g_pwmR);
                } else {
                    Motor_Brake();
                    Encoder_ResetDistance();
                    step_time = g_system_ticks_ms;
                    park_step = 2;
                }
            }
            break;

        case 2: // 倒车旋转入库
            if (park_type == 20) {
                // 倒车转向 90度
                if (g_system_ticks_ms - step_time < 900) {
                    // 差速倒车
                    g_pwmL = Speed_Loop_Left(-12, g_encL);
                    g_pwmR = Speed_Loop_Right(-4, g_encR, g_trim);
                    Motor_SetSpeed(g_pwmL, g_pwmR);
                } else {
                    Motor_Brake();
                    Encoder_ResetDistance();
                    step_time = g_system_ticks_ms;
                    park_step = 3;
                }
            } else {
                // 侧方斜向入库
                if (g_system_ticks_ms - step_time < 800) {
                    g_pwmL = Speed_Loop_Left(-6, g_encL);
                    g_pwmR = Speed_Loop_Right(-14, g_encR, g_trim);
                    Motor_SetSpeed(g_pwmL, g_pwmR);
                } else {
                    Motor_Brake();
                    Encoder_ResetDistance();
                    step_time = g_system_ticks_ms;
                    park_step = 3;
                }
            }
            break;

        case 3: // 直线后退入位并刹车
            if (g_system_ticks_ms - step_time < 800) {
                g_pwmL = Speed_Loop_Left(-8, g_encL);
                g_pwmR = Speed_Loop_Right(-8, g_encR, g_trim);
                Motor_SetSpeed(g_pwmL, g_pwmR);
            } else {
                Motor_Brake();
                park_step = 4;
            }
            break;

        case 4: // 停车完成，严格锁死刹车不溜车
            Motor_Brake();
            break;
    }
}

// --------------------------- 标定与自检模式 ---------------------------
static void Process_Calibration_Mode(Track_Result_t *track)
{
    if (g_task == 3) {
        // 灰度标定模式: 电机静止，持续刷新 8 路 ADC 读数至 g_dbg
        Motor_Stop();
        for (int i = 0; i < 8; i++) {
            g_dbg[i] = track->raw_adc[i];
            g_dbg_bin[i] = track->is_black[i];
        }
    } else if (g_task == 4) {
        // 电机自检模式: 双轮 2 秒正转、1 秒停止、2 秒反转
        static uint32_t t_start = 0;
        uint32_t phase = (g_system_ticks_ms - t_start) % 5000;
        if (phase < 2000) {
            Motor_SetSpeed(3000, 3000);
        } else if (phase < 3000) {
            Motor_Brake();
        } else {
            Motor_SetSpeed(-3000, -3000);
        }
    } else if (g_task == 6) {
        // 自动配平模式: 给定相同目标脉冲，计算实际输出比值更新 g_trim
        static int32_t sum_l = 0, sum_r = 0;
        static uint16_t samples = 0;

        Motor_SetSpeed(3200, 3200);
        sum_l += g_encL;
        sum_r += g_encR;
        samples++;

        if (samples > 200) { // 采样 2 秒
            if (sum_r != 0) {
                g_trim = (float)sum_l / (float)sum_r;
            }
            Motor_Brake();
            g_task = 0; // 标定完成停转
        }
    }
}

// --------------------------- 主函数 ---------------------------
int main(void)
{
    // 1. 初始化系统滴答定时器 (1ms 中断)
    SysTick_Config(SystemCoreClock / 1000);

    // 2. 初始化硬件模块
    Motor_Init();
    Encoder_Init();
    Track_Init();
    Button_Init();

    // 上电安全等待 500ms
    Delay_Ms(500);

    Track_Result_t track_data;
    uint32_t last_ctrl_time = 0;

    while (1) {
        // 严格以 10ms 控制周期执行 (坑点4)
        if ((g_system_ticks_ms - last_ctrl_time) >= CTRL_PERIOD_MS) {
            last_ctrl_time = g_system_ticks_ms;

            // 1. 读取左右编码器脉冲增量
            g_encL = Encoder_GetLeftSpeed();
            g_encR = Encoder_GetRightSpeed();

            // 2. 扫描 8 路模拟灰度传感器
            Track_ScanAll(&track_data, g_track_threshold);

            // 3. 检查按键输入
            Check_Button();

            // 4. 根据模式分支执行
            if (g_task == 10) {
                Process_Basic_Task(&track_data);
            } else if (g_task == 20 || g_task == 21) {
                Process_Parking_Task(g_task);
            } else {
                Process_Calibration_Mode(&track_data);
            }
        }
    }
}
