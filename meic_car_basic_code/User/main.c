/*
 * ============================================================================
 * MEIC 智能寻迹小车 · 招新题核心控制系统 (基础任务 + 任务二泊车 + 标定系统)
 * 主控：STM32F103C8T6 (C03C 核心板)
 * 硬件资源映射：
 *   - TB6612 电机驱动: PWMA(PA0), AIN1(PA1), AIN2(PA2), PWMB(PA3), BIN1(PB1), BIN2(PB8)
 *   - MG513XP28 编码器: 左(PA6/PA7, TIM3), 右(PB6/PB7, TIM4)
 *   - 74HC4051 8路模拟灰度: AD0(PB12), AD1(PB13), AD2(PB14), OUT(PB0, ADC1_IN8)
 *   - 系统时基定时器: TIM1 (1ms 硬件中断，避免与 stm32f10x_it.c 的 SysTick_Handler 冲突)
 *   - 板载按键: KEY1(PA11), KEY2(PA12)，用于运行时免烧录切换任务與启停
 * 
 * 运行模式设定 (RUN_MODE / g_task):
 *   10: 方形巡线，在第4个直角特征（A点）停车；停车位置须实车验证
 *   11: A-B 单边巡线，到第一个直角特征停车
 *    0: 直行闭环定速调试 (受 TEST_RUN0_STOP_CM 控制，0.0f 为不限距)
 *    1: 纯巡线连续调试 (连续巡线过弯，不计圈停车)
 *    2: 单次直角弯调试 (过完第 1 个直角弯回正后自动停车)
 *    3: 灰度传感器标定模式 (Keil Watch 查看 g_dbg[0..7])
 *    4: 电机驱动与接线自检 (交替正反转)
 *    6: 同 PWM 编码器对比模式（不自动改写 g_trim）
 *   20: 任务二 - 倒车入库 (定距直行 -> 差速倒车转90度 -> 倒车入库 -> 停车入位)
 *   21: 任务二 - 侧方停车 (定距直行 -> 斜向倒车入位 -> 回正停车)
 * ============================================================================
 */

#include "stm32f10x.h"
#include <string.h>

#include "motor.h"
#include "encoder.h"
#include "track.h"
#include "pid.h"

// --------------------------- 核心参数配置 ---------------------------
#define RUN_MODE                    10      // 默认基础任务全流程
#define CTRL_PERIOD_MS              10      // 控制周期严格保持 10ms (坑点4)
#define TEST_RUN0_STOP_CM           0.0f    // 直行定距停车强制关闭 (0.0f=不限距离，坑点2)
#define DEFAULT_SPEED_CMPS          12.0f   // 低速调试值；尚不能满足整圈30秒限时，须实测提速
#define LINE_LOST_LIMIT_MS          500U    // 持续丢线后锁定停车

// --------------------------- 全局调试变量 (Keil Watch 可直接观测) ---------------------------
volatile uint16_t g_dbg[8];                 // 8路灰度原始读数
volatile uint8_t  g_dbg_bin[8];             // 8路二值化状态 (1=压黑, 0=白底)
volatile int16_t  g_encL = 0;               // 左轮瞬时脉冲/10ms
volatile int16_t  g_encR = 0;               // 右轮瞬时脉冲/10ms
volatile int16_t  g_pwmL = 0;               // 左轮输出 PWM
volatile int16_t  g_pwmR = 0;               // 右轮输出 PWM
volatile float    g_trim = 1.00f;           // 右轮配平因子 (坑点3)
volatile float    g_pulse_per_cm = DEFAULT_PULSE_PER_CM; // 脉冲当量 (推车100cm实测校准)
volatile uint16_t g_track_threshold = DEFAULT_TRACK_THRESHOLD; // 灰度阈值
volatile uint8_t  g_task = RUN_MODE;        // 当前执行任务/模式 (可在 Watch 窗口或按键在线修改)
volatile uint8_t  g_fault = 0;             // 1=丢线, 2=转弯超时, 3=控制超时, 4=ADC, 5=参数
volatile uint8_t  g_corner_count = 0;       // 弯道/直角计数
volatile int32_t g_trim_sum_l = 0, g_trim_sum_r = 0; // 同PWM采样统计，仅用于诊断

// --------------------------- 状态机枚举 ---------------------------
typedef enum {
    STATE_IDLE = 0,
    STATE_START_ARMING,     // 起步/出弯武装：防起跑线或刚出弯误判直角 (坑点5)
    STATE_TRACKING,         // 闭环寻迹巡线
    STATE_CORNER_TURNING,   // 直角弯强制差速转向过渡
    STATE_STOPPED,          // 停车刹车制动
    // 泊车状态
    STATE_PARK_FORWARD,
    STATE_PARK_ROTATE,
    STATE_PARK_BACKWARD,
    STATE_PARK_FINISH
} CarState_t;

volatile CarState_t g_state = STATE_IDLE;
volatile uint32_t   g_system_ticks_ms = 0;

// --------------------------- TIM1 1ms 系统时基初始化 ---------------------------
/*
 * 使用 TIM1 产生 1ms 时基中断，避免与工程模板 stm32f10x_it.c 中的 SysTick_Handler 重复定义
 */
static void SystemTimer_TIM1_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // 72MHz / 72 = 1MHz 计数频率，重装载 1000 - 1 -> 1ms 中断周期
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM1, ENABLE);
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        g_system_ticks_ms++;
    }
}

static void Delay_Ms(uint32_t ms)
{
    uint32_t start = g_system_ticks_ms;
    while ((g_system_ticks_ms - start) < ms);
}

// --------------------------- 板载按键初始化与检测 ---------------------------
static void Button_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 检查按键切换模式 (坑点7: 验收中途不能重烧程序，支持按键切换任务与重启当前任务)
static void Check_Button(void)
{
    static uint8_t stable = 0, candidate = 0;
    static uint32_t changed_ms = 0;
    uint8_t keys = 0;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET) keys |= 1;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == Bit_RESET) keys |= 2;
    if (keys != candidate) {
        candidate = keys;
        changed_ms = g_system_ticks_ms;
    }
    if (keys != stable && g_system_ticks_ms - changed_ms >= 30) {
        uint8_t pressed = keys & (uint8_t)~stable;
        stable = keys;
        if (pressed & 1) {
            if (g_task == 10) g_task = 11;
            else if (g_task == 11) g_task = 20;
            else if (g_task == 20) g_task = 21;
            else g_task = 10;
            g_state = STATE_IDLE;
        } else if (pressed & 2) {
            g_state = STATE_IDLE;
        }
    }
}

// --------------------------- 巡线与基础任务状态机 (Mode 10 / 1 / 2) ---------------------------
static void Process_Track_And_Basic_Task(Track_Result_t *track, uint8_t mode)
{
    static uint32_t arm_start_ms = 0;
    static uint32_t turn_start_ms = 0;
    static uint8_t  s_armed = 0;
    static uint32_t lost_ms = 0;
    static uint8_t lost = 0;
    static uint8_t corner_samples = 0;
    static int8_t   turn_direction = 0; // -1: 左转直角, +1: 右转直角

    // 将目标速度(cm/s)转换为每个控制周期(10ms)的目标脉冲数
    float base_pulse = (DEFAULT_SPEED_CMPS * g_pulse_per_cm) * 0.01f;

    if (g_state == STATE_IDLE) {
        lost = 0;
        corner_samples = 0;
        g_fault = 0;
    }
    if (g_state == STATE_START_ARMING || g_state == STATE_TRACKING) {
        if (track->all_white) {
            if (!lost) { lost = 1; lost_ms = g_system_ticks_ms; }
            if (g_system_ticks_ms - lost_ms >= LINE_LOST_LIMIT_MS) {
                g_fault = 1;
                g_state = STATE_STOPPED;
            }
        } else lost = 0;
    }

    switch (g_state) {
        case STATE_IDLE:
            Motor_Stop();
            PID_ResetAll();
            Encoder_ResetDistance();
            s_armed = 0;
            g_corner_count = 0;
            arm_start_ms = g_system_ticks_ms;
            g_state = STATE_START_ARMING;
            break;

        case STATE_START_ARMING:
            // 坑点5：起步或刚出弯时，必须连续 > 300ms 检测到正常的中间单线(1~2路压黑)才激活直角判定
            {
                float steer = Steering_Loop(track->line_pos_error);
                int16_t target_l = (int16_t)(base_pulse + steer);
                int16_t target_r = (int16_t)(base_pulse - steer);
                g_pwmL = Speed_Loop_Left(target_l, g_encL);
                g_pwmR = Speed_Loop_Right(target_r, g_encR, g_trim);
                Motor_SetSpeed(g_pwmL, g_pwmR);

                if (track->black_count >= 1 && track->black_count <= 2 &&
                    (track->is_black[3] || track->is_black[4])) {
                    if (g_system_ticks_ms - arm_start_ms >= 300) {
                        s_armed = 1;
                        g_state = STATE_TRACKING;
                    }
                } else {
                    // 未稳定在单线上，重置连续计时器
                    arm_start_ms = g_system_ticks_ms;
                }
            }
            break;

        case STATE_TRACKING:
            // 正常闭环寻迹
            {
                // 检测是否到达直角弯
                if (s_armed && !track->is_cross_or_stop &&
                    (track->is_right_angle_left || track->is_right_angle_right)) {
                    if (++corner_samples < 3) break;
                    corner_samples = 0;
                    // 从 A 朝 B 发车：第1个弯是 B，第4个弯是 A；到点即停车。
                    g_corner_count++;
                    if (mode == 11 || (mode == 10 && g_corner_count >= 4)) {
                        Motor_Brake();
                        g_state = STATE_STOPPED;
                        break;
                    }
                    turn_start_ms = g_system_ticks_ms;

                    // 根据左右压黑探头数量判断转弯方向
                    if (track->is_right_angle_left || (track->left_black_count > track->right_black_count)) {
                        turn_direction = -1; // 左转
                    } else {
                        turn_direction = 1;  // 右转
                    }

                    PID_ResetAll();
                    g_state = STATE_CORNER_TURNING;
                    break;
                }

                corner_samples = 0;
                // 正常外环 PD 转向 + 内环 PI 速度控制
                float steer = Steering_Loop(track->line_pos_error);
                int16_t target_l = (int16_t)(base_pulse + steer);
                int16_t target_r = (int16_t)(base_pulse - steer);

                g_pwmL = Speed_Loop_Left(target_l, g_encL);
                g_pwmR = Speed_Loop_Right(target_r, g_encR, g_trim);
                Motor_SetSpeed(g_pwmL, g_pwmR);
            }
            break;

        case STATE_CORNER_TURNING:
            // 直角弯强制差速旋转过渡
            {
                if (turn_direction == -1) {
                    // 左转直角：左轮小幅反转制动，右轮加速前进
                    g_pwmL = Speed_Loop_Left((int16_t)(-base_pulse * 0.45f), g_encL);
                    g_pwmR = Speed_Loop_Right((int16_t)(base_pulse * 1.45f), g_encR, g_trim);
                } else {
                    // 右转直角：左轮加速前进，右轮小幅反转制动
                    g_pwmL = Speed_Loop_Left((int16_t)(base_pulse * 1.45f), g_encL);
                    g_pwmR = Speed_Loop_Right((int16_t)(-base_pulse * 0.45f), g_encR, g_trim);
                }
                Motor_SetSpeed(g_pwmL, g_pwmR);

                // 中心单线恢复才算完成转弯；1200ms超时进入故障停车。
                uint32_t turn_elapsed = g_system_ticks_ms - turn_start_ms;
                if (turn_elapsed >= 1200) {
                    g_fault = 2;
                    Motor_Brake();
                    g_state = STATE_STOPPED;
                    break;
                }
                if (turn_elapsed >= 320 && track->black_count >= 1 &&
                    track->black_count <= 2 && (track->is_black[3] || track->is_black[4])) {
                    PID_ResetAll();

                    // Mode 2: 单次直角弯调试模式，转完第 1 个弯即停车
                    if (mode == 2) {
                        Motor_Brake();
                        g_state = STATE_STOPPED;
                        break;
                    }

                    // 否则重新进入出弯武装状态，防止同一弯道重复计数
                    s_armed = 0;
                    arm_start_ms = g_system_ticks_ms;
                    g_state = STATE_START_ARMING;
                }
            }
            break;

        case STATE_STOPPED:
        default:
            Motor_Brake();
            break;
    }
}

// --------------------------- 任务二：泊车状态机 (Mode 20 倒车入库 / Mode 21 侧方停车) ---------------------------
static void Process_Parking_Task(uint8_t park_type)
{
    static uint32_t step_time = 0;
    float base_pulse = (DEFAULT_SPEED_CMPS * g_pulse_per_cm) * 0.01f;

    switch (g_state) {
        case STATE_IDLE:
            Motor_Stop();
            PID_ResetAll();
            Encoder_ResetDistance();
            step_time = g_system_ticks_ms;
            g_state = STATE_PARK_FORWARD;
            break;

        case STATE_PARK_FORWARD:
            // 第一步：从默认起点平稳直行至车库旁准备点
            {
                float target_dist = (park_type == 20) ? 40.0f : 32.0f;
                if (Encoder_GetAverageDistanceCm(g_pulse_per_cm) < target_dist) {
                    g_pwmL = Speed_Loop_Left((int16_t)base_pulse, g_encL);
                    g_pwmR = Speed_Loop_Right((int16_t)base_pulse, g_encR, g_trim);
                    Motor_SetSpeed(g_pwmL, g_pwmR);
                } else {
                    Motor_Brake();
                    PID_ResetAll();
                    Encoder_ResetDistance();
                    step_time = g_system_ticks_ms;
                    g_state = STATE_PARK_ROTATE;
                }
            }
            break;

        case STATE_PARK_ROTATE:
            // 第二步：差速倒车转向入库
            if (park_type == 20) {
                // 倒车入库：倒车旋转 90 度对准车库口
                if (g_system_ticks_ms - step_time < 850) {
                    g_pwmL = Speed_Loop_Left((int16_t)(-base_pulse * 1.2f), g_encL);
                    g_pwmR = Speed_Loop_Right((int16_t)(-base_pulse * 0.25f), g_encR, g_trim);
                    Motor_SetSpeed(g_pwmL, g_pwmR);
                } else {
                    Motor_Brake();
                    PID_ResetAll();
                    Encoder_ResetDistance();
                    step_time = g_system_ticks_ms;
                    g_state = STATE_PARK_BACKWARD;
                }
            } else {
                // 侧方停车：先右后斜向倒车切入车位
                if (g_system_ticks_ms - step_time < 750) {
                    g_pwmL = Speed_Loop_Left((int16_t)(-base_pulse * 0.4f), g_encL);
                    g_pwmR = Speed_Loop_Right((int16_t)(-base_pulse * 1.3f), g_encR, g_trim);
                    Motor_SetSpeed(g_pwmL, g_pwmR);
                } else {
                    Motor_Brake();
                    PID_ResetAll();
                    Encoder_ResetDistance();
                    step_time = g_system_ticks_ms;
                    g_state = STATE_PARK_BACKWARD;
                }
            }
            break;

        case STATE_PARK_BACKWARD:
            // 第三步：倒车入位/回正，确保停稳后四轮与车身投影完全在库内且不压线
            if (park_type == 20) {
                if (g_system_ticks_ms - step_time < 750) {
                    g_pwmL = Speed_Loop_Left((int16_t)(-base_pulse * 0.8f), g_encL);
                    g_pwmR = Speed_Loop_Right((int16_t)(-base_pulse * 0.8f), g_encR, g_trim);
                    Motor_SetSpeed(g_pwmL, g_pwmR);
                } else {
                    Motor_Brake();
                    g_state = STATE_PARK_FINISH;
                }
            } else {
                // 侧方停车左后回正入位
                if (g_system_ticks_ms - step_time < 650) {
                    g_pwmL = Speed_Loop_Left((int16_t)(-base_pulse * 1.2f), g_encL);
                    g_pwmR = Speed_Loop_Right((int16_t)(-base_pulse * 0.4f), g_encR, g_trim);
                    Motor_SetSpeed(g_pwmL, g_pwmR);
                } else {
                    Motor_Brake();
                    g_state = STATE_PARK_FINISH;
                }
            }
            break;

        case STATE_PARK_FINISH:
        default:
            Motor_Brake();
            break;
    }
}

// --------------------------- 标定与单项调试模式 (Mode 0 / 3 / 4 / 6) ---------------------------
static void Process_Calibration_And_Debug_Mode(void)
{
    static uint32_t mode_start_ms = 0;
    static uint16_t trim_samples = 0;

    if (g_state == STATE_IDLE) {
        Motor_Stop();
        PID_ResetAll();
        Encoder_ResetDistance();
        mode_start_ms = g_system_ticks_ms;
        g_trim_sum_l = 0;
        g_trim_sum_r = 0;
        trim_samples = 0;
        g_fault = 0;
        g_state = STATE_TRACKING;
    }

    if (g_task == 0) {
        // Mode 0: 直行定速闭环调试
        if (TEST_RUN0_STOP_CM > 0.01f &&
            Encoder_GetAverageDistanceCm(g_pulse_per_cm) >= TEST_RUN0_STOP_CM) {
            Motor_Brake();
            return;
        }
        int16_t target_pulse = (int16_t)((DEFAULT_SPEED_CMPS * g_pulse_per_cm) * 0.01f);
        g_pwmL = Speed_Loop_Left(target_pulse, g_encL);
        g_pwmR = Speed_Loop_Right(target_pulse, g_encR, g_trim);
        Motor_SetSpeed(g_pwmL, g_pwmR);
    } else if (g_task == 3) {
        // Mode 3: 灰度传感器静态标定模式 (电机停转，通过 Watch 窗口读取 g_dbg[0..7])
        Motor_Stop();
    } else if (g_task == 4) {
        // Mode 4: 电机正反转接线自检 (2秒正转 -> 1秒制动 -> 2秒反转)
        uint32_t phase = (g_system_ticks_ms - mode_start_ms) % 5000;
        if (phase < 2000) {
            Motor_SetSpeed(2800, 2800);
        } else if (phase < 3000) {
            Motor_Brake();
        } else {
            Motor_SetSpeed(-2800, -2800);
        }
    } else if (g_task == 6) {
        // Mode 6: 同 PWM 编码器对比，0.5 秒预热 + 1.5 秒采样，不修改闭环配平系数
        Motor_SetSpeed(3200, 3200);
        trim_samples++;
        if (trim_samples > 50 && trim_samples <= 200) {
            g_trim_sum_l += g_encL;
            g_trim_sum_r += g_encR;
        } else if (trim_samples > 200) {
            // 同 PWM 的转速比无法推出编码器每厘米当量，不能写入闭环目标 g_trim。
            // 保留原始累计脉冲供排查电机/编码器；配平须推车量距标定。
            Motor_Brake();
            g_task = 3; // 采样后切回静态观察模式
            g_state = STATE_IDLE;
        }
    } else {
        Motor_Stop();
    }
}

static uint8_t Control_CheckPeriod(uint32_t *last_ctrl_time)
{
    uint32_t elapsed = g_system_ticks_ms - *last_ctrl_time;
    if (elapsed < CTRL_PERIOD_MS) return 0;
    *last_ctrl_time = g_system_ticks_ms;
    if (elapsed >= 2U * CTRL_PERIOD_MS) {
        Motor_Brake();
        g_fault = 3;
        g_state = STATE_STOPPED;
    }
    return 1;
}

// --------------------------- 主函数 ---------------------------
int main(void)
{
    // 配置中断优先级分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // 1. 初始化 TIM1 1ms 硬件系统时基
    SystemTimer_TIM1_Init();

    // 2. 初始化外设硬件模块
    Motor_Init();
    Encoder_Init();
    Track_Init();
    Button_Init();
    PID_ResetAll();

    // 上电安全等待 500ms，避开手部按压抖动
    Delay_Ms(500);

    Track_Result_t track_data;
    memset(&track_data, 0, sizeof(track_data));

    uint32_t last_ctrl_time = g_system_ticks_ms;
    uint8_t  last_task = g_task;

    while (1) {
        // 严格以 10ms 控制周期执行 (坑点4)
        if (Control_CheckPeriod(&last_ctrl_time)) {

            // 1. 读取左右编码器脉冲增量
            g_encL = Encoder_GetLeftSpeed();
            g_encR = Encoder_GetRightSpeed();

            // 2. 扫描 8 路模拟灰度传感器，并同步更新全局调试数组 g_dbg 供 Keil Watch 随时查看
            Track_ScanAll(&track_data, g_track_threshold);
            for (int i = 0; i < 8; i++) {
                g_dbg[i] = track_data.raw_adc[i];
                g_dbg_bin[i] = track_data.is_black[i];
            }

            // 3. 检查按键输入与 Watch 窗口在线任务切换
            Check_Button();
            if (g_task != last_task) {
                last_task = g_task;
                g_state = STATE_IDLE;
            }

            // 4. 故障停车锁定，只有任务切换或重新启动才能解除。
            if (g_fault && g_state != STATE_IDLE) {
                Motor_Brake();
                continue;
            }
            if (g_state == STATE_IDLE) g_fault = 0;
            if (!track_data.adc_valid) {
                g_fault = 4;
                g_state = STATE_STOPPED;
                Motor_Brake();
                continue;
            }
            if (!(g_pulse_per_cm > 0.01f && g_pulse_per_cm < 1000.0f) ||
                !(g_trim >= 0.5f && g_trim <= 2.0f) ||
                g_track_threshold == 0 || g_track_threshold > 4095) {
                g_fault = 5;
                g_state = STATE_STOPPED;
                Motor_Brake();
                continue;
            }
            if (g_task == 10 || g_task == 11 || g_task == 1 || g_task == 2) {
                Process_Track_And_Basic_Task(&track_data, g_task);
            } else if (g_task == 20 || g_task == 21) {
                Process_Parking_Task(g_task);
            } else {
                Process_Calibration_And_Debug_Mode();
            }
        }
    }
}
