#include "pid.h"

static PID_Controller_t s_pid_speed_l;
static PID_Controller_t s_pid_speed_r;
static PID_Controller_t s_pid_steer;
static uint8_t s_pid_inited = 0;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float max_i, float max_out)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_integral = max_i;
    pid->max_output = max_out;
    PID_Reset(pid);
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->prev_err = 0.0f;
    pid->integral = 0.0f;
}

static void PID_EnsureInit(void)
{
    if (!s_pid_inited) {
        // 速度内环 PI 参数 (按 10ms 周期整定): Kp=140.0, Ki=12.0, 积分限幅 3000, 最大输出 7199
        PID_Init(&s_pid_speed_l, 140.0f, 12.0f, 0.0f, 3000.0f, 7199.0f);
        PID_Init(&s_pid_speed_r, 140.0f, 12.0f, 0.0f, 3000.0f, 7199.0f);
        // 转向外环 PD 参数 (按 10ms 周期整定): Kp=6.5, Kd=18.0, 最大差动限制 35 脉冲/10ms
        PID_Init(&s_pid_steer, 6.5f, 0.0f, 18.0f, 0.0f, 35.0f);
        s_pid_inited = 1;
    }
}

void PID_ResetAll(void)
{
    PID_EnsureInit();
    PID_Reset(&s_pid_speed_l);
    PID_Reset(&s_pid_speed_r);
    PID_Reset(&s_pid_steer);
}

float PID_PositionCompute(PID_Controller_t *pid, float target, float current)
{
    pid->err = target - current;

    // 积分分离与抗饱和
    pid->integral += pid->err;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    float derivative = pid->err - pid->last_err;
    pid->last_err = pid->err;

    float output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // 输出限幅
    if (output > pid->max_output) output = pid->max_output;
    if (output < -pid->max_output) output = -pid->max_output;

    return output;
}

/*
 * 左轮速度内环 (PI控制，10ms 周期)
 * 目标: target_pulse (每个控制周期脉冲数)
 * 实测: current_pulse
 * 输出: PWM 占空比 (-7199 ~ +7199)
 */
int16_t Speed_Loop_Left(int16_t target_pulse, int16_t current_pulse)
{
    PID_EnsureInit();

    if (target_pulse == 0 && current_pulse == 0) {
        PID_Reset(&s_pid_speed_l);
        return 0;
    }

    return (int16_t)PID_PositionCompute(&s_pid_speed_l, (float)target_pulse, (float)current_pulse);
}

/*
 * 右轮速度内环 (带配平因子 right_trim)
 */
int16_t Speed_Loop_Right(int16_t target_pulse, int16_t current_pulse, float right_trim)
{
    PID_EnsureInit();

    if (target_pulse == 0 && current_pulse == 0) {
        PID_Reset(&s_pid_speed_r);
        return 0;
    }

    // 乘以配平因子
    float trimmed_target = (float)target_pulse * right_trim;
    return (int16_t)PID_PositionCompute(&s_pid_speed_r, trimmed_target, (float)current_pulse);
}

/*
 * 转向外环 (PD控制)
 * 输入: line_error (-3.5 ~ +3.5，0为黑线正中，>0 表示黑线在车头右侧)
 * 输出: 转向差速量 steer_delta
 *   - 当 line_error > 0 (黑线偏右)，返回正值 steer_delta > 0，左轮加速、右轮减速实现右转追线
 *   - 当 line_error < 0 (黑线偏左)，返回负值 steer_delta < 0，左轮减速、右轮加速实现左转追线
 */
float Steering_Loop(float line_error)
{
    PID_EnsureInit();
    // 目标偏差为 0，输入 -line_error 使输出符号与 line_error 保持一致 (正值代表需右转)
    return PID_PositionCompute(&s_pid_steer, 0.0f, -line_error);
}
