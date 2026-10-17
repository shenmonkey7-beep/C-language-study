/* Exercise the production C implementation with deterministic peripheral inputs. */
#include <assert.h>
#include <stdio.h>
#define main firmware_main
#include "../meic_car_basic_code/User/main.c"
#undef main
#include "../meic_car_basic_code/Hardware/motor.c"
#include "../meic_car_basic_code/Hardware/encoder.c"
#include "../meic_car_basic_code/Hardware/track.c"
#include "../meic_car_basic_code/Hardware/pid.c"

static Track_Result_t center(void)
{
    Track_Result_t t;
    memset(&t, 0, sizeof(t));
    t.black_count = 2;
    t.is_black[3] = t.is_black[4] = 1;
    t.adc_valid = 1;
    return t;
}

static void arm(Track_Result_t *t, uint8_t mode)
{
    g_state = STATE_IDLE;
    Process_Track_And_Basic_Task(t, mode);
    for (int i = 0; i < 31; ++i) {
        g_system_ticks_ms += 10;
        Process_Track_And_Basic_Task(t, mode);
    }
    assert(g_state == STATE_TRACKING);
}

static void corner(Track_Result_t *t, uint8_t mode)
{
    t->black_count = 3;
    t->is_right_angle_right = 1;
    t->is_black[3] = t->is_black[4] = 0;
    for (int i = 0; i < 3; ++i) {
        g_system_ticks_ms += 10;
        Process_Track_And_Basic_Task(t, mode);
    }
}

int main(void)
{
    Track_Result_t t = center();
    PID_Controller_t p;
    // Anti-windup: sustained impossible demand must not accumulate integral.
    PID_Init(&p, 10, 1, 0, 1000, 100);
    for (int i = 0; i < 1000; ++i) assert(PID_PositionCompute(&p, 100, 0) == 100);
    assert(p.integral == 0);
    assert(PID_PositionCompute(&p, -1, 0) < 0);
    PID_ResetAll();
    assert(Steering_Loop(1) > 0);
    PID_ResetAll();
    assert(Steering_Loop(-1) < 0);

    Encoder_Init();
    mock_t3.count = 65530;
    assert(Encoder_GetLeftSpeed() == -6);
    mock_t3.count = 4;
    assert(Encoder_GetLeftSpeed() == 10);
    Encoder_ResetDistance();
    mock_t3.count = 7;
    assert(Encoder_GetLeftSpeed() == 3); // Reset distance must not erase pending pulses.
    mock_t4.count = 65530;
    assert(Encoder_GetRightSpeed() == 6);

    Track_Init();
    for (int i = 0; i < 8; ++i) mock_adc_values[i] = 3000;
    mock_adc_values[3] = mock_adc_values[4] = 800;
    Track_ScanAll(&t, 1800);
    assert(t.adc_valid && t.black_count == 2 && t.line_pos_error == 0);
    mock_adc_failure = 1;
    Track_ScanAll(&t, 1800);
    assert(!t.adc_valid);
    mock_adc_failure = 0;

    t = center();
    arm(&t, 11);
    corner(&t, 11);
    assert(g_state == STATE_STOPPED && g_corner_count == 1);
    t = center();
    arm(&t, 10);
    // Arming requires the center probes, not merely any one or two dark probes.
    g_state = STATE_IDLE;
    Process_Track_And_Basic_Task(&t, 10);
    t.is_black[3] = t.is_black[4] = 0;
    t.is_black[0] = 1;
    t.black_count = 1;
    g_system_ticks_ms += 500;
    Process_Track_And_Basic_Task(&t, 10);
    assert(g_state == STATE_START_ARMING);
    t = center();
    arm(&t, 10);
    for (int i = 0; i < 4; ++i) {
        corner(&t, 10);
        if (i == 3) break;
        assert(g_state == STATE_CORNER_TURNING);
        t = center();
        g_system_ticks_ms += 330;
        Process_Track_And_Basic_Task(&t, 10);
        assert(g_state == STATE_START_ARMING);
        g_system_ticks_ms += 310;
        Process_Track_And_Basic_Task(&t, 10);
        assert(g_state == STATE_TRACKING);
    }
    assert(g_state == STATE_STOPPED && g_corner_count == 4);
    assert(mock_t2.ccr1 == 0 && mock_t2.ccr4 == 0);

    t = center();
    arm(&t, 10);
    corner(&t, 10);
    g_system_ticks_ms += 1200;
    Process_Track_And_Basic_Task(&t, 10);
    assert(g_fault == 2 && g_state == STATE_STOPPED);
    t = center();
    arm(&t, 10);
    t.all_white = 1;
    t.black_count = 0;
    Process_Track_And_Basic_Task(&t, 10);
    g_system_ticks_ms += 500;
    Process_Track_And_Basic_Task(&t, 10);
    assert(g_fault == 1 && g_state == STATE_STOPPED);

    t = center();
    arm(&t, 10);
    t.is_cross_or_stop = 1;
    t.black_count = 8;
    for (int i = 0; i < 5; ++i) Process_Track_And_Basic_Task(&t, 10);
    assert(g_corner_count == 0);

    // Held key causes exactly one selection; releasing then pressing allows another.
    mock_a.input = GPIO_Pin_11 | GPIO_Pin_12;
    Check_Button();
    g_system_ticks_ms += 40;
    Check_Button();
    g_task = 10;
    mock_a.input &= (uint16_t)~GPIO_Pin_11;
    Check_Button();
    g_system_ticks_ms += 40;
    Check_Button();
    assert(g_task == 11);
    g_system_ticks_ms += 1000;
    Check_Button();
    assert(g_task == 11);

    g_task = 6;
    g_state = STATE_IDLE;
    g_trim = 1.2f;
    g_encL = 20;
    g_encR = 10;
    for (int i = 0; i < 201; ++i) Process_Calibration_And_Debug_Mode();
    assert(g_task == 3 && g_trim == 1.2f);
    {
        uint32_t last = 0xFFFFFFFAU;
        g_system_ticks_ms = 4;
        assert(Control_CheckPeriod(&last)); // 32-bit tick wrap is still 10ms.
        assert(!Control_CheckPeriod(&last)); // Never catch up with immediate extra loops.
        g_system_ticks_ms += 20;
        assert(Control_CheckPeriod(&last));
        assert(g_fault == 3 && g_state == STATE_STOPPED);
    }
    puts("PASS: PID, steering, encoder wrap/reset, ADC failure, A-B/lap stop, arming, turn timeout, line loss, crossing, held key, calibration, scheduler wrap/overrun");
    return 0;
}
