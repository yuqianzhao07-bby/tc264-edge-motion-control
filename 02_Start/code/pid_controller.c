#include "pid_controller.h"

void pid_init(PID_Controller *pid, int32 kp, int32 ki, int32 kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->mode = PID_MODE_POSITION;  // 默认位置式
    pid_reset(pid);
}

void pid_set_mode(PID_Controller *pid, PID_Mode mode)
{
    pid->mode = mode;
}

void pid_reset(PID_Controller *pid)
{
    pid->error = 0;
    pid->last_error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->last_output = 0;
    pid->output = 0;
    pid->call_count = 0;

    // 默认参数
    pid->error_thres = 10;
    pid->max_step = 500;
}

void pid_set_limits(PID_Controller *pid, int32 min, int32 max)
{
    pid->out_min = min;
    pid->out_max = max;
}

void pid_set_step_limit(PID_Controller *pid, int32 max_step)
{
    pid->max_step = max_step;
}

void pid_set_error_thres(PID_Controller *pid, int32 thres)
{
    pid->error_thres = thres;
}

void pid_update_params(PID_Controller *pid, int32 kp, int32 ki, int32 kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

// ==========================================
// PID计算: 支持位置式和增量式两种模式
// ==========================================
int32 pid_calculate(PID_Controller *pid, int32 error)
{
    pid->call_count++;
    pid->error = error;

    if (pid->mode == PID_MODE_POSITION)
    {
        // === 位置式PID ===
        // output = Kp*error + Ki*integral + Kd*(error - last_error)
        // 对大偏差响应快, 一帧到位

        // 比例项
        int32 p_term = pid->Kp * pid->error;

        // 积分项 (积分分离)
        int32 i_term = 0;
        if (pid->Ki != 0)
        {
            int32 abs_error = (pid->error > 0) ? pid->error : -pid->error;
            if (abs_error < pid->error_thres)
            {
                pid->integral += pid->error;
                // 积分限幅
                if (pid->integral > 8000) pid->integral = 8000;
                if (pid->integral < -8000) pid->integral = -8000;
                i_term = pid->Ki * pid->integral;
            }
            else
            {
                pid->integral = 0;
            }
        }

        // 微分项 (限幅delta_error防掉帧导致的D项尖峰)
        int32 delta = pid->error - pid->last_error;
        if (delta > 15)  delta = 15;   // 掉帧时error跳变, 限幅防急转
        if (delta < -15) delta = -15;
        int32 d_term = pid->Kd * delta;

        // 直接输出
        pid->output = p_term + i_term + d_term;

        // 输出限幅
        if (pid->output > pid->out_max)
            pid->output = pid->out_max;
        if (pid->output < pid->out_min)
            pid->output = pid->out_min;
    }
    else
    {
        // === 增量式PID ===
        // increment = Kp*(e-e') + Ki*e + Kd*(e-2e'+e'')
        // 输出平稳, 适合速度控制

        int32 p_term = pid->Kp * (pid->error - pid->last_error);

        int32 i_term = 0;
        if (pid->Ki != 0)
        {
            int32 abs_error = (pid->error > 0) ? pid->error : -pid->error;
            if (abs_error < pid->error_thres)
            {
                pid->integral += pid->error;
                // 积分限幅 (防止饱和后累积过大)
                if (pid->integral > 8000) pid->integral = 8000;
                if (pid->integral < -8000) pid->integral = -8000;
                i_term = pid->Ki * pid->error;
            }
            else
            {
                pid->integral = 0;
            }
        }

        // 微分项 (限幅加速度防掉帧尖峰)
        int32 delta2 = pid->error - 2 * pid->last_error + pid->prev_error;
        if (delta2 > 30)  delta2 = 30;
        if (delta2 < -30) delta2 = -30;
        int32 d_term = pid->Kd * delta2;

        int32 increment = p_term + i_term + d_term;

        // 输出变化率限制
        if (increment > pid->max_step)
            increment = pid->max_step;
        else if (increment < -pid->max_step)
            increment = -pid->max_step;

        pid->output += increment;

        // 输出限幅
        if (pid->output > pid->out_max)
        {
            pid->output = pid->out_max;
            pid->integral = 0;
        }
        if (pid->output < pid->out_min)
        {
            pid->output = pid->out_min;
            pid->integral = 0;
        }
    }

    // 更新历史
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    pid->last_output = pid->output;

    return pid->output;
}
