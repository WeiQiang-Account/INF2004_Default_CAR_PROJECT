/**
 * @file    pid.c
 * @brief   PID with clamped integral and "stop integrating when saturated"
 *          anti-windup, so the wheel does not overshoot after a stall or hump.
 */
#include "pid.h"

static float pid_clamp(float v, float lim)
{
    if (v > lim)  { return lim; }
    if (v < -lim) { return -lim; }
    return v;
}

void pid_init(pid_ctrl_t *p_pid, float kp, float ki, float kd, float i_limit, float out_limit)
{
    p_pid->kp = kp;
    p_pid->ki = ki;
    p_pid->kd = kd;
    p_pid->i_limit = i_limit;
    p_pid->out_limit = out_limit;
    pid_reset(p_pid);
}

void pid_reset(pid_ctrl_t *p_pid)
{
    p_pid->integ = 0.0f;
    p_pid->prev_err = 0.0f;
    p_pid->has_prev = 0;
}

float pid_update(pid_ctrl_t *p_pid, float err, float dt_s, float ff)
{
    float p = p_pid->kp * err;
    float d = 0.0f;
    float new_integ = pid_clamp(p_pid->integ + (p_pid->ki * err * dt_s), p_pid->i_limit);
    float out;

    if ((p_pid->has_prev != 0) && (dt_s > 0.0f))
    {
        d = p_pid->kd * (err - p_pid->prev_err) / dt_s;
    }

    out = ff + p + new_integ + d;

    /* Anti-windup: only keep the new integral if it does not push further
     * into saturation.                                                     */
    if (((out >  p_pid->out_limit) && (err > 0.0f)) ||
        ((out < -p_pid->out_limit) && (err < 0.0f)))
    {
        out = ff + p + p_pid->integ + d;
    }
    else
    {
        p_pid->integ = new_integ;
    }

    p_pid->prev_err = err;
    p_pid->has_prev = 1;
    return pid_clamp(out, p_pid->out_limit);
}
