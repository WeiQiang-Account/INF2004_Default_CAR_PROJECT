/**
 * @file    pid.h
 * @brief   Small PID controller with anti-windup (one per wheel).
 */
#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;
    float ki;         /* per second */
    float kd;         /* seconds    */
    float i_limit;    /* clamp on the integral contribution */
    float out_limit;  /* clamp on the output                */
    float integ;      /* state: integral contribution       */
    float prev_err;   /* state: previous error              */
    int   has_prev;
} pid_ctrl_t;

void  pid_init(pid_ctrl_t *p_pid, float kp, float ki, float kd, float i_limit, float out_limit);
void  pid_reset(pid_ctrl_t *p_pid);

/** @param err  target - measured
 *  @param dt_s time since the last call, in seconds
 *  @param ff   feed-forward term added before clamping
 *  @return     controller output, clamped to +-out_limit                   */
float pid_update(pid_ctrl_t *p_pid, float err, float dt_s, float ff);

#endif /* PID_H */
