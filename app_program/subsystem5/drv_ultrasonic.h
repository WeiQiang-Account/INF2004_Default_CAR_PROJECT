#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H
#include "subsystem5.h"

typedef enum { US_OK = 0, US_NO_ECHO, US_NO_SENSOR } us_status_t;
/* US_NO_ECHO  : echo line rose but nothing came back in range  -> path is clear
 * US_NO_SENSOR: echo line never rose at all                   -> wiring/sensor fault */

void        us_hw_init(void);                 /* TRIG, ECHO, servo PWM, GPIO IRQ branch */
void        servo_set_deg(int deg);           /* 90 = ahead, > 90 = left                */
us_status_t us_ping(uint16_t *mm);            /* one ping: mm or S5_DIST_NO_ECHO        */
us_status_t us_ping2(uint16_t *mm);           /* two pings, conservative median         */
#endif
