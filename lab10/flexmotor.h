#ifndef __FLEXMOTOR_H
#define __FLEXMOTOR_H

#include "types.h"

void motor_init(/*uint8_t chan*/);
void motor_set_duty(uint8_t chan, uint16_t duty_us);

#endif