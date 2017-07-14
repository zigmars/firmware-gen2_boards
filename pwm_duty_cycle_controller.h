/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2016, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _PWM_DUTY_CYCLE_CONTROLLER_H_
#define _PWM_DUTY_CYCLE_CONTROLLER_H_

#include "pwm.h"

#define SVM_TABLE_LEN 360

extern const unsigned int svm_table [SVM_TABLE_LEN];
extern int duty_cycle;

void apply_duty_cycle (int duty_cycle_value);
void pwm_duty_cycle_controller (void);
void set_pwm_duty_cycle (int target_value);
unsigned int get_motor_rotation_direction (void);
unsigned int calculate_duty_cycle(unsigned int rotor_pos_deg, unsigned int amplitude);

#endif
