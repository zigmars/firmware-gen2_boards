/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _PWM_H_
#define _PWM_H_

#include "stm32f10x.h"

#define PWM_PERIOD_US 						50
// #define PWM_FULL_PERIOD_IN_SYS_CLKS 		((SystemCoreClock * PWM_PERIOD_US) / 1000000) /* PWM_PERIOD_S / (1/SycClk) */
#define PWM_FULL_PERIOD_IN_SYS_CLKS 		((u32)((		 72000000ULL * PWM_PERIOD_US) / 1000000)) /* PWM_PERIOD_S / (1/SycClk) */
#define PWM_HALF_PERIOD_IN_SYS_CLKS			(PWM_FULL_PERIOD_IN_SYS_CLKS / 2) /* PERIOD = PWM_FULL_PERIOD_IN_SYS_CLKS / 2 : for center aligned mode */
#define PWM_VALUE_DUTY_CYCLE_MAX 			(PWM_HALF_PERIOD_IN_SYS_CLKS-1)
#define MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX     (PWM_VALUE_DUTY_CYCLE_MAX / 2)

#define PWM_PERIOD_INTERRUPT		TIM3_IRQHandler

void enable_phase_a (void);
void enable_phase_b (void);
void enable_phase_c (void);
void disable_phase_a (void);
void disable_phase_b (void);
void disable_phase_c (void);
void set_pwm_phase_a (unsigned int value);
void set_pwm_phase_b (unsigned int value);
void set_pwm_phase_c (unsigned int value);

void pwm_init (void);

#endif
