/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MOTOR_FOC_H
#define _MOTOR_FOC_H

// direction of motor movement
#define RIGHT 		1
#define LEFT 		2

// phase states
#define OFF 		0
#define NORMAL 		1
#define INVERTED 	2

// State machine
//#define BLDC_NORMAL		0
//#define BLDC_OVER_MAX_CURRENT	1

#define DEGRES_120_IN_RADIANS 	(120.0 * (M_PI/180.0))

#define DEG_TO_RAD(angle) qfp_fmul(angle, M_PI/180.0)
#define RAD_TO_DEG(angle) qfp_fmul(angle, 180.0/M_PI)

extern volatile unsigned int motor_speed_erps;
extern volatile unsigned int PWM_cycles_counter;
extern volatile int motor_rotor_position;
extern volatile int position_correction_value;

extern volatile int adc_phase_a_current;
extern volatile int adc_phase_b_current;
extern volatile int adc_phase_c_current;

extern volatile int adc_phase_a_current_offset;
extern volatile int adc_phase_c_current_offset;

extern volatile int motor_max_current_factor;

extern volatile int mr_delta_phase_angle_right;
extern volatile int mr_delta_phase_angle_left;

void FOC_slow_loop (void);
void FOC_fast_loop (void);

void hall_sensors_read_and_action (void);
void hall_sensors_interrupt (void);
void motor_calc_current_dc_offset (void);

#endif /* _MOTOR_FOC_H_ */
