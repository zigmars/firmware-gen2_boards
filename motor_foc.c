/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2016, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "motor_foc.h"

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stdio.h"
#include "core_cm3.h"
#include "adc.h"
#include "buzzer.h"
#include "gpio.h"
#include "pwm.h"
#include "qfplib-m3.h"
#include "math.h"
#include "main.h"
#include "utils.h"
#include "filter.h"
#include "IMU/imu.h"
#include "pwm_duty_cycle_controller.h"
#include "balance_controller.h"

volatile unsigned int motor_speed_erps = 0; // motor speed in electronic rotations per second
volatile unsigned int PWM_cycles_per_SVM_TABLE_step = 0;
volatile unsigned int PWM_cycles_counter = 0;
volatile int motor_rotor_position = 0; // in degrees
volatile unsigned int motor_rotor_absolute_position = 0; // in degrees
volatile int position_correction_value = 0; // in degrees
volatile int interpolation_angle_step = 0; // x1000
volatile int interpolation_sum = 0; // x1000
volatile int interpolation_angle = 0;

volatile int adc_phase_a_current = 0;
volatile int adc_phase_b_current = 0;
volatile int adc_phase_c_current = 0;

volatile int adc_phase_a_current_sum = 0;
volatile int adc_phase_c_current_sum = 0;
volatile int adc_phase_current_sum_cycles = 0;

volatile int adc_phase_a_current_offset;
volatile int adc_phase_c_current_offset;

volatile int motor_max_current_factor = 1000; // 1000 -> 100%
volatile unsigned int over_current_fault_counter = 0;

volatile int mr_delta_phase_angle_right = MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT;
volatile int mr_delta_phase_angle_left = MOTOR_ROTOR_DELTA_PHASE_ANGLE_LEFT;


void motor_over_current_action (void)
{
	// disable PWM / motor!!
	disable_phase_a ();
	disable_phase_b ();
	disable_phase_c ();

	buzzer_on ();

	// infinite loop: block here, user will need to reset the system
	while (1) ;
}

// runs every 1ms
void FOC_slow_loop (void)
{
	//---------------------------
	// Clarke transform assuming balanced currents
	// removing DC offset
//GPIO_SetBits(BUZZER__PORT, BUZZER__PIN);
	__disable_irq();
		adc_phase_a_current = adc_phase_a_current_sum;
		adc_phase_a_current_sum = 0;
		adc_phase_c_current = adc_phase_c_current_sum;
		adc_phase_c_current_sum = 0;
		int temp_adc_phase_current_sum_cycles = adc_phase_current_sum_cycles;
		adc_phase_current_sum_cycles = 0;
		int temp_motor_rotor_position = motor_rotor_position;
		unsigned int temp_direction = get_motor_rotation_direction();
	__enable_irq();

	adc_phase_a_current /= temp_adc_phase_current_sum_cycles;
	adc_phase_c_current /= temp_adc_phase_current_sum_cycles;

	adc_phase_a_current = adc_phase_a_current - adc_phase_a_current_offset;
	adc_phase_c_current = adc_phase_c_current - adc_phase_c_current_offset;


	/* Calc phase B current assuming balanced currents
	 * considering: a + b + c = 0 ; a + c = -b ; b = -(a + c) ; b = -a -c */
#if MOTOR_TYPE == MOTOR_TYPE_EUC1
	adc_phase_b_current = -adc_phase_a_current - adc_phase_c_current;
#elif MOTOR_TYPE == MOTOR_TYPE_EUC2
	// change currents as for this motor, the phases are: BAC and not ABC
	adc_phase_b_current = adc_phase_a_current;
	adc_phase_a_current = -adc_phase_b_current - adc_phase_c_current;
#elif MOTOR_TYPE == MOTOR_TYPE_MICROWORKS_500W_30KMH
	adc_phase_b_current = adc_phase_c_current;
	adc_phase_c_current = adc_phase_a_current;
	adc_phase_a_current = -adc_phase_b_current - adc_phase_c_current;
#endif

	// calc ia and ib in Amps
	float ia = qfp_fmul(adc_phase_a_current, ADC_CURRENT_GAIN_MILLIAMPS);
	float ib = qfp_fmul(adc_phase_b_current, ADC_CURRENT_GAIN_MILLIAMPS);
	float ic = qfp_fmul(adc_phase_c_current, ADC_CURRENT_GAIN_MILLIAMPS);

	float i_alpha = ia;
	float i_beta = qfp_fdiv(qfp_fsub(ib, ic), ONE_BY_SQRT3);

	temp_motor_rotor_position = (temp_motor_rotor_position + (270 /*- 1*/)) % 360; // this makes the motor to run (almost) smooth in both directions -- id current seems good after this
	float motor_rotor_position_radians = DEG_TO_RAD(temp_motor_rotor_position);

	float id = qfp_fadd(qfp_fmul(i_alpha, qfp_fcos(motor_rotor_position_radians)), qfp_fmul(i_beta, qfp_fsin(motor_rotor_position_radians)));
	float iq = qfp_fsub(qfp_fmul(i_beta, qfp_fcos(motor_rotor_position_radians)), qfp_fmul(i_alpha, qfp_fsin(motor_rotor_position_radians)));

	// Filter Id and Iq currents
	// ------------------------------------------------------------------------
	float alpha_idiq = 5.0;
	static float moving_average_id = 0.0;
	static float moving_average_iq = 0.0;
	id = ema_filter_float(&id, &moving_average_id, &alpha_idiq);
	iq = ema_filter_float(&iq, &moving_average_iq, &alpha_idiq);
	// ------------------------------------------------------------------------

	// Calculate phase/angle correction value to try keep id current = 0
	// ------------------------------------------------------------------------
	static float correction_value = 0;

	// I
	correction_value = qfp_fsub(correction_value, qfp_fmul(K_POSITION_CORRECTION_VALUE, id));

	// D
	static float id_old = 0;
	float delta = qfp_fsub(id, id_old);
	id_old = id;
	correction_value = qfp_fsub(correction_value, qfp_fmul(D_POSITION_CORRECTION_VALUE, delta));


	if ((duty_cycle < 5 && duty_cycle > -5) || motor_speed_erps < 80) // avoid PI controller windup
	{ // motor_speed_erps < 80 seems a good value to avoid motor stalling at start up, very low speed
		correction_value = 0.0;
	}
	if (correction_value > 30.0) { correction_value = 30.0; }
	if (correction_value < -30.0) { correction_value = -30.0; }
	position_correction_value = (int) correction_value;

	// ------------------------------------------------------------------------

	// Max current controller
	// ------------------------------------------------------------------------
	if (abs_f(iq) > MOTOR_MAX_CURRENT) // start limitting the current
	{
		if (motor_max_current_factor > 0)
		{
			motor_max_current_factor -= MOTOR_MAX_CURRENT_STEP;
		}
	}
	else
	{
		if (motor_max_current_factor < 1000)
		{
			motor_max_current_factor += MOTOR_MAX_CURRENT_STEP;
		}
	}
	// ------------------------------------------------------------------------

	static unsigned int loop_timer1 = 0;
	loop_timer1++;
	if (loop_timer1 > 1)
	{
		loop_timer1 = 0;
		balance_controller ();
	}

	static unsigned int loop_timer = 0;
	loop_timer++;
	if (loop_timer > 10)
	{
		loop_timer = 0;

		int motor_speed = (int) motor_speed_erps;
		if (get_motor_rotation_direction() == LEFT) { 
			motor_speed *= -1;
		}
		
		printf ("%.2f, %.2f, %.2f, %d, %d\r\n", id, iq, correction_value, motor_speed, duty_cycle);
	}
}

// runs every 50us (PWM frequency)
void FOC_fast_loop (void)
{
	// count number of fast loops / PWM cycles
	if (PWM_cycles_counter < PWM_CYCLES_COUNTER_MAX)
	{
		PWM_cycles_counter++;
	}
	else
	{
		PWM_cycles_counter = 0;
		motor_speed_erps = 0;
		interpolation_angle_step = (SVM_TABLE_LEN * 1000) / PWM_CYCLES_COUNTER_MAX;
		interpolation_sum = 0;
	}

#define DO_INTERPOLATION 1 // may be useful when debugging
#if DO_INTERPOLATION == 1
	// calculate the interpolation angle
	// interpolation seems a problem when motor starts, so avoid to do it at very low speed
	if ( !(duty_cycle < 5 && duty_cycle > -5) || motor_speed_erps >= 80)
	{
		if (interpolation_sum <= (60 * 1000)) // interpolate only for angle <= 60º
		{
			// add step interpolation value to motor_rotor_position
			interpolation_sum += interpolation_angle_step;
			interpolation_angle = interpolation_sum / 1000;

			if (get_motor_rotation_direction() == RIGHT)
			{
				motor_rotor_position = mod_angle_degrees(motor_rotor_absolute_position + position_correction_value - interpolation_angle);
			}
			else
			{
				motor_rotor_position = mod_angle_degrees(motor_rotor_absolute_position + position_correction_value + interpolation_angle);
			}
		}
	}
#endif

	pwm_duty_cycle_controller ();

	// measure raw currents A and C and filter
	int _adc_phase_a_current = (int) adc_get_phase_a_current_value ();
	int _adc_phase_c_current = (int) adc_get_phase_c_current_value ();
	adc_phase_a_current_sum += _adc_phase_a_current;
	adc_phase_c_current_sum += _adc_phase_c_current;
	adc_phase_current_sum_cycles++;

	// verify motor over current
	//------------------------------------------------------------------------
	_adc_phase_a_current -= adc_phase_a_current_offset;
	int _adc_phase_b_current;
	_adc_phase_c_current -= adc_phase_c_current_offset;

	/* Calc phase B current assuming balanced currents
	 * considering: a + b + c = 0 ; a + c = -b ; b = -(a + c) ; b = -a -c */
#if MOTOR_TYPE == MOTOR_TYPE_EUC1
	_adc_phase_b_current = -_adc_phase_a_current - _adc_phase_c_current;
#elif MOTOR_TYPE == MOTOR_TYPE_EUC2
	// change currents as for this motor, the phases are: BAC and not ABC
	_adc_phase_b_current = _adc_phase_a_current;
	_adc_phase_a_current = -_adc_phase_b_current - _adc_phase_c_current;
#elif MOTOR_TYPE == MOTOR_TYPE_MICROWORKS_500W_30KMH
	_adc_phase_b_current = _adc_phase_c_current;
	_adc_phase_c_current = _adc_phase_a_current;
	_adc_phase_a_current = -_adc_phase_b_current -_adc_phase_c_current;
#endif

	if (
			(abs(_adc_phase_a_current) > ADC_MOTOR_OVER_CURRENT_LIMIT) ||
			(abs(_adc_phase_b_current) > ADC_MOTOR_OVER_CURRENT_LIMIT) ||
			(abs(_adc_phase_c_current) > ADC_MOTOR_OVER_CURRENT_LIMIT))
	{
		over_current_fault_counter += 10;

		if (over_current_fault_counter > 1000) { motor_over_current_action(); }
	}
	else
	{
		if (over_current_fault_counter > 0) { over_current_fault_counter -= 1; }
	}
	//------------------------------------------------------------------------

}

// calc the DC offset value for the current ADCs
void motor_calc_current_dc_offset (void)
{
	adc_phase_a_current_offset = 0;
	adc_phase_c_current_offset = 0;
	unsigned int i;
	for (i = 0; i < 200; i++)
	{
		adc_phase_a_current_offset += adc_get_phase_a_current_value ();
		adc_phase_c_current_offset += adc_get_phase_c_current_value ();
		delay_ms (10);
	}
	adc_phase_a_current_offset /= i;
	adc_phase_c_current_offset /= i;
}

void hall_sensors_read_and_action (void)
{
	unsigned int hall_sensors = 0;
	static unsigned int flag_count_speed = 0;

	// read hall sensors signal pins and mask other pins
	hall_sensors = (GPIO_ReadInputData (HALL_SENSORS__PORT) & (HALL_SENSORS_MASK));

	if (get_motor_rotation_direction() == LEFT)
	{
		switch (hall_sensors) // angle increments with rotation
		{
			case (1<<13):
			motor_rotor_absolute_position = (60 * 5); // 6
			break;

			case (1<<14)+(1<<13): // transition to positive value of hall sensor A
			motor_rotor_absolute_position = (60 * 4); // 5
			break;

			case (1<<14):
			motor_rotor_absolute_position = (60 * 3); // 4
			flag_count_speed = 1;
			break;

			case (1<<14)+(1<<12):
			motor_rotor_absolute_position = (60 * 2); // 3
			break;

			case (1<<12):
			motor_rotor_absolute_position = (60 * 1); // 2
			break;

			case (1<<13)+(1<<12):
			motor_rotor_absolute_position = (60 * 0); // 1

			// count speed only when motor did rotate half of 1 electronic rotation
			if (flag_count_speed)
			{
				flag_count_speed = 0;
				motor_speed_erps = PWM_CYCLES_COUNTER_MAX / PWM_cycles_counter;
				interpolation_angle_step = (SVM_TABLE_LEN * 1000) / PWM_cycles_counter;
				PWM_cycles_counter = 0;
			}
			break;

			default:
			return;
			break;
		}

		motor_rotor_absolute_position += mr_delta_phase_angle_left;
	}
	else if (get_motor_rotation_direction() == RIGHT)
	{
		switch (hall_sensors) // angle DEcrements with rotation
		{
			case (1<<13):
			motor_rotor_absolute_position = (60 * 2); // 3
			break;

			case (1<<14)+(1<<13): // transition to positive value of hall sensor A
			motor_rotor_absolute_position = (60 * 1); // 2
			break;

			case (1<<14):
			motor_rotor_absolute_position = (60 * 0); // 1

			// count speed only when motor did rotate half of 1 electronic rotation
			if (flag_count_speed)
			{
				flag_count_speed = 0;
				motor_speed_erps = PWM_CYCLES_COUNTER_MAX / PWM_cycles_counter;
				interpolation_angle_step = (SVM_TABLE_LEN * 1000) / PWM_cycles_counter;
				PWM_cycles_counter = 0;
			}
			break;

			case (1<<12)+(1<<14):
			motor_rotor_absolute_position = (60 * 5); // 6
			break;

			case (1<<12):
			motor_rotor_absolute_position = (60 * 4); // 5
			break;

			case (1<<13)+(1<<12):
			motor_rotor_absolute_position = (60 * 3); // 4
			flag_count_speed = 1;
			break;

			default:
			return;
			break;
		}

		motor_rotor_absolute_position += mr_delta_phase_angle_right;
	}

	motor_rotor_position = mod_angle_degrees(motor_rotor_absolute_position + position_correction_value);
	interpolation_sum = 0;
}

void hall_sensors_interrupt (void)
{
	hall_sensors_read_and_action ();
}
