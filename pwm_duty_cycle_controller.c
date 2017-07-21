/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "main.h"
#include "gpio.h"
#include "pwm.h"
#include "pwm_duty_cycle_controller.h"
#include "motor_foc.h"
#include "utils.h"

// Full amplitude
const unsigned int svm_table [SVM_TABLE_LEN] = \
{
	900, 927, 954, 982, 1009, 1036, 1063, 1090, 1117, 1144, 1171, 1197, \
	1224, 1251, 1277, 1303, 1330, 1356, 1382, 1408, 1433, 1459, 1484, \
	1509, 1534, 1559, 1583, 1608, 1632, 1656, 1679, 1687, 1695, 1702, \
	1709, 1716, 1722, 1728, 1734, 1740, 1746, 1751, 1756, 1761, 1765, \
	1769, 1773, 1777, 1780, 1783, 1786, 1789, 1791, 1793, 1795, 1797, \
	1798, 1799, 1799, 1800, 1800, 1800, 1799, 1799, 1798, 1797, 1795, \
	1793, 1791, 1789, 1786, 1783, 1780, 1777, 1773, 1769, 1765, 1761, \
	1756, 1751, 1746, 1740, 1734, 1728, 1722, 1716, 1709, 1702, 1695, \
	1687, 1679, 1687, 1695, 1702, 1709, 1716, 1722, 1728, 1734, 1740, \
	1746, 1751, 1756, 1761, 1765, 1769, 1773, 1777, 1780, 1783, 1786, \
	1789, 1791, 1793, 1795, 1797, 1798, 1799, 1799, 1800, 1800, 1800, \
	1799, 1799, 1798, 1797, 1795, 1793, 1791, 1789, 1786, 1783, 1780, \
	1777, 1773, 1769, 1765, 1761, 1756, 1751, 1746, 1740, 1734, 1728, \
	1722, 1716, 1709, 1702, 1695, 1687, 1679, 1656, 1632, 1608, 1583, \
	1559, 1534, 1509, 1484, 1459, 1433, 1408, 1382, 1356, 1330, 1303, \
	1277, 1251, 1224, 1197, 1171, 1144, 1117, 1090, 1063, 1036, 1009, \
	982, 954, 927, 900, 873, 846, 818, 791, 764, 737, 710, 683, 656, 629, \
	603, 576, 549, 523, 497, 470, 444, 418, 392, 367, 341, 316, 291, 266, \
	241, 217, 192, 168, 144, 121, 113, 105, 98, 91, 84, 78, 72, 66, 60, \
	54, 49, 44, 39, 35, 31, 27, 23, 20, 17, 14, 11, 9, 7, 5, 3, 2, 1, 1, \
	0, 0, 0, 1, 1, 2, 3, 5, 7, 9, 11, 14, 17, 20, 23, 27, 31, 35, 39, 44, \
	49, 54, 60, 66, 72, 78, 84, 91, 98, 105, 113, 121, 113, 105, 98, 91, \
	84, 78, 72, 66, 60, 54, 49, 44, 39, 35, 31, 27, 23, 20, 17, 14, 11, \
	9, 7, 5, 3, 2, 1, 1, 0, 0, 0, 1, 1, 2, 3, 5, 7, 9, 11, 14, 17, 20, \
	23, 27, 31, 35, 39, 44, 49, 54, 60, 66, 72, 78, 84, 91, 98, 105, 113, \
	121, 144, 168, 192, 217, 241, 266, 291, 316, 341, 367, 392, 418, 444, \
	470, 497, 523, 549, 576, 603, 629, 656, 683, 710, 737, 764, 791, 818, \
	846, 873
};

int duty_cycle_target;
int duty_cycle = 0;
unsigned int _direction;

void apply_duty_cycle (int duty_cycle_value)
{
	int _duty_cycle = duty_cycle_value;
	unsigned int value_a = 0;
	unsigned int value_b = 0;
	unsigned int value_c = 0;
	static unsigned int old_direction = 0;

	if (_duty_cycle >= 0)
	{
		_direction = RIGHT;
	}
	else
	{
		_direction = LEFT;
		_duty_cycle = -_duty_cycle; // invert the value, to be always positive
	}

	// if direction changes, we need to execute the code of commutation for the new update of the angle other way the motor will block
	if (_direction != old_direction)
	{
		old_direction = _direction;
		hall_sensors_read_and_action ();
	}

	// apply over current limit factor
	_duty_cycle *= motor_max_current_factor;
	_duty_cycle /= 1000;

	// apply minimum duty_cycle value
	/*	I found that there are a minimum duty_cycle value after the motor start to move, 
		like if there is a gap with low values. This code is to remove that gap and 
		I expect it to improve the quick motor direction change rotation that 
		I think is important for the balance of EUC.
	*/
	int temp1 = 1000 - MOTOR_MIN_DUTYCYCLE;
	_duty_cycle = MOTOR_MIN_DUTYCYCLE + (((_duty_cycle * temp1) + MOTOR_MIN_DUTYCYCLE) / 1000);

	// apply limits
	if (_duty_cycle > 1000) _duty_cycle = 1000;
	if (_duty_cycle < 0) _duty_cycle = 0;

	// scale and apply _duty_cycle
	unsigned int temp_rotor_pos = (unsigned int)motor_rotor_position;
	value_a = calculate_duty_cycle(temp_rotor_pos, (unsigned int)_duty_cycle);

	// add 120 degrees and limit
	temp_rotor_pos = mod_angle_degrees(motor_rotor_position + 120);
	value_b = calculate_duty_cycle(temp_rotor_pos, (unsigned int)_duty_cycle);

	// subtract 120 degrees and limit
	temp_rotor_pos = mod_angle_degrees(motor_rotor_position + 240);
	value_c = calculate_duty_cycle(temp_rotor_pos, (unsigned int)_duty_cycle);

#if MOTOR_TYPE == MOTOR_TYPE_EUC1
	set_pwm_phase_a (value_a);
	set_pwm_phase_b (value_b);
	set_pwm_phase_c (value_c);
#elif MOTOR_TYPE == MOTOR_TYPE_EUC2
	set_pwm_phase_a (value_b);
	set_pwm_phase_b (value_a);
	set_pwm_phase_c (value_c);
#elif MOTOR_TYPE == MOTOR_TYPE_MICROWORKS_500W_30KMH
	set_pwm_phase_a (value_b);
	set_pwm_phase_b (value_c);
	set_pwm_phase_c (value_a);
#endif
}

// run at each PWM period = 50us
void pwm_duty_cycle_controller (void)
{
	// limit PWM increase/decrease rate
	static unsigned int counter = 0;
	if (counter++ > PWM_DUTY_CYCLE_CONTROLLER_COUNTER)
	{
		counter = 0;

		// increment or decrement duty_cycle
		if (duty_cycle_target > duty_cycle) { duty_cycle++; }
		else if (duty_cycle_target < duty_cycle) { duty_cycle--; }
	}
	apply_duty_cycle (duty_cycle);
}

void set_pwm_amplitude_target (int target_value)
{
	duty_cycle_target = target_value;
}

unsigned int get_motor_rotation_direction (void)
{
	return _direction;
}

unsigned int calculate_duty_cycle(unsigned int rotor_pos_deg, unsigned int amplitude)
{
	int sv_value = svm_table[rotor_pos_deg];
	sv_value = (sv_value - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * amplitude;
	sv_value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + (sv_value / 1000);
	return (unsigned int) sv_value;
}
