/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "gpio.h"
#include "main.h"
#include "stdio.h"
#include "buzzer.h"
#include "hall_sensors.h"
#include "leds.h"
#include "pwm.h"
#include "pwm_duty_cycle_controller.h"
#include "usart.h"
#include "filter.h"
#include "math.h"
#include "qfplib-m3.h"
#include "filter.h"
#include "stm32f10x_tim.h"
#include "timer.h"
#include "adc.h"
#include "usart.h"
#include "IMU/imu.h"
#include "balance_controller.h"
#include "motor_foc.h"

static volatile unsigned int _ms;

unsigned int log_enable = 0;

void delay_ms (unsigned int ms)
{
	_ms = 1;
	while (ms >= _ms) ;
}

void SysTick_Handler(void) // runs every 1ms
{
	// for delay_ms ()
	_ms++;
}

void initialize (void)
{
	/* Setup SysTick Timer for 1 millisecond interrupts, also enables Systick and Systick-Interrupt */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}

	gpio_init ();
	buzzer_init ();
	leds_init();
	if (! IMU_init () ){
		buzzer_on();
		delay_ms(2000);
		buzzer_off();
		while(1);
	}
	TIM2_init ();
	adc_init ();
	motor_calc_current_dc_offset ();
//  TIM4_init ();
	usart1_bluetooth_init ();
	hall_sensor_init ();
	pwm_init ();
}

int main(void)
{
	/* MCU is initialized in 72MHz mode (by default) */
	
	/* needed for printf */
	// turn off buffers, so IO occurs immediately
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	initialize ();

	buzzer_on();
	delay_ms(20);
	buzzer_off();
	
	// don't start until the potentiometer is on the middle value --> PWM ~= 0
	unsigned int duty_cycle_value;
	while ((duty_cycle_value = adc_get_potentiometer_value()) < 1720 ||
					duty_cycle_value > 1880);


	set_pwm_amplitude_target (0);
	enable_phase_a ();
	enable_phase_b ();
	enable_phase_c ();


	hall_sensors_interrupt ();

	static unsigned int moving_average = 4095 / 2;
	unsigned int alpha = 20;
	char buffer[64];
	float value;
	while (1)
	{
		delay_ms (1);

		led2_on();
		FOC_slow_loop ();
		led2_off();

		// get the parametters for PID, from the bluetooth
		int objects_readed;
		float number;
		fflush(stdin); // needed to unblock scanf() after a not expected formatted data
		objects_readed = scanf("%s %f", &buffer, &value);

		if (objects_readed > 0)
		{
			switch (buffer[0])
			{
			case 'p':
				printf("p = %f\r\n", value);
				kp = value;
				break;

			case 'i':
				printf("i = %f\r\n", value);
				ki = value;
				break;

			case 'd':
				printf("d = %f\r\n", value);
				kd = value;
				break;

			case 'l':
				if (value == 1) log_enable = 1;
				else log_enable = 0;
				break;

			case 'L':
				if (-60 <= value && value <= 60){
					mr_delta_phase_angle_left = value;
				}
				printf("mr_...left = %d\r\n", mr_delta_phase_angle_left);
				break;

			case 'R':
				if (-60 <= value && value <= 60){
					mr_delta_phase_angle_right = value;
				}
				printf("mr_...right = %d\r\n", mr_delta_phase_angle_right);
				break;

			default:
				break;
			}
		}
	}
}

