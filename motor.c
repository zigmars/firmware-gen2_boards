/*
 * EGG Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106.
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "gpio.h"
#include "pwm.h"
#include "motor.h"
#include "qfplib-m3.h"
#include "math.h"
#include "main.h"

unsigned int motor_speed_erps = 0; // motor speed in electronic rotations per second
unsigned int PWM_cycles_per_SVM_TABLE_step = 0;
unsigned int PWM_cycles_counter = 0;
unsigned int interpolation_PWM_cycles_counter = 0;
unsigned int motor_rotor_position = 0; // in degrees
unsigned int motor_rotor_absolute_position = 0; // in degrees
unsigned int interpolation_counter = 0;

unsigned int adc_phase_a_current_offset;
unsigned int adc_phase_c_current_offset;

static unsigned int _direction = RIGHT;

static unsigned int svm_table_index_a;
static unsigned int svm_table_index_b;
static unsigned int svm_table_index_c;

struct Bldc_phase_state bldc_phase_state;

int duty_cycle = 0;

// Space Vector Modulation PWMs values, please read this blog message:
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
// Please see file: BLDC_SPWM_Lookup_tables.ods
#define SVM_TABLE_LEN 360
unsigned int svm_table [SVM_TABLE_LEN] =
{
    1847	,
    1894	,
    1941	,
    1988	,
    2035	,
    2082	,
    2128	,
    2175	,
    2222	,
    2268	,
    2315	,
    2361	,
    2407	,
    2453	,
    2498	,
    2544	,
    2589	,
    2634	,
    2678	,
    2723	,
    2767	,
    2811	,
    2854	,
    2897	,
    2940	,
    2983	,
    3025	,
    3067	,
    3108	,
    3149	,
    3163	,
    3175	,
    3188	,
    3200	,
    3212	,
    3223	,
    3234	,
    3244	,
    3254	,
    3264	,
    3273	,
    3282	,
    3290	,
    3298	,
    3305	,
    3312	,
    3318	,
    3324	,
    3329	,
    3334	,
    3339	,
    3343	,
    3346	,
    3349	,
    3352	,
    3354	,
    3356	,
    3357	,
    3358	,
    3358	,
    3358	,
    3357	,
    3356	,
    3354	,
    3352	,
    3349	,
    3346	,
    3343	,
    3339	,
    3334	,
    3329	,
    3324	,
    3318	,
    3312	,
    3305	,
    3298	,
    3290	,
    3282	,
    3273	,
    3264	,
    3254	,
    3244	,
    3234	,
    3223	,
    3212	,
    3200	,
    3188	,
    3175	,
    3163	,
    3149	,
    3163	,
    3175	,
    3188	,
    3200	,
    3212	,
    3223	,
    3234	,
    3244	,
    3254	,
    3264	,
    3273	,
    3282	,
    3290	,
    3298	,
    3305	,
    3312	,
    3318	,
    3324	,
    3329	,
    3334	,
    3339	,
    3343	,
    3346	,
    3349	,
    3352	,
    3354	,
    3356	,
    3357	,
    3358	,
    3358	,
    3358	,
    3357	,
    3356	,
    3354	,
    3352	,
    3349	,
    3346	,
    3343	,
    3339	,
    3334	,
    3329	,
    3324	,
    3318	,
    3312	,
    3305	,
    3298	,
    3290	,
    3282	,
    3273	,
    3264	,
    3254	,
    3244	,
    3234	,
    3223	,
    3212	,
    3200	,
    3188	,
    3175	,
    3163	,
    3149	,
    3108	,
    3067	,
    3025	,
    2983	,
    2940	,
    2897	,
    2854	,
    2811	,
    2767	,
    2723	,
    2678	,
    2634	,
    2589	,
    2544	,
    2498	,
    2453	,
    2407	,
    2361	,
    2315	,
    2268	,
    2222	,
    2175	,
    2128	,
    2082	,
    2035	,
    1988	,
    1941	,
    1894	,
    1847	,
    1800	,
    1752	,
    1705	,
    1658	,
    1611	,
    1564	,
    1517	,
    1471	,
    1424	,
    1377	,
    1331	,
    1284	,
    1238	,
    1192	,
    1146	,
    1101	,
    1055	,
    1010	,
    965	,
    921	,
    876	,
    832	,
    788	,
    745	,
    702	,
    659	,
    616	,
    574	,
    532	,
    491	,
    450	,
    436	,
    424	,
    411	,
    399	,
    387	,
    376	,
    365	,
    355	,
    345	,
    335	,
    326	,
    317	,
    309	,
    301	,
    294	,
    287	,
    281	,
    275	,
    270	,
    265	,
    260	,
    256	,
    253	,
    250	,
    247	,
    245	,
    243	,
    242	,
    241	,
    241	,
    241	,
    242	,
    243	,
    245	,
    247	,
    250	,
    253	,
    256	,
    260	,
    265	,
    270	,
    275	,
    281	,
    287	,
    294	,
    301	,
    309	,
    317	,
    326	,
    335	,
    345	,
    355	,
    365	,
    376	,
    387	,
    399	,
    411	,
    424	,
    436	,
    450	,
    436	,
    424	,
    411	,
    399	,
    387	,
    376	,
    365	,
    355	,
    345	,
    335	,
    326	,
    317	,
    309	,
    301	,
    294	,
    287	,
    281	,
    275	,
    270	,
    265	,
    260	,
    256	,
    253	,
    250	,
    247	,
    245	,
    243	,
    242	,
    241	,
    241	,
    241	,
    242	,
    243	,
    245	,
    247	,
    250	,
    253	,
    256	,
    260	,
    265	,
    270	,
    275	,
    281	,
    287	,
    294	,
    301	,
    309	,
    317	,
    326	,
    335	,
    345	,
    355	,
    365	,
    376	,
    387	,
    399	,
    411	,
    424	,
    436	,
    450	,
    491	,
    532	,
    574	,
    616	,
    659	,
    702	,
    745	,
    788	,
    832	,
    876	,
    921	,
    965	,
    1010	,
    1055	,
    1101	,
    1146	,
    1192	,
    1238	,
    1284	,
    1331	,
    1377	,
    1424	,
    1471	,
    1517	,
    1564	,
    1611	,
    1658	,
    1705	,
    1752
};

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
    PWM_cycles_per_SVM_TABLE_step = PWM_CYCLES_COUNTER_MAX / SVM_TABLE_LEN;
  }

  // calculate the interpolation angle
  interpolation_PWM_cycles_counter++;
  if (interpolation_PWM_cycles_counter > PWM_cycles_per_SVM_TABLE_step)
  {
    interpolation_PWM_cycles_counter = 0;
    if (interpolation_counter <= 60) // limit max interpolation value/angle
    {
      interpolation_counter++;
      if (motor_rotor_position > 0)
      {
	motor_rotor_position--;
      }
      else
      {
	motor_rotor_position = 359;
      }
    }
    else
    {
      // keep this value static over the loops, when interpolation_position >= 60
      interpolation_PWM_cycles_counter = PWM_cycles_per_SVM_TABLE_step;
    }
  }

  apply_duty_cycle ();
}

// calc the DC offset value for the current ADCs
void motor_calc_current_dc_offset (void)
{
  unsigned int i = 0;
  while (i++ < 200)
  {
    adc_phase_a_current_offset += adc_get_phase_a_current_value ();
    adc_phase_c_current_offset += adc_get_phase_c_current_value ();
    delay_ms (10);
  }
  adc_phase_a_current_offset /= 200;
  adc_phase_c_current_offset /= 200;
}

void apply_duty_cycle (void)
{
  int duty_cycle_value = duty_cycle;
  int value = 0;

  // invert in the case of negative value
  if (duty_cycle_value < 0)
    duty_cycle_value *= -1;

  // scale and apply _duty_cycle
  value = svm_table[motor_rotor_position];
  if (value > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    value = (value - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + value;
  }
  else
  {
    value = (MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value;
  }
  set_pwm_phase_a (value);

  // add 120 degrees and limit
  unsigned int temp = motor_rotor_position + 120;
  unsigned int temp_motor_rotor_position;
  if (temp > 359)
  {
    temp_motor_rotor_position = (temp - 359) - 1;
  }
  else
  {
    temp_motor_rotor_position = temp;
  }
  value = svm_table[temp_motor_rotor_position];
  if (value > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    value = (value - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + value;
  }
  else
  {
    value = (MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value;
  }
  set_pwm_phase_b (value);

  // subtract 120 degrees and limit
  int temp1 = motor_rotor_position - 120;
  if (temp1 < 0)
  {
    temp_motor_rotor_position = (unsigned int) (360 + temp1); // temp is negative
  }
  else
  {
    temp_motor_rotor_position = (unsigned int) temp1;
  }
  value = svm_table[temp_motor_rotor_position];
  if (value > MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX)
  {
    value = (value - MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX + value;
  }
  else
  {
    value = (MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value) * duty_cycle_value;
    value = value / 1000;
    value = MIDDLE_PWM_VALUE_DUTY_CYCLE_MAX - value;
  }
  set_pwm_phase_c (value);
}

void commutate (void)
{
  #define HALL_SENSORS_MASK (HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN)

  static unsigned int hall_sensors = 0;
  unsigned int sector;

  hall_sensors = (GPIO_ReadInputData (HALL_SENSORS__PORT) & (HALL_SENSORS_MASK)); // mask other pins

  if (duty_cycle > 0)
    _direction = RIGHT;
  else
    _direction = LEFT;

_direction = LEFT;

  if (_direction == RIGHT)
  {
//    // the next sequence was obtained experimentaly
//    switch (hall_sensors)
//    {
//      case 8192:
//      svm_table_index_a = 28; // 1
//      svm_table_index_b = 4;
//      svm_table_index_c = 16;
//      motor_rotor_position = degrees_to_radiands(60);
//      break;
//
//      case 24576:
//      svm_table_index_a = 22; // 2
//      svm_table_index_b = 34;
//      svm_table_index_c = 10;
//      motor_rotor_position = degrees_to_radiands(120);
//      break;
//
//      case 16384:
//      svm_table_index_a = 16; // 3
//      svm_table_index_b = 28;
//      svm_table_index_c = 4;
//      motor_rotor_position = degrees_to_radiands(180);
//      break;
//
//      case 20480:
//      svm_table_index_a = 10; // 4
//      svm_table_index_b = 22;
//      svm_table_index_c = 34;
//      motor_rotor_position = degrees_to_radiands(240);
//      break;
//
//      case 4096:
//      svm_table_index_a = 4; // 5
//      svm_table_index_b = 16;
//      svm_table_index_c = 28;
//      motor_rotor_position = degrees_to_radiands(310);
//      break;
//
//      case 12288:
//      svm_table_index_a = 34; // 6
//      svm_table_index_b = 10;
//      svm_table_index_c = 22;
//      motor_rotor_position = degrees_to_radiands(0);
//
//      calc_motor_speed ();
//      break;
//
//      default:
//      return;
//      break;
//    }
  }
  else if (_direction == LEFT)
  {
    switch (hall_sensors)
    {
      case 8192:
      motor_rotor_absolute_position = 100; // 4
      break;

      case 24576:
      motor_rotor_absolute_position = 40; // 5
      break;

      case 16384:
      motor_rotor_absolute_position = 340; // 6
      break;

      case 20480:
      motor_rotor_absolute_position = 280; // 1
      break;

      case 4096:
      motor_rotor_absolute_position = 220; // 2
      break;

      case 12288:
      motor_rotor_absolute_position = 160; // 3

      motor_speed_erps = PWM_CYCLES_COUNTER_MAX / PWM_cycles_counter;
      PWM_cycles_per_SVM_TABLE_step = PWM_cycles_counter / SVM_TABLE_LEN;
      PWM_cycles_counter = 0;
      break;

      default:
      return;
      break;
    }

    motor_rotor_position = motor_rotor_absolute_position;
    interpolation_counter = 0;
    interpolation_PWM_cycles_counter = 0;
  }

    apply_duty_cycle ();
}

void commutate_timer (void)
{
  svm_table_index_dec ();

  apply_duty_cycle ();
}

void bldc_set_direction (unsigned int direction)
{
  _direction = direction;
}

unsigned int bldc_get_direction (void)
{
  return _direction;
}

void bldc_set_state (unsigned int state)
{
//  bldc_machine_state = state;
}

unsigned int bldc_get_state (void)
{
//  return bldc_machine_state;
}

// -999 < duty_cycle_value < 1000
void motor_set_duty_cycle (int duty_cycle_value)
{
  duty_cycle = duty_cycle_value;
}
