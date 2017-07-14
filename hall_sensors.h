/*
 * Generic Electric Unicycle firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _HALL_SENSORS_H_
#define _HALL_SENSORS_H_

//#define HALL_SENSORS_INTERRUPT EXTI15_10_IRQHandler

void hall_sensor_init (void);
unsigned int get_hall_sensors_us (void);

#endif
