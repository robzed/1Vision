/*
 *  speed_control.h
 *  robot_core
 *
 *  Created by Rob Probin on 05/02/2007.
 *  Copyright (C) 2007 Rob Probin.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "motor_sensing.h"
#include "motor.h"

#define USE_MOTOR_CLASS 0

//
// This class controls the speed by turning the motor on and off based
// on the time returned from the optical shaft encoder.
//
// Internally it binds to a specific motor_sensing interrupt.
//
class Speed_Control : Sensing_Callback {

public:
	void edge_happened(motortime_t timer_average, motortime_t last_timer_sample, int edge_count);
	void stop_speed_control();
	void start_speed_control(int mm_per_sec
#if !USE_MOTOR_CLASS
							 ,bool forward
#endif
	);
	void update_motor_speed_adjust();
#if USE_MOTOR_CLASS
	Speed_Control(int motor_to_control, Motor& motor_ref);
#else
	Speed_Control(int motor_to_control);
#endif
	~Speed_Control();
private:
	// speed control variables
	bool control_speed;
	motortime_t timer_target;		// not initially set to anything
	unsigned char motor_number;
	unsigned int update_speed_adjust_last_time;
	int last_pulse_count_seen;
#if !USE_MOTOR_CLASS
	bool go_forward;
#endif
//#define PWM_SLOW_SPEED_ENHANCEMENT
#ifdef PWM_SLOW_SPEED_ENHANCEMENT
	short pwm_speed;
#endif
#if USE_MOTOR_CLASS
	Motor& motor;
#endif
#if !USE_MOTOR_CLASS && MOTOR_DEBUG_LOG
	Logger logger1;
#endif
};




#endif

