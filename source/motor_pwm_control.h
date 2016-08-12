/*
 *  motor_pwm_control.h
 *  
 *
 *  Created by Rob Probin on 30/06/2009.
 *  Copyright (C) 2009 Rob Probin.
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

#ifndef MOTOR_PWM_CONTROL_H
#define MOTOR_PWM_CONTROL_H

#include "tick_timer.h"

#define MOTOR_DEBUG_LOG 0		// allows logging of stuff

#if MOTOR_DEBUG_LOG
class Motor;
#endif

class complex_pwm_drive_control : Timer_Callback {
public:
#if !MOTOR_DEBUG_LOG
	complex_pwm_drive_control(int motor_number, int initial_full_sweep_rate_in_ms, int full_sweep_deceleration_in_ms);
#else
	complex_pwm_drive_control(int motor_number, int initial_full_sweep_rate_in_ms, int full_sweep_deceleration_in_ms, Motor& m);
#endif
	~complex_pwm_drive_control();
	void do_ramp_up_from_stop();
	void do_ramp_up_from_current();
	void do_ramp_down_from_current();

	void set_direction(bool forward);
	void set_full_sweep_rate(int sweep_time_in_ms, int sweep_deceleration_in_ms);
	
	void stop_drive();		// should only be used if the robot is halted
	bool is_idle();
	void disable_pwm();		// should only be used if the robot is halted
	
	// timer callbacks
	unsigned int interrupt(); // call_at_1600Hz();
private:
	// internal data
	char motor;
	bool enabled;
	int update_increase_amount;
	int update_decrease_amount;
	int current_change_amount;
	int current_value;
	//short last_value;
	//int counts;
#if MOTOR_DEBUG_LOG
	Motor& motor_ref;
#endif
};


#endif 


