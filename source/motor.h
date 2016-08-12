/*
 *  motor.h
 *  robot_core
 *
 *  Created by Rob Probin on 12/12/2006.
 *  Copyright (C) 2006 Rob Probin.
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

#ifndef MOTOR_H
#define MOTOR_H

#include "timing.h"
#include "motor_pwm_control.h"

//
// Motor represents the high level control and data back from a motor.
// It is the motor plus optical encoder representation.
//
// It should be used in preference to lower level routines in:
// motor_pwm, motor_sensing, basic_motor_primitives.h, robot_hardware_defs and
// motor_pwm_control. What about SpeedControl?
//
//
// The job of this routine is to stop skipping due to acceleration, deceleration
// and also to make sure the worm drive doesn't lock (which happens if the 
// wheels are going faster than the motor.
//
//
// Select mode of operation
//
#define ON_OFF_DRIVE 0
#define RAMP_PWM_DRIVE 1

#if MOTOR_DEBUG_LOG
#include "logger.h"
#endif

class Motor {
	
public:
	Motor(int motor_number);	// must construct with 
#if MOTOR_DEBUG_LOG
	~Motor();
#endif
	
	void slow_down();
	void speed_up();

	void inform_stopped();
	void inform_running();
	
	void set_direction(bool forward_direction);
	
	bool motor_ramp_finished();
	void stop_drive();		// should only be used if the robot is halted
	void unhook_pwm();		// should only be used if the robot is halted
	
#if MOTOR_DEBUG_LOG
	void log(Logger_data_t data);
#endif
	
private:
#if RAMP_PWM_DRIVE
	complex_pwm_drive_control drive;
#elif ON_OFF_DRIVE
	int motor;
	bool forward_dir;
#endif
#if MOTOR_DEBUG_LOG
	Logger* logger;
#endif
};



bool robot_stopped();

#endif // MOTOR_H

