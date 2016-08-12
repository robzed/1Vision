/*
 *  motor.cpp
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

#include "motor.h"
#include "basic_motor_primitives.h"

#if MOTOR_DEBUG_LOG 
#include "logger.h"
#endif

namespace { 
//	const int RAMP_UP_TIME_FROM_STOP_IN_ms = 750;
	const int RAMP_UP_TIME_FROM_STOP_IN_ms = 2000;
	const int RAMP_DOWN_TIME_IN_ms = 375;
}

//
// Code - Member function definitions
// 

Motor::Motor(int motor_number)
: 
#if ON_OFF_DRIVE
motor(motor_number),
forward_dir(true)
#elif RAMP_PWM_DRIVE
#if !MOTOR_DEBUG_LOG
drive(motor_number, RAMP_UP_TIME_FROM_STOP_IN_ms, RAMP_DOWN_TIME_IN_ms)
#else
drive(motor_number, RAMP_UP_TIME_FROM_STOP_IN_ms, RAMP_DOWN_TIME_IN_ms, *this)
#endif
#endif
{
#if MOTOR_DEBUG_LOG
	logger = new Logger;
#endif
}


#if MOTOR_DEBUG_LOG
Motor::~Motor()
{
	logger->print();
	delete logger;
}
#endif
#if MOTOR_DEBUG_LOG
void Motor::log(Logger_data_t data)
{
	logger->log(data);
}
#endif


// Code to slow a motor down
void Motor::slow_down()
{
#if ON_OFF_DRIVE
	motor_remove_power(motor);
#elif RAMP_PWM_DRIVE
	drive.do_ramp_down_from_current();
#endif
}

void Motor::speed_up()
{
#if ON_OFF_DRIVE
	motor_full(motor, forward_dir);
#elif RAMP_PWM_DRIVE
	drive.do_ramp_up_from_current();
#endif
}

void Motor::stop_drive()
{
#if RAMP_PWM_DRIVE
	drive.stop_drive();
#endif
}

void Motor::inform_stopped()
{
}
void Motor::inform_running()
{
}

bool Motor::motor_ramp_finished()
{
#if RAMP_PWM_DRIVE
	return drive.is_idle();
#endif
}


void Motor::set_direction(bool forward_direction)
{	
	//if(forward_dir != forward_direction && ( ! drive.is_idle() || ! robot_stopped() ))
	//{
	//	halt
	//}
#if ON_OFF_DRIVE
	forward_dir = forward_direction;
#elif RAMP_PWM_DRIVE	
	drive.set_direction(forward_direction);	
#endif
}


void Motor::unhook_pwm()
{
#if RAMP_PWM_DRIVE
	drive.disable_pwm();
#endif
}

#include "motor_sensing.h"

static Timer stopped_timer(0,40);		// 400ms
static int stopped_motor_count = 0;	// should only be zero if it's never moved

// for debug
//#include "pc_uart.h"
//#include "tick_timer.h"
// --end--
bool robot_stopped()
{
	//write_int(fetch_timer_ticks());
	// if motors not moving (input count can only ever go up)
	if(stopped_motor_count == get_motor_input_count(1) + get_motor_input_count(2))
	{
		// and stopped for 400ms
		if(stopped_timer.expired())
		{
			//write_string("+");
			return true;		// then we have stopped
		}
		//write_string("-");
		return false;
	}
	
	//write_string("n");
	// otherwise reset the stopped timer and the stopped count
	stopped_timer.set(0,40);		// 400ms
	stopped_motor_count = get_motor_input_count(1) + get_motor_input_count(2);
	return false;
}
