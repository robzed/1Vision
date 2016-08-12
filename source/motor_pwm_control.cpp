/*
 *  motor_pwm_control.cpp
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

#include "motor_pwm_control.h"
#include "motor_pwm.h"
#include "basic_motor_primitives.h"
#include "tick_timer.h"
#include "pc_uart.h"

#if MOTOR_DEBUG_LOG
#include "motor.h"
#endif


namespace {

//	const int minimum_pwm_start_value = 300;	// on load minimum is 300
	const int minimum_pwm_start_value = 30;		// off load minimum for testing ... not sure what bottom value is
	const int scale_factor = 1024;
	const int scale_log2 = 10;

	const int minimum_current_value = minimum_pwm_start_value*scale_factor;
}

#if !MOTOR_DEBUG_LOG
complex_pwm_drive_control::complex_pwm_drive_control(int motor_number, int initial_full_sweep_rate_in_ms, int full_sweep_deceleration_in_ms)
: motor(motor_number), enabled(false), current_value(minimum_current_value)
#else
complex_pwm_drive_control::complex_pwm_drive_control(int motor_number, int initial_full_sweep_rate_in_ms, int full_sweep_deceleration_in_ms, Motor& m)
: motor(motor_number), enabled(false), current_value(minimum_current_value), motor_ref(m)
#endif
{
	register_timer_callback(this, TIMER_CALLBACK_TICKS_FOR_1600Hz);
	set_full_sweep_rate(initial_full_sweep_rate_in_ms, full_sweep_deceleration_in_ms);
}


complex_pwm_drive_control::~complex_pwm_drive_control()
{
	delete_timer_callback(this);
}


void complex_pwm_drive_control::stop_drive()
{
	enabled = false;
	set_single_pwm(motor, 0);
	current_value = minimum_current_value;
}

void complex_pwm_drive_control::disable_pwm()
{
	stop_drive();
	disable_single_pwm(motor);
}

bool complex_pwm_drive_control::is_idle()
{
	return !enabled;
}
//void complex_pwm_drive_control::motor_pause_acceleration(int motor)
//{
//}

//void complex_pwm_drive_control::increase_motor_drive(int motor)
//{
//}

//void complex_pwm_drive_control::decrease_motor_drive(int motor)
//{
//}

void complex_pwm_drive_control::set_direction(bool forward)
{
	
	// do we need to do this if we've set the direction before?
	// well, it saves storing it
	enabled = false;		// disable update routine
	DISABLE_MOTOR_1();		// make sure disabled ... probaly not required, but still
	set_single_pwm(motor, 0);	// disable pwm drive on this motor as well

	if(forward)
	{
		if(motor==1)
		{
			//write_string("f1");
			MOTOR_1_SELECT_FORWARD();
		}
		else
		{
			//write_string("f2");
			MOTOR_2_SELECT_FORWARD();
		}
	}
	else
	{
		if(motor==1)
		{
			//write_string("b1");
			MOTOR_1_SELECT_BACKWARD();
		}
		else
		{
			//write_string("b2");
			MOTOR_2_SELECT_BACKWARD();
		}
	}
}

void complex_pwm_drive_control::set_full_sweep_rate(int sweep_time_in_ms, int sweep_deceleration_in_ms)
{
	// our interrupt frequency is 1600Hz
	// the PWM period is 9830 Hz
	// the PWM value is 0 to 1000
	// to get from 0 to 1000 in steps of 1 at 1600Hz takes ... 625ms
	// faster requires larger steps
	// slow requires less updates
	
	// if the result = 2, we will finish in 312.5ms
	// if the result = 1, we will finish in 625ms
	// if the result = 0.5, we will finish in 1.25s
	// if the result = 0.25, we will finish in 2.5s
	// update_frequency = 1600, update_time = 1/1600 = 0.625us
	
	// total_time = 1000 * 0x625us / step_size =  1000 / (step_size * 1600)
	// step_size = 1000 / (total_time * 1600)
	// want to used fixed point numbers BUT should be single instruction divide
	// on interrupt ... maybe approx to 1024?
	//
	// if we want step size to be 1024x bigger	
	// step_size_fraction_of_1024*1024 = 1000*1024 / (total_time * 1600)
	// step_size_fraction_of_1024 = 640 / total_time
	//
	// if we want total time to be in ms...
	// step_size_fraction_of_1024 = 640,000 / total_time_in_ms
	// 
	update_increase_amount = 640000 / sweep_time_in_ms;
	//update_decrease_amount = -(update_increase_amount*2);	// twice as fast on decrease ... adhoc value for testing
	update_decrease_amount = -640000 / sweep_deceleration_in_ms;
	current_change_amount = update_increase_amount;
}

void complex_pwm_drive_control::do_ramp_up_from_stop()
{	
	set_single_pwm(motor, minimum_pwm_start_value);
	current_value = minimum_pwm_start_value*scale_factor;
	
	//if(update_increase_amount < 0)
	//{
	//	update_increase_amount = -update_increase_amount;
	//}
	current_change_amount = update_increase_amount;
	enabled = true;
}

void complex_pwm_drive_control::do_ramp_up_from_current()
{
	//if(update_increase_amount < 0)
	//{
	//	update_increase_amount = -update_increase_amount;
	//}
	current_change_amount = update_increase_amount;
	enabled = true;
}

void complex_pwm_drive_control::do_ramp_down_from_current()
{
	//if(update_increase_amount > 0)
	//{
	//	update_increase_amount = -update_increase_amount;
	//}
	current_change_amount = update_decrease_amount;
	enabled = true;
}


#define TINY_COMPLEX_PWM_DEBUG 0
unsigned int complex_pwm_drive_control::interrupt()
{	
	if(enabled)
	{
		const int max_drive_value = scale_factor*max_pwm;
		int output_value;
		current_value += current_change_amount;

		if(current_value >= max_drive_value)
		{
			output_value = max_pwm;
			current_value = max_drive_value;
			enabled = false;
		}
		else if(current_value <= minimum_current_value)		// could be minimum_pwm_start_value*scale_factor
		{
			current_value = minimum_current_value;
			output_value = 0;
			enabled = false;
		}
		else
		{
			output_value = current_value >> scale_log2;
		}
		set_single_pwm(motor, output_value);	
#if TINY_COMPLEX_PWM_DEBUG
		char c[2];
		c[1] = 0;
		c[0] = '!' + (output_value >> 4); // /2=512 /4=256 /8=128 /16=64
		write_string(c);
#endif

#if MOTOR_DEBUG_LOG
		motor_ref.log(output_value);
#endif
	}
	
	return TIMER_CALLBACK_TICKS_FOR_1600Hz;
}
