/*
 *  speed_control.cpp
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

#include "speed_control.h"
#include "tick_timer.h"
#ifdef PWM_SLOW_SPEED_ENHANCEMENT
#include "motor_pwm.h"
#endif

#if !USE_MOTOR_CLASS
#include "basic_motor_primitives.h"
#endif
#include "pc_uart.h"

#if USE_MOTOR_CLASS	
Speed_Control::Speed_Control(int motor_to_control, Motor& motor_ref)
#else
Speed_Control::Speed_Control(int motor_to_control)
#endif
: control_speed(false)
, motor_number(motor_to_control)
, update_speed_adjust_last_time(0)
#ifdef PWM_SLOW_SPEED_ENHANCEMENT
, pwm_speed(1000)
#endif
#if USE_MOTOR_CLASS
, motor(motor_ref)
#endif
{
	// find which motor to control
	Motor_input_data* i = motor_sense(motor_to_control);
	i->set_edge_reading(this);
	// throw away old edge reading value
}

//
// What happens when the speed control is destroyed?
//
Speed_Control::~Speed_Control()
{
	// erase me from the list
	// (not compatible with stacked edge_readings)
	Motor_input_data* i = motor_sense(motor_number);
	i->set_edge_reading(0);
#if !USE_MOTOR_CLASS && MOTOR_DEBUG_LOG
	logger1.print();
#endif
}

//
//
//
void Speed_Control::start_speed_control(int mm_per_sec
#if !USE_MOTOR_CLASS
										,bool forward
#endif
)
{
#if !USE_MOTOR_CLASS
	go_forward = forward;
#endif
#ifdef PWM_SLOW_SPEED_ENHANCEMENT
	if(mm_per_sec < 250) { pwm_speed = 700; }
	else { pwm_speed = 1000; }
#endif
	timer_target = convert_to_speed(mm_per_sec*1000);
	control_speed = true;
#ifdef PWM_SLOW_SPEED_ENHANCEMENT
	motor_full(motor_number, go_forward);
	enable_single_pwm(motor_number, pwm_speed);
#endif
}

//
//
//
void Speed_Control::stop_speed_control()
{
	control_speed = false;
	update_speed_adjust_last_time = 0;
#ifdef PWM_SLOW_SPEED_ENHANCEMENT
	disable_single_pwm(motor_number);
#endif
}


//
//
//
void Speed_Control::edge_happened(motortime_t timer_average, motortime_t last_timer_sample, int count)
{
	// control the speed based on timer average
	// notice we also need to take account of position adjustments to keep it going straight
	// cut this routine when motor is too far ahead and stop motor ... temporarily
#if MOTOR_DEBUG_LOG
	int up = -0x40000000;
#endif
	
	if(control_speed)
	{
		//write_string(".");
		//write_int(timer_average);
		//write_string(">=");
		//write_int(timer_target);
		if(timer_average >= timer_target)	// timer is longer (or equal) than target, therefore speed is slower (or equal) ... so speed up
		{
			// turn motor on
#ifndef PWM_SLOW_SPEED_ENHANCEMENT
#if !USE_MOTOR_CLASS
			motor_full(motor_number, go_forward);
#else
			// don't want to call this every time ... bad effects on pwm
			//motor.direction(go_forward);
			motor.speed_up();
#endif			
#else
			adjust_single_pwm(motor_number, pwm_speed);
#endif
		}
		else								// timer is smaller than target, therefore speed is faster ... so slow down
		{
			// turn motor off
#ifndef PWM_SLOW_SPEED_ENHANCEMENT
#if !USE_MOTOR_CLASS
			motor_remove_power(motor_number);
#else
			motor.slow_down();
#endif
#if MOTOR_DEBUG_LOG
			up = 0;
#endif
#else
			adjust_single_pwm(motor_number, 0);
#endif
		}
		
#if MOTOR_DEBUG_LOG
		
		if(timer_average < 1 || timer_average > 0x3fffffff)
		{
#if USE_MOTOR_CLASS
			motor.log((-1)+up);
#else
			logger1.log((-1)+up);
			logger1.log(fetch_fast_ticks());
#endif
		}
		else
		{
#if USE_MOTOR_CLASS
			motor.log((-timer_average)+up);
#else
			logger1.log((-timer_average)+up);
			logger1.log(fetch_fast_ticks());
#endif
		}
#endif		
		
	}
	
}


// Although this routine says it will check above 100mm/sec, because of fin 
// differences it might only be stable > 200mm/s
//
void Speed_Control::update_motor_speed_adjust()
{
	//	static int led_state = 0;
	
	if(control_speed)
	{
		int current_tick_count = motor_sense(motor_number)->return_tick_count();
		
		if(update_speed_adjust_last_time == 0)	// never called before
		{
			update_speed_adjust_last_time = fetch_timer_ticks();
			last_pulse_count_seen = current_tick_count;
		}
		else // check the motor is still running
		{
			if(update_speed_adjust_last_time!=fetch_timer_ticks())	// "if timer has changed"
			{
				update_speed_adjust_last_time = fetch_timer_ticks();
				// 10ms has elapsed
				// at 10ms we should get at least 1 tick if the speed is above 100mm/second
				// at lower speeds we should allow longer
				if(last_pulse_count_seen == current_tick_count)	// if the same might have stopped... for speeds > 100mm/s
				{
					//write_string("$");
					//					led(led_state);
					//					led_state = 1-led_state;
					
					// the idea is that for speeds above 100mm/sec if we stop then it will
					// break the speed controller above.
#ifndef PWM_SLOW_SPEED_ENHANCEMENT
#if !USE_MOTOR_CLASS
					motor_full(motor_number, go_forward);
#else
					// don't want to call this every time ... bad effects on pwm
					//motor.direction(go_forward);
					motor.speed_up();
#endif
#else
					adjust_single_pwm(motor_number, pwm_speed);
#endif
					
					// what do we do for speeds less than 100mm/sec?
					// one option is to wait for several timer ticks to elapse, e.g. 2 for 50mm/sec
					//
					// Alternatively, if we don't actually need less than 100mm/sec then
					// we can ignore this case. That's what we will do initially.
				}
				else // if it has changed, turn off the led and ensure next time led comes on
				{
					//					led(0);
					//					led_state = 1;
				}
				// ensure the next time we look we have the correct reference point
				last_pulse_count_seen = current_tick_count;
				
			} // end of "if timer has changed"
		}
	}
	else // this should really be init when control speed = 0;
	{
		update_speed_adjust_last_time = 0;
	}
}
