/*
 *  speed_adjust.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 26/11/2006.
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

#include "speed_adjust.h"
//#include "tick_timer.h"
//#include "motor_sensing.h"
#include "pc_uart.h"
#include "motor.h"

#if !USE_MOTOR_CLASS
#include "basic_motor_primitives.h"
#endif


//#if SPEED_ADJUST_WITH_PWM
//
////
//// SPEED ADJUST
////
//
//// NOTES - these tables are set for the specific motors we have. 
////
//// We probably can increase the acceleration.
////
//const unsigned int pwm_table1[] =	// tweeked for motor 1
//{
//	//	pwm value
//	//
//	0,
//	400,
//	500,
//	600,
//	700,
//	800,
//	900,
//	1000,
//	//	250,
//	//	350,
//	//	450,
//	//	600,
//	//	700,
//	//	800,
//	//	900,
//	9999
//};
//const unsigned int pwm_table2[] = // tweeked for motor 2
//{
//	//	pwm value
//	//
//	0,
//	400,
//	500,
//	600,
//	700,
//	800,
//	900,
//	1000,
//	9999
//};
//
//
//
//Speed_adjust2::Speed_adjust2(int table_number)
//{
//	if(table_number == 1)
//	{
//		table_selected = pwm_table1;
//	}
//	else
//	{
//		table_selected = pwm_table2;
//	}
//	
//	top_speed = false;
//	//bottom_speed = true;
//	pwm_index = 0;
//	current_value_copy = table_selected[pwm_index];
//}
//
//void Speed_adjust2::boost_speed()
//{
////	write_int(pwm_index);
//	current_value_copy += 25;
//	if(current_value_copy >= 1000)
//	{
//		current_value_copy = 1000;
//		top_speed = true;
//	}
//
//#if 0
//	if(!top_speed)
//	{
//		pwm_index++;
//		current_value_copy = table_selected[pwm_index];
//		if(current_value_copy == 9999)
//		{
//			pwm_index--;
//			current_value_copy = table_selected[pwm_index];
//			top_speed = true;
//		}
//		bottom_speed = false;
//	}
//#endif
//}
//
//void Speed_adjust2::reduce_speed()
//{
////	write_int(pwm_index);
//	current_value_copy -= 25;
//	if(current_value_copy < 0)
//	{
//		current_value_copy = 0;
//	}
//	top_speed = false;
//	
//#if 0
//	if(pwm_index != 0)
//	{
//		pwm_index--;
//		current_value_copy = table_selected[pwm_index];
//		top_speed = false;
//	}
//#endif
//}
//
//
//bool Speed_adjust2::is_top_speed()
//{
//	return top_speed;
//}
//
//
//int Speed_adjust2::get()
//{
//	return current_value_copy;
//}
//
//#else // binary speed adjust ... sort of


//
// old junk :-)
//
//struct Master_Speed_Table_t
//{
//	bool top_speed;
//	bool bottom_speed;
//	int speed_value;
//	int motor_disable;
//};
//
//Master_Speed_Table_t Master_Speed_Table = {
//	{ false, true, -1, true },
//	{ false, true, 200, false },
//	{ false, true, 400, false },
//	{ false, true, 600, false },
//	{ false, true, 800, false },
//	{ false, true, -1, false },
//};

Speed_adjust2::Speed_adjust2(int motor_number)
: 
running_speed(false),
motor(motor_number), 
current_top_speed(-1),
#if USE_MOTOR_CLASS
sc(motor_number, motor)
#else
sc(motor_number),
forward_direction(true)
#endif
{
//	motor = motor_number;
//	running_speed = false;
}


void Speed_adjust2::set_speed(int speed)
{
	if(current_top_speed != speed)
	{
		current_top_speed = speed;
		if(running_speed)	// only turn motor to speed control if not reduced
		{
			//write_string("(start1="); write_int(speed); write_string(")");
#if !USE_MOTOR_CLASS
			sc.start_speed_control(speed, forward_direction);
#else
			sc.start_speed_control(speed);
#endif
		}
	}
}

void Speed_adjust2::set_max_speed()
{
	if(current_top_speed != -1)
	{
		current_top_speed = -1;
		if(running_speed)			// only turn motor to full if not reduced
		{
			//write_string("(stop1)");
			sc.stop_speed_control();
			// start motor after stopping speed controller. 
			// We can't touch motor whilst speed controller operating
#if !USE_MOTOR_CLASS
			motor_full(motor, forward_direction);
#else
			motor.speed_up();
#endif
		}
	}
}

void Speed_adjust2::cleanup_after_stopped()
{
	sc.stop_speed_control();
#if !USE_MOTOR_CLASS
	motor_remove_power(motor);
#else
	motor.slow_down();				// probably don't need this
	motor.inform_stopped();
	motor.stop_drive();
#endif
	running_speed = false;
	current_top_speed = -1;
}

void Speed_adjust2::boost_speed()
{
	if(running_speed) return;
	// otherwise we were not running
	running_speed = true;
	//write_string(" +");
	//write_int(motor);
	
	
	if(current_top_speed != -1)
	{
		// start motor before speed controller. We can't touch motor whilst speed controller operating
		//motor_full(motor, forward_direction);		// *** THIS LINE BREAKS SPEED CONTROL ***

		// NOTE: doesn't handle start from halted ... because the line above breaks speed control.
		// Shouldn't be a problem however since we only use this for deccelerating...
#if !USE_MOTOR_CLASS
		sc.start_speed_control(current_top_speed, forward_direction);
#else
		sc.start_speed_control(current_top_speed);
#endif
		//write_string("c");
	}
	else
	{
		sc.stop_speed_control();
		// start motor after stopping speed controller. We can't touch motor whilst speed controller operating
#if !USE_MOTOR_CLASS
		motor_full(motor, forward_direction);
#else
		motor.speed_up();
#endif
		//write_string("t");
	}
	//write_string("(sp="); write_int(current_top_speed); write_string(")");
}

void Speed_adjust2::reduce_speed()
{
	if(!running_speed) return;
	// otherwise we were running
	running_speed = false;
	//write_string(" -");
	//write_int(motor);
	
	// when we are trying to reduce speed ... i.e. stop the motor, don't control 
	// the speed control
	sc.stop_speed_control();
	// stop motor after speed controller. We can't touch motor whilst speed controller operating
#if !USE_MOTOR_CLASS
	motor_remove_power(motor);
#else
	motor.slow_down();
#endif
}

bool Speed_adjust2::is_top_speed()
{
	return running_speed;
}

bool Speed_adjust2::is_bottom_speed()
{
	return !running_speed;
}

void Speed_adjust2::set_direction_forward()
{
#if USE_MOTOR_CLASS	
	motor.set_direction(true);
#else
	forward_direction = true;
#endif
}

void Speed_adjust2::set_direction_backward()
{
#if USE_MOTOR_CLASS	
	motor.set_direction(false);
#else
	forward_direction = false;
#endif
}

void Speed_adjust2::update()
{
	sc.update_motor_speed_adjust();
}



//#endif



//#if 0
////
//// OLD STUFF
////
//
//int Speed_adjust::get()
//{
//	return new_pwm;
//}
//
//Speed_adjust::Speed_adjust(int num)
//{
//	last_timer = fetch_timer_ticks();
//	pwm_index = 2;
//	new_pwm = 0;
//	pwm_direction = 1;
//	going = false;
//	table_number = num;
//	
//}
//
//void Speed_adjust::start()
//{
//	going = true;
//	pwm_direction = 1;
//}
//
//void Speed_adjust::stop()
//{
//	going = true;
//	pwm_direction = -1;
//}
//
//void Speed_adjust::short_pause()
//{
//	pwm_direction = 1;
//	pwm_index = 1;
//	new_pwm = 0;
//}
//
//void Speed_adjust::update()
//{
//	if(!going) return;
//	
//	unsigned int current_ticks = fetch_timer_ticks();
//	if(current_ticks >= last_timer+5)	// 50ms steps
//	{
//		const unsigned int* pwmt_ptr;
//		if(table_number==1)
//		{
//			pwmt_ptr = pwm_table1;
//		}
//		else
//		{
//			pwmt_ptr = pwm_table2;
//		}
//		
//		new_pwm = pwmt_ptr[pwm_index];
//		pwm_index+=pwm_direction;
//		if(pwmt_ptr[pwm_index] == 9999)
//		{
//			pwm_index-=pwm_direction;
//		}
//		last_timer = current_ticks;
//	}
//}
//
//
//#endif
//


//#if 0
//
//#warning "motortime_t and speed_t need to be signed for this to work as before"
//static motortime_t target1 = convert_to_timer_count(500000);
//
//static int current_pwm1 = 1000;
//static int current_pwm2 = 1000;
//
//
//void speed_adjustment()
//{
//	motortime_t mt1, mt2;
//	speed_t s1, s2;
//	
//	
//	mt1 = get_average_time(1);
//	mt2 = get_average_time(2);
//	
//	const int Ks = 6;
//	int error = (mt1-target1) >> Ks;
//	current_pwm1 += error;
//	if(current_pwm1 > 1000) { current_pwm1 = 1000; }
//	if(current_pwm1 < 0) { current_pwm1 = 0; }
//	
//#if 0
//	if(mt1 > target1)
//	{
//		//   actual time > target time
//		// means
//		//   actual speed < target speed
//		// therefore
//		//   increase PWM
//		if(current_pwm1 < 1000) { current_pwm1 += 50; }
//	}
//	else if(mt1 < target1)
//	{
//		//   actual time < target time
//		// means
//		//   actual speed > target speed
//		// therefore
//		//   decrease PWM
//		if(current_pwm1 > 0) { current_pwm1 -= 50; }
//	}
//#endif	
//	
//	adjust_pwm(current_pwm1, current_pwm2);
//	
//	//s1 = convert_to_speed(mt1);
//	//s2 = convert_to_speed(mt2);
//	
//	write_int(current_pwm1); write_string(","); write_int(target1);
//	write_string(" = "); write_int(error); write_string(" = ");
//	/*write_int(s1); write_string(" ");*/ write_int(mt1); write_cr();
//}
//#endif
//
//
