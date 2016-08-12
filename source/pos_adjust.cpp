/*
 *  pos_adjust.cpp
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

#include "pos_adjust.h"
#include "motor_sensing.h"
#include "motor.h"
#include "pc_uart.h"
#include "language_support.h"

//
//
//
#define DEBUGGING_ENABLED 0
#include "debug_support.h"


void Pos_adjust2::move_straight_with_stop(int distance_in_um)
{
	rotation_flag = NO_ROTATION;
	if(distance_in_um > 0)
	{
		baseline1 = convert_to_um(get_motor_input_count(1));
		baseline2 = convert_to_um(get_motor_input_count(2));
		
		target_distance1_from_baseline = distance_in_um;
		target_distance2_from_baseline = distance_in_um;
#if 0
		write_line("baseline");
		write_int(baseline1); write_cr();
		write_int(baseline2); write_cr();
		write_line("target dist from baseline");
		write_int(target_distance1_from_baseline); write_cr();
		write_int(target_distance2_from_baseline); write_cr();
		write_line("start");
#endif
		// calculate how long you need to stop
//		if(distance_in_um > 120000)		// more than this distance we need a fixed distance to slow down
//		{
//			slowdown_distance = 100000;
//		}
//		else	// less than the distance it's approximately half
//		{
//			slowdown_distance = distance_in_um/2;
//		}
		
		// we now are not finished
		finished_flag = false;
		
		s1.set_direction_forward();
		s2.set_direction_forward();
		mode.change(Mode_t::MOVE_STRAIGHT_STOP);
		reduced_state = '-';
	}
}

const int width_between_wheels_in_um = 75000;		// center of wheel to center of wheel = 75mm
#define pi 3.141592654
const int turn_on_spot_circumference_in_um = static_cast<int>(width_between_wheels_in_um*pi); // Pi * D

void Pos_adjust2::turn_on_spot_from_to_stop(int angle_in_degrees)
{
	// angle in degrees ... -ve = clockwise, +ve = anticlockwise
	int distance_in_um = (angle_in_degrees * turn_on_spot_circumference_in_um) / 360;

	bool anticlockwise = false;
	if(distance_in_um > 0)
	{
		anticlockwise = true;
	}
	else
	{
		distance_in_um = -distance_in_um;
	}

	if(distance_in_um > 0)
	{
		baseline1 = convert_to_um(get_motor_input_count(1));
		baseline2 = convert_to_um(get_motor_input_count(2));

		target_distance1_from_baseline = distance_in_um;
		target_distance2_from_baseline = distance_in_um;
		
		// we now are not finished
		finished_flag = false;
		
// motor 1 = right side (starboard)
// motor 2 = left side (port)
		if(anticlockwise)
		{
			rotation_flag = ANTICLOCKWISE;
			s1.set_direction_forward();
			s2.set_direction_backward();
		}
		else
		{
			rotation_flag = CLOCKWISE;
			s1.set_direction_backward();
			s2.set_direction_forward();
		}

		if(distance_in_um > 20000)	// if greater than 2 cm away do normal deceleration
		{
			mode.change(Mode_t::MOVE_STRAIGHT_STOP);
		}
		else
		{
			mode.change(Mode_t::MOVE_STRAIGHT_STOP_SMALL_DISTANCE);
		}
		reduced_state = '-';
	}
}

void Pos_adjust2::move_straight_with_no_stop(int distance_in_um)
{
}

bool Pos_adjust2::is_finished()
{
	return finished_flag;
}

void Pos_adjust2::get_current_distance_from_baseline(int &distance_um_out, int &rotation_degrees_out)
{
	int raw_distance_motor1 = convert_to_um(get_motor_input_count(1));
	int raw_distance_motor2 = convert_to_um(get_motor_input_count(2));
	int distance_from_baseline_motor1 = raw_distance_motor1 - baseline1;
	int distance_from_baseline_motor2 = raw_distance_motor2 - baseline2;
	
	//
	// ROTATION
	//
	if(rotation_flag == ANTICLOCKWISE)
	{
		int difference_in_distances_from_baseline = (distance_from_baseline_motor1+distance_from_baseline_motor2)/2;
		
		int angle_in_degrees = (difference_in_distances_from_baseline * 360) / turn_on_spot_circumference_in_um;
		// +ve = anticlockwise, -ve = clockwise
		rotation_degrees_out = angle_in_degrees;
	}
	else if(rotation_flag == CLOCKWISE)
	{
		int difference_in_distances_from_baseline = (distance_from_baseline_motor1+distance_from_baseline_motor2)/2;
		
		int angle_in_degrees = (difference_in_distances_from_baseline * 360) / turn_on_spot_circumference_in_um;
		// +ve = anticlockwise, -ve = clockwise
		rotation_degrees_out = -angle_in_degrees;
	}
	else // if(rotation_flag == NO_ROTATION)
	{
		// we assume the difference is rotation on the spot. This is probably not 
		// accurate, but a first guess approximation (for testing).
		int difference_in_distances_from_baseline = (distance_from_baseline_motor1-distance_from_baseline_motor2)/2;
		
		int angle_in_degrees = (difference_in_distances_from_baseline * 360) / turn_on_spot_circumference_in_um;
		// +ve = anticlockwise, -ve = clockwise
		rotation_degrees_out = angle_in_degrees;
	}
			
	//
	// DISTANCE
	//
	// required for ensuring we go in a straight line
	if(rotation_flag == CLOCKWISE)
	{
		// motor 1 is backwards
		distance_from_baseline_motor1 = -distance_from_baseline_motor1;
	}
	else if(rotation_flag == ANTICLOCKWISE)
	{
		// motor 2 is backwards
		distance_from_baseline_motor2 = -distance_from_baseline_motor2;		
	}
	
	int average_distance_from_baseline = (distance_from_baseline_motor1 + distance_from_baseline_motor2) / 2;
	
	// for the distance travelled we use the average distance from baseline
	// should this be minimum distance?
	distance_um_out = average_distance_from_baseline;
	
}

void Pos_adjust2::update()
{
	switch(mode.get())
	{
		case Mode_t::INACTIVE:
		default:
			// do nothing
			break;
			
		case Mode_t::MOVE_STRAIGHT_STOP:
			update_move_straight_stop(false);
			break;
		
		case Mode_t::MOVE_STRAIGHT_STOP_SMALL_DISTANCE:
			update_move_straight_stop(true);
			break;
			
		case Mode_t::MOVE_STRAIGHT_GO:
			// not written yet
			halt("no move straight go");
			break;
	}
	
	s1.update();
	s2.update();
}


void Pos_adjust2::update_move_straight_stop(bool short_distance)
{
	// To go in a straight line...
	// need to keep ticks synchronised
	// three speed modes of operation
	// 1. motor ticks 1 > motor ticks 2
	// 2. motor ticks 1 = motor ticks 2
	// 3. motor ticks 1 < motor ticks 2
	//
	// Meeting the target.
	// Both motors need to be involved - otherwise we will stop going in a straight line
	// We have four options:
	// 1. stop if single motor distance (+ slow down time) >  target (get a random under/overshoot)
	// 2. stop if smallest motor distance > target (we will overshoot)
	// 3. stop if largest motor distance > target (we will undershoot)
	// 4. stop if (smallest motor distance + largest motor distance) / 2 > target (closest to target?)
	//
	// Probably option 3 or 4 are the best.
	// 
	
	// in case we need to use these figures twice
	int raw_distance_motor1 = convert_to_um(get_motor_input_count(1));
	int raw_distance_motor2 = convert_to_um(get_motor_input_count(2));
	
	// required for ensuring we go in a straight line
	int distance_from_baseline_motor1 = raw_distance_motor1 - baseline1;
	int distance_from_baseline_motor2 = raw_distance_motor2 - baseline2;
	
	// required for deciding to stop
	int distance_from_target1 = target_distance1_from_baseline - distance_from_baseline_motor1;
	int distance_from_target2 = target_distance2_from_baseline - distance_from_baseline_motor2;
	
	int average_distance_from_target = (distance_from_target1 + distance_from_target2) / 2;
	//write_string(" ", average_distance_from_target);
	//bool accelerating = 0;
	//bool decelerating = 0;

	//
	// we are either trying to go as fast as possible or slowing down to stop
	//
	//write_int(distance_from_target1);
	//write_cr();
	//write_int(distance_from_target2);
	//write_cr();
	//write_int(average_distance_from_target);
	//{
	//	static int last;
	//	if(last != average_distance_from_target)
	//	{
	//		write_int(average_distance_from_target); write_string(" ");
	//		last = average_distance_from_target;
	//	}
	//}
#if !USE_MOTOR_CLASS
	bool stopping = 0;
	if(average_distance_from_target > 100000)	// 10cm
	{
		// creates more wheel spin if left at maximum speed. 500mm/s works quite well.
		//s1.set_max_speed();
		//s2.set_max_speed();
		s1.set_speed(500);
		s2.set_speed(500);
		
		if(reduced_state != 'A')
		{
			write_line("10=", average_distance_from_target);		
			reduced_state = 'A';
		}
	}/*
	else if(average_distance_from_target > 50000) // 5cm
	{
		s1.set_speed(500);
		s2.set_speed(500);
		write_string("5");
	}
	else if(average_distance_from_target > 20000) // 2cm
	{
		s1.set_speed(400);
		s2.set_speed(400);
		write_string("2");
	}*/
	else if(average_distance_from_target > 40000) // 4cm
	{
		s1.set_speed(300);
		s2.set_speed(300);
		if(reduced_state != '4')
		{
			write_line("4=", average_distance_from_target);
			reduced_state = '4';
		}
	}
//	else if(average_distance_from_target > 5000) // 5mm
	else if(average_distance_from_target > 10000) // 1cm
	{
		s1.set_speed(200);
		s2.set_speed(200);
		if(reduced_state != '1')
		{
			write_line("1=", average_distance_from_target);
			reduced_state = '1';
		}
	}
	else // less than 4mm ... please stop!
	{
		if(short_distance && average_distance_from_target > 0)
		{
			s1.set_speed(200);
			s2.set_speed(200);
		}
		else
		{
			stopping = 1;
			if(reduced_state != 'S')
			{
				write_line("S=", average_distance_from_target);
				reduced_state = 'S';
			}
		}
	}
#else
	bool stopping = 0;
	if(average_distance_from_target > 150000)	// 15cm
	{
		s1.set_speed(500);
		s2.set_speed(500);		
	}
	else if(average_distance_from_target > 40000) // 4cm
	{
		s1.set_speed(300);
		s2.set_speed(300);
	}
	else if(average_distance_from_target > 10000) // 1cm
	{
		s1.set_speed(200);
		s2.set_speed(200);
	}
	else // less than 4mm ... please stop!
	{
		if(short_distance && average_distance_from_target > 0)
		{
			s1.set_speed(200);
			s2.set_speed(200);
		}
		else
		{
			stopping = 1;
		}
	}
#endif
	
	
#if 0 
	//if(average_distance_from_target <= slowdown_distance)
	{
		// we need to start slowing down
		decelerating = 1;
		//step("decelerating");
		//write_line("-");
	}
	else
	{
		accelerating = 1;
		//step("accelerating");
		//write_line("+");
	}
#endif

	//
	// Now we deal with going in a straight line
	//
//	if(accelerating)
	if(!stopping)
	{
#define ENABLE_ONGOING_POSITION_CONTROL 0
#if ENABLE_ONGOING_POSITION_CONTROL
		if(distance_from_baseline_motor1 > distance_from_baseline_motor2)	// motor 1 has travelled further
		{
			// boost speed of motor 2. If at 100% then reduce speed of motor 1.
			if(!s2.is_top_speed())
			{
				s2.boost_speed();
				step("s2 boost");
			}
			else
			{
				s1.reduce_speed();
				step("s1 reduce (s2=t)");
			}
		}
		else if(distance_from_baseline_motor1 < distance_from_baseline_motor2)	// motor 2 has travelled further
		{
			// boost speed of motor 1. If at 100% then reduce speed of motor 2.
			if(!s1.is_top_speed())
			{
				s1.boost_speed();
				step("s1 boost");
			}
			else
			{
				s2.reduce_speed();
				step("s2 reduce (s1=t)");
			}
		}
		else // ticks1 = ticks2
#endif
		{
			// keep doing what you are doing.
			// increase speed of both motors.
			if(!s1.is_top_speed() || !s2.is_top_speed())
			{
				s1.boost_speed();
				s2.boost_speed();
				step("s1+s2 boost");
			}
		}
	}
//	else if(decelerating)
	else
	{
#if ENABLE_ONGOING_POSITION_CONTROL
		if(distance_from_baseline_motor1 > distance_from_baseline_motor2)		// motor 1 has travelled further than motor 2
		{
			// Reduce speed of motor 1.
			s1.reduce_speed();
			step("s1 reduce");
		}
		else if(distance_from_baseline_motor1 < distance_from_baseline_motor2)	// motor 2 has travelled further than motor 1
		{
			// Reduce speed of motor 2.
			s2.reduce_speed();
			step("s2 reduce");
		}
		else // ticks1 = ticks2
#endif
		{
			// keep doing what you are doing.
			// Decrease speed of both motors.
			s1.reduce_speed();
			s2.reduce_speed();
			step("s1+s2 reduce");
		}
				
		// have we finished?
		if(s1.is_bottom_speed() && s2.is_bottom_speed() && robot_stopped())
		{
			// when we have stopped, ensure the speed controller isn't trying to 
			// do anything probably not needed...
			s1.cleanup_after_stopped();
			s2.cleanup_after_stopped();
			finished_flag = true;
			mode.change(Mode_t::INACTIVE);  //mode = INACTIVE;
#if 1
			write_string("finished, dist from baseline: m1=");
			write_int(distance_from_baseline_motor1); 
			write_line(" m2=", distance_from_baseline_motor2); 
			//write_line("distance from target:");
			//write_int(distance_from_target1); write_cr();
			//write_int(distance_from_target2); write_cr();
			//write_line("finished");
#endif
		}
	}
	
	// to go in a curve ...
	// need to keep difference increase per time slice constant
	//
}

void Mode_t::change(mode_t new_mode)
{
	//write_string("Mode: ");
	//write_int(new_mode);
	//write_cr();
	mode_internal = new_mode;
}

Mode_t::Mode_t(mode_t start_mode)
{
	mode_internal = start_mode;
}


Mode_t::mode_t Mode_t::get()
{
	return mode_internal;
}


//
// Constructor
//
Pos_adjust2::Pos_adjust2()
: s1(1), s2(2),
baseline1(0), baseline2(0), 
target_distance1_from_baseline(0), target_distance2_from_baseline(0),
//slowdown_distance(50000),	// 5cm = 50mm
finished_flag(true),		// we start off as finished
mode(Mode_t::INACTIVE)
{
}

