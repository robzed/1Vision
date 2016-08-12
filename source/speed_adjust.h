/*
 *  speed_adjust.h
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

#ifndef SPEED_ADJUST_H
#define SPEED_ADJUST_H

#include "speed_control.h"
#include "motor.h"

//class Speed_adjust {
//	// Responsibilities:
//	//   Avoid skidding by controlled acceleration
//	//   Avoid skidding by controlled deceleration
//	//
//	// We actually use a table rather than speed analysis
//	//
//public:
//	Speed_adjust(int table_number);
//	void update();
//	int get();
//	void start();
//	void stop();
//	void short_pause();
//private:
//		unsigned int last_timer;
//	int pwm_index;
//	int new_pwm;
//	int pwm_direction;
//	bool going;
//	int table_number;
//};

//#if 0
//// interface
//class Speed_adjust2 {
//public:
//	Speed_adjust2(int table_number);
//	virtual void boost_speed()=0;
//	virtual void reduce_speed()=0;
//	virtual bool is_top_speed()=0;
//	virtual ~Speed_adjust2()=0;
//	virtual int get()=0;
//};
//
//class Speed_adjust2_table_implementation : public Speed_adjust2{
//	// Responsibilities:
//	//   Avoid skidding by controlled acceleration
//	//   Avoid skidding by controlled deceleration
//	//
//	// We actually use a table rather than speed analysis
//	//
//public:
//	Speed_adjust2_table_implementation(int table_number);
//	void boost_speed();
//	void reduce_speed();
//	bool is_top_speed();
//	int get();
//private:
//	int pwm_index;
//	int current_value_copy;
//	bool top_speed;
//	const unsigned int *table_selected;
//};
//
//
//class Speed_adjust2_binary_implementation : public Speed_adjust2{
//	// Responsibilities:
//	//   Avoid skidding by controlled acceleration
//	//   Avoid skidding by controlled deceleration
//	//
//	// We actually use a table rather than speed analysis
//	//
//public:
//	Speed_adjust2_binary_implementation();
//	void boost_speed();
//	void reduce_speed();
//	bool is_top_speed();
//	int get();
//private:
//	bool running_speed;
//};
//
//#endif
//
//#define SPEED_ADJUST_WITH_PWM 0
//
//#if SPEED_ADJUST_WITH_PWM
//class Speed_adjust2 {
//	// Responsibilities:
//	//   Avoid skidding by controlled acceleration
//	//   Avoid skidding by controlled deceleration
//	//
//	// We actually use a table rather than speed analysis
//	//
//public:
//	Speed_adjust2(int table_number);
//	void boost_speed();
//	void reduce_speed();
//	bool is_top_speed();
//	int get();
//private:
//	int pwm_index;
//	int current_value_copy;
//	bool top_speed;
//	const unsigned int *table_selected;
//};
//
//#else



class Speed_adjust2 {
	// Responsibilities:
	//   Avoid skidding by controlled acceleration
	//   Avoid skidding by controlled deceleration
	//
	// We actually use a table rather than speed analysis
	//
public:
	Speed_adjust2(int motor_number);
	void boost_speed();
	void reduce_speed();
	bool is_top_speed();
	bool is_bottom_speed();
	void set_speed(int speed);
	void set_max_speed();
	void cleanup_after_stopped();
	
	// motor direction
	void set_direction_forward();
	void set_direction_backward();
	void update();
private:
	bool running_speed;		// basically two modes:
							// true = boosted (either full power or speed control)
							// false = reduced (stopped motors)
#if !USE_MOTOR_CLASS
	int motor;				// which motor this Speed adjust is bound to
#else
	Motor motor;
#endif
	int current_top_speed;	// speed to try to run at, except -1 = full forward.
	Speed_Control sc;		// speed controller for speeds under top speed
#if !USE_MOTOR_CLASS
	bool forward_direction;	// which way are we going? (can probably remove once we've moved over to the Motor class)
#endif
};
//#endif

#endif // SPEED_ADJUST_H

