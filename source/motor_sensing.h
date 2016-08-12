/*
 *  motor_sensing.h
 *  
 *
 *  Created by Rob Probin on 09/09/2006.
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
 */

#ifndef MOTOR_SENSING_H
#define MOTOR_SENSING_H

typedef unsigned int motortime_t;
typedef unsigned int speed_t;

//
// Basic Functions
//
void set_up_motor_photo_interruptors();
// these are for debugging
void print_motor_data();
void switch_sensing_off();		// allows the subsystem to be turned off for readings to be taken (usually for debug)

//
// conversions
//
speed_t convert_to_speed(motortime_t timer_count);			// get a timer count and convert to speed
motortime_t convert_to_timer_count(speed_t speed_in_um_per_sec);	// useful if you need to know the timer count rather than converting to speed everytime.

// sensor related 
int convert_to_um(int ticks);
unsigned int convert_to_sensor_ticks(unsigned um);


//
// Interface class for edge calls
//
class Sensing_Callback {
public:
	virtual void edge_happened(motortime_t timer_average, motortime_t last_timer_sample, int edge_count) = 0; // pure virtual
	virtual ~Sensing_Callback() { }
};

//
// Class that actually stores data about motors
//

#define STORE_READINGS 0

typedef unsigned int timertick_t;

class Motor_input_data {
public:
	void pulse(timertick_t timer_val);
	Motor_input_data();
	void print();
	motortime_t return_average_time() { return timer_average; }
	int return_tick_count() { return edge_count; }
	void clear_count() { edge_count = 0; }
	Sensing_Callback* set_edge_reading(Sensing_Callback* new_routine);		// sets new routine, returns old routine (for chaining)
	
private:
	int edge_count;
#if STORE_READINGS
	static const int number_of_readings = 8;
	timertick_t timer_readings[number_of_readings];
	int reading_index;
#endif
	timertick_t old_timer_value;
	motortime_t timer_average;	
	motortime_t old_timer_difference;
	Sensing_Callback* sense_ptr;
};


//
// How to get the sensing data for a specific motor
//
Motor_input_data* motor_sense(int motor_number);	// Return the motor sense object related to a specific motor


//
// Direct accessors via motor number (replace at some point)
//
motortime_t get_average_time(int motor_number);

// position measurements
int get_motor_input_count(int motor_number);		// number of clicks
void clear_motor_input_count(int motor_number);




#endif // MOTOR_SENSING_H
