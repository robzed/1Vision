/*
 *  tick_timer.h
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

#ifndef TICK_TIMER_H
#define TICK_TIMER_H

//typedef unsigned int timer_ticks_t;

void set_up_timer_interrupt();
unsigned int fetch_timer_ticks();

// returns a value in nanoseconds ... can use for accurate timing up 
// to 4 seconds. Accurate to +/- 17ns, if you take account of the actual run time
// of nanoseconds itself using time=nanoseconds()-nanoseconds(); and interrupts
// are disabled (or the measurement is made multiple times selecting the smallest).
//unsigned int nanoseconds();
unsigned int cpu_ticks();
void write_ticks();


// for ms timing accurate, this can help
unsigned int how_many_fast_ticks_per_second();
unsigned int fetch_fast_ticks();


class Timer_Callback {
public:
	virtual unsigned int interrupt() = 0; // pure virtual, should return number of ticks till next 
	virtual ~Timer_Callback() { }
};

#define TIMER_CALLBACK_TICKS_FOR_1600Hz 2

void register_timer_callback(Timer_Callback* callback_object, unsigned int ticks_till_call);
void delete_timer_callback(Timer_Callback* callback_object);

// for messing about with the tones of the buzzer
void alter_tick_timer_frequency(unsigned int frequency);
void alter_tick_timer_frequency_cHz(unsigned int frequency_in_cHz);
void default_tick_timer_frequency();

// for current limit stuff
//#define ENABLE_CURRENT_LIMIT_WARNING
bool is_current_limit_on();

#endif

