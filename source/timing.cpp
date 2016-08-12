/*
 *  timing.cpp
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
 *
 */

#include "timing.h"
#include "tick_timer.h"

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        delay
// | AUTHOR(s):    Martin Thomas
// | DATE STARTED: old
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----

void delay()
{
	volatile int i,j;
	
	for (i=0;i<100;i++)
		for (j=0;j<1000;j++);
}


void delay(unsigned int time_in_ms)
{
	time_in_ms /= 10;
	unsigned int time_target = fetch_timer_ticks() + time_in_ms;
	while(time_target > fetch_timer_ticks())
	{
		// do nothing
	}
}

unsigned int convert_to_fast_tick_period(unsigned int ms)
{
	unsigned int count;
	// count = ms * how_many_per_second
	count = ms*how_many_fast_ticks_per_second();
	count /= 1000;
	// 1.6 per ms.
	return count;
}


inline unsigned int timer_seconds(unsigned int seconds)
{
	return seconds*100;
}

inline unsigned int timer_milliseconds(unsigned int milliseconds)
{
	return milliseconds/10;
}

inline unsigned int timer_centiseconds(unsigned int cs)
{
	return cs;
}

//
// Timer related functions
// 
Timer::Timer()
: new_timer_ticks(0)
{
}

Timer::Timer(unsigned int seconds, unsigned int centiseconds)
: new_timer_ticks(0)
{
	set(seconds, centiseconds);
}

//#include "pc_uart.h"
bool Timer::expired()
{
	//write_string("timer:");
	//write_int(fetch_timer_ticks());
	//write_string(" ");
	//write_int(new_timer_ticks);
	//write_cr();
	return fetch_timer_ticks() > new_timer_ticks;
}

void Timer::set(unsigned int seconds, unsigned int centiseconds)
{
	new_timer_ticks = fetch_timer_ticks() + timer_seconds(seconds) + timer_centiseconds(centiseconds);
}


//
// Timer related functions
// 
msTimer::msTimer()
: new_timer_ticks(0)
{
}

msTimer::msTimer(unsigned int milliseconds)
: new_timer_ticks(0)
{
	set(milliseconds);
}

//#include "pc_uart.h"
bool msTimer::expired()
{
	//write_string("timer:");
	//write_int(fetch_timer_ticks());
	//write_string(" ");
	//write_int(new_timer_ticks);
	//write_cr();
	return fetch_fast_ticks() > new_timer_ticks;
}

void msTimer::set(unsigned int milliseconds)
{
	new_timer_ticks = fetch_fast_ticks() + convert_to_fast_tick_period(milliseconds);
}

void msTimer::set_raw(unsigned int fast_ticks)
{
	new_timer_ticks = fetch_fast_ticks() + fast_ticks;
}


