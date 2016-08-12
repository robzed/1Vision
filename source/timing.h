/*
 *  timing.h
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

#ifndef TIMING_H
#define TIMING_H

void delay();		// short non-specific time
void delay(unsigned int time_in_ms);


// +-------------------------------+-------------------------+-----------------------
// | TITLE:        very_short_delay
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 17th June 2006
// +
// | DESCRIPTION:  Short delay to avoid switching the ports too quickly
// +----------------------------------------------------------------ROUTINE HEADER----
inline void very_short_delay()
{
	volatile int x = 0;
	x++;
}




// this timer is good but only is accurate to 10ms (+/-10ms)
class Timer {
	
public:
	Timer();				// expired initially
	Timer(unsigned int seconds, unsigned int centiseconds);		// version that sets initially to some time in the future
	bool expired();
	void set(unsigned int seconds, unsigned int centiseconds);
private:
	unsigned int new_timer_ticks;
};


// similar timer, but accurate to 1ms (+/-2ms)
class msTimer {
	
public:
	msTimer();				// expired initially
	msTimer(unsigned int milliseconds);		// version that sets initially to some time in the future
	bool expired();
	void set(unsigned int milliseconds);
	void set_raw(unsigned int fast_ticks);
private:
	unsigned int new_timer_ticks;
};
#endif // TIMING_H


