/*
 *  buzzer.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 21/07/2007.
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
 */

#include "buzzer.h"
#include "robot_hardware_defs.h"
#include "hardware_support.h"
#include "tick_timer.h"

Buzzer::Buzzer()
: beeptimer(),
timed_beeping(false)
{
}

void Buzzer::beep(unsigned int centiseconds)
{
	beeptimer.set(0,centiseconds);
	timed_beeping = true;
	on();
}

void Buzzer::service()
{
	if(timed_beeping and beeptimer.expired())
	{
		timed_beeping = false;
		off();
	}
}

bool Buzzer::finished()
{
	return !timed_beeping;
}

void Buzzer::on()
{
	T0_MR2 = T0_MR0;
	T0_EMR = 3<<8;		// toggle the output
	pin_function_select(MAIN_sound_output_port, 2);	// select the timer match output
}

void Buzzer::off()
{
	// change it back to an i/o pin
	pin_function_select(MAIN_sound_output_port, 0);	
}

struct semitone_type {
	unsigned int tone_cHz;
	const char* note_name;
};

semitone_type semitone[] = 
{
	{ 26163, "C" },
	{ 27718, "C#" },
	{ 29366, "D" },
	{ 31112, "D#" },
	{ 32963, "E" },
	{ 34923, "F" },
	{ 36999, "F#" },
	{ 39200, "G" },
	{ 41530, "G#" },
	{ 44000, "A" },
	{ 46616, "A#" },
	{ 49388, "B" },
};
#include "pc_uart.h"
void Buzzer::beep(unsigned int duration, int pitch)
{
	bool lower = false;
	int octive = 0;
	
	while(pitch < 0)
	{
		pitch = pitch+12;
		octive++;
		lower = true;
	}
	if(pitch >= 12)
	{
		octive = pitch / 12;
		pitch %= 12;
	}

	unsigned int freq = semitone[pitch].tone_cHz;
	if(lower)
	{
		freq >>= octive;
	}
	else
	{
		freq <<= octive;
	}
	
	// Notice that we've altered our time base. Normally it's 100Hz (0.01s)
	// but the time base is now 100*(frequency/1600) Hz
	// therefore if the frequency input is 3200Hz, the time base will be 200Hz (0.005s)
	// and if the frequency is 160Hz, the time base will be 10Hz (0.1s)
	// Alternatively the time multiplication factor is (frequency_in_Hz/1600)
	// 
	// This code compensates for that change.
	duration = (duration*freq)/160000;
	
	// actually set up the tone
	off();
	alter_tick_timer_frequency_cHz(freq);
	beep(duration);
	while(not finished())
	{
		service();
	}
	default_tick_timer_frequency();
	
}
