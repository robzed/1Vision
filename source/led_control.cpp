/*
 *  led_control.cpp
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

#include "led_control.h"
#include "timing.h"

#include "lpc210x_gnuarm.h"
#include "robot_hardware_defs.h"

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        ledInit
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: old
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----

LED::LED(byte port_in)
: LEDport(port_in, true)
{
	// GPIO_IODIR |= (1<<port_in);	// define LED-Pin as output
								//	GPIO_IODIR &= ~(1<<SWPIN);	// define Switch-Pin as input
								//GPIO_IOCLR = (1<<ENABLE_LED);	// set Bit = off (active high)
	off();
}


//
// ******************************************************************
//

void LED::flash(int number_of_times)
{
	for(int i=0; i<number_of_times; i++)	
	{
		on();
		//GPIO_IOCLR=(1<<MOTOR_1_ENABLE_PIN);	// set all outputs in mask to 0
		delay(100);
		off();
		//GPIO_IOSET=(1<<MOTOR_1_ENABLE_PIN);	// set all outputs in mask to 1
		delay(100);
	}
}

//
// ******************************************************************
//

void LED::set(bool state)
{
	if(state)
	{
		on();
	}
	else
	{
		off();
	}
}

