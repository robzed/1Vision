/*
 *  logger.cpp
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

#include "logger.h"
#include "pc_uart.h"
#include "timing.h"

Logger::Logger()
: dest(data), length(0)
{
}

void Logger::log(Logger_data_t value)
{
	if(length == size)
	{
		if(dest >= (data+size))
		{
			dest = data;
		}
	}
	else	// not at max yet
	{
		length++;
	}

	*dest = value;
	dest++;
}

void Logger::print()
{
	Logger_data_t* pstart;
	if(length == size)
	{
		// full means current location is oldest data
		pstart = dest;
	}
	else // less than full would mean oldest data is at beginning of array
	{
		pstart = data;
	}
	
	// now we loop from the oldest to the newest
	write_line("length,", length);
	int len = length;
	while(len)
	{
		if(pstart >= (data+size))
		{
			// reset to start if at end of array
			pstart = data;
		}
		write_int(length-len); write_string(","); write_int(*pstart); write_cr();
		//if((len & 0x02) == 0)
		//{
			delay(10);
		//}
		len--;
		pstart++;
	}
}

