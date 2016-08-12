/*
 *  debug_support.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 21/03/2007.
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

#include "debug_support.h"
#include "pc_uart.h"

void step_worker(const char* desc)
{
	write_string(desc);
	write_string(" (((press s to step)))");
	int c;
	do {
		c = read_char();
	} while(c != 's');
	write_line(" - ok");
}

