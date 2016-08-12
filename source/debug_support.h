/*
 *  debug_support.h
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


#ifndef DEBUG_SUPPORT_H
#define DEBUG_SUPPORT_H

// TO USE:
//
// Define DEBUGGING_ENABLED before the include of this file - 
// this then enables the functions.


//
// internal support functions
//
void step_worker(const char* desc);

// actual debug functions
inline void step(const char* desc)
{
#if DEBUGGING_ENABLED
	step_worker(desc);
#endif
}

// write statements that are enabled by DEBUGGING_ENABLED
#if DEBUGGING_ENABLED

#include "pc_uart.h"

#define dwrite_line write_line
#define dwrite_string write_string
#define dwrite_int(x) write_int(x)
#define dwrite_hex(x) write_hex(x)
#define dwrite_cr() write_cr()

#else

//#define dwrite_line(x)
#define dwrite_string(x)
#define dwrite_int(x)
#define dwrite_hex(x)
#define dwrite_cr()
inline void dwrite_line(const char* string, int num) { };
inline void dwrite_line(const char* string) { };
//inline void dwrite_string(const char* string, int num) { };
//inline void dwrite_string(const char* string) { };
#endif


#endif // DEBUG_SUPPORT_H
