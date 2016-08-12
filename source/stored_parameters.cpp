/*
 *  stored_parameters.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 2/11/2008.
 *  Copyright (C) 2008 Rob Probin.
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

#include "stored_parameters.h"
#include "language_support.h"

#define DEBUGGING_ENABLED 1			// turn this on for read/write debugging
#include "debug_support.h"

// Problems:
// 1. This current system is difficult ot maintain - it takes too long to add
//    a new variable.
// 2. Are they recorded at a fixed address, or fixed order, or what? They 
//    should be maintainable on upgrade.
//
// PROBLEM 1
// =========
// Problem constraints:
// types - we shouldn't make it more difficult by knowing types (e.g. get_uint8 
// would be bad). Although they are all uint32?
// 
// Solutions:
//
// a. programming_fields.h type solution
//    data_thing(NAME_FOR_ENUM, data1, data2, data3)
//    included twice with different macros
//    http://www.drdobbs.com/cpp/184401387
//    This is the alternative header based X-Macros
//    Usually the X macros are in a file with the extension .def.
//		// File: color_table.h
//		X(red, "red")
//		X(green, "green")
//		X(blue, "blue")
//
//		// File: main.c
//		#include <stdio.h>
//
//		#define X(a, b) a,
//		enum COLOR {
//		#include "color_table.h"
//		};
//		#undef X
//
//		#define X(a, b) b,
//		char *color_name[] = {
//		#include "color_table.h"
//		};
//		#undef X
//
// See also http://en.wikipedia.org/wiki/C_preprocessor#X-Macros
//
// b. Classic X-Macros
//    http://www.drdobbs.com/cpp/184401387
//
//    #define COLOR_TABLE \  (line concatenation)
//    X(red, "red")       \  (line concatenation)
//    X(green, "green")   \  (line concatenation)
//    X(blue, "blue")
//
//    #define X(a, b) a,
//    enum COLOR {
//      COLOR_TABLE
//    };
//    #undef X
//
//    #define X(a, b) b,
//    char *color_name[] = {
//      COLOR_TABLE
//    };
//    #undef X
//
// c. Modified X-Macro
//    http://stackoverflow.com/questions/126277/making-something-both-a-c-identifier-and-a-string
//
// 
// For (a), (b) and (c) also see http://blog.brush.co.nz/2009/08/xmacros/
//
// d. Class based runtime solution?
//
// e. Class based compile-time solution?
//
// f. Seperate macro processor, e.g. M4 (see comment Cameron Kerr 26 Aug 2009, 
//    11:52 on http://blog.brush.co.nz/2009/08/xmacros/)
//
//
// Others
// http://www.reddit.com/r/programming/comments/9dqpy/dry_code_with_xmacros_a_littleknown_c_technique/c0cdp2s
// http://s11n.net/papers/supermacros_cpp.html
// http://s11n.net/papers/supermacros_cpp.pdf


const unsigned int eeprom_format_version_number = EEPROM_FORMAT_VERSION_NUMBER;

//
// These are the addresses of the eeprom locations
//
enum eeprom_address {
	version_number_addr,
	length_addr,
	
#define def_param(name, type, default) name##_addr,
#include "stored_parameters.def"
	
	checksum_addr,				// checksum should always be last one
	EEPROM_LENGTH
};

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 2 November 2008
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----

stored_parameters::stored_parameters()
{
	// constructor loads all data from eeprom
	load_parameters();
}


void stored_parameters::save_parameter(unsigned short address, unsigned int value, unsigned int& checksum)
{
	ee.write(address, value);
	checksum += value;
}

void stored_parameters::save_parameters()
{
	// could introduct intelligence at some point to see if the parameters had actually changed before saving!
	unsigned int checksum = 0;
	
	save_parameter(version_number_addr, eeprom_format_version_number, checksum);
	save_parameter(length_addr, EEPROM_LENGTH, checksum);
	
	//
	// save each of the parameters in turn
	//
#define def_param(name, type, default) save_parameter(name##_addr, name, checksum);
#include "stored_parameters.def"
	
	ee.write(checksum_addr, checksum);
}

unsigned int stored_parameters::load_parameter(unsigned short address, unsigned int& checksum)
{
	unsigned int data = ee.read(address);
	checksum += data;
	
	return data;
}

void stored_parameters::load_parameters()
{
	unsigned int checksum_calc = 0;

	unsigned int version_number = load_parameter(version_number_addr, checksum_calc);
	unsigned int length = load_parameter(length_addr, checksum_calc);

	//
	// load each of the parameters in turn
	//
#define def_param(name, type, default) name = load_parameter(name##_addr, checksum_calc);
#include "stored_parameters.def"

	//
	// now validate the loaded data
	//
	bool validation = true;
	if(checksum_calc != ee.read(checksum_addr))
	{
		dwrite_line("***Stored Parameters - checksum failed***");
		validation = false;
	}
	if(version_number != eeprom_format_version_number)
	{
		dwrite_line("Parameters-version changed");
		validation = false;
	}
	if(length != EEPROM_LENGTH)
	{
		dwrite_line("Parameters-length changed");
		validation = false;
	}
	
	if(validation == false)
	{
		dwrite_line("***Check Failed - Writing defaults***");
		// if not ok, use defaults
		load_defaults();
	}
	
	
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 5th April 2010
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----
void stored_parameters::load_defaults()
{
#define def_param(name, type, default) name = default;
#include "stored_parameters.def"
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 5th April 2010
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----
void stored_parameters::list()
{
#define def_param(name, type, default) write_line(#name" = ", name);
#include "stored_parameters.def"
}



void stored_parameters::set_standard_16x16_maze()
{
	map_size_x = 16;
	map_size_y = 16;
	map_target_x = 7;
	map_target_y = 7;
	map_target_size_x = 2;
	map_target_size_y = 2;
	save_parameters();			// could introduct intelligence at some point to see if the parameters had actually changed before saving!
}

void stored_parameters::set_test_5x5_maze()
{
	map_size_x = 5;
	map_size_y = 5;
	map_target_x = 4;
	map_target_y = 4;
	map_target_size_x = 1;
	map_target_size_y = 1;
	save_parameters();
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        Define all the getter functions
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 5th April 2010
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----
#define def_param(name, type, default) unsigned int stored_parameters::get_##name() { return name; }
#include "stored_parameters.def"

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        Define all the setter functions
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 5th April 2010
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----
// (If this got any longer we could get it to call a working function, but it's not really worth it at the moment)
#define def_param(name, type, default) \
void stored_parameters::set_##name(unsigned int name##_to_set) \
{ \
name = name##_to_set; \
save_parameters(); \
}
#include "stored_parameters.def"

