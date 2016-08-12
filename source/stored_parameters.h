/*
 *  stored_parameters.h
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

#ifndef STORED_PARAMETERS_H
#define STORED_PARAMETERS_H

#include "eeprom.h"

//
// Really a simple data storage class
//
class stored_parameters  {
	
public:
	stored_parameters();

	void set_standard_16x16_maze();
	void set_test_5x5_maze();

	void list();
	void load_defaults();
	
	// Define a whole bunch of getters with the form:
	//		unsigned int get_xxx();
	// where xxx is the name in the file "stored_parameters.def"
#define def_param(name, type, default) unsigned int get_##name();
#include "stored_parameters.def"
	// Define a whole bunch of setters with the form:
	//		void set_xxx(unsigned int xxx_to_set);
	// where xxx is the name in the file "stored_parameters.def"
#define def_param(name, type, default) void set_##name(unsigned int name##_to_set);
#include "stored_parameters.def"
	
private:
	void load_parameters();
	void save_parameters();
	unsigned int load_parameter(unsigned short address, unsigned int& checksum);
	void save_parameter(unsigned short address, unsigned int value, unsigned int& checksum);
	
	// copy of eeprom data
#define def_param(name, type, default) unsigned int name;
#include "stored_parameters.def"
	
	eeprom ee;
};


//#define def_param(name, type, default) name,
//enum stored_parameter_enum {
//#include "stored_parameters.def"
//	number_of_stored_parameter_enums
//};


#endif // STORED_PARAMETERS_H
