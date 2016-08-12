/*
 *  command_line.h
 *  robot_core
 *
 *  Created by Rob Probin on 18/11/2006.
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

#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

// the type of function called by 
typedef int command_function_t(int argc, const char *const argv[]);

class Command {
public:
	int execute();
	void write_help();
};

// forward declarations
struct command_list_t;


//class String_utilities {
	
//public:
	// other utility functions
	//bool string_equals(const char *s1, const char *s2);
	bool are_strings_equal(const char *s1, const char *s2);
	
	// see Zex for more advanced splitting/stripping utilities
	int split_line(char *line_to_split, char *vectors_to_char_string[], int max_vectors);
	bool convert_val(const char* in, int* out);
//private:
	
//};


//
// comman
//
class Command_line {
	
public:
	void run();
	int execute(char *line_to_execute);		// line will be modified!
	const command_list_t* find_command(const char *function_name);
	Command_line();
private:
	//String_utilities util;
	static const int input_line_size_max = 80;
	char input_line[input_line_size_max+1];
	short input_line_pos;
//	unsigned int (*size)();
//	const command_list_t* cmds;
	void new_command_line();
};


//
// These routines are for enhanced command lists
//
bool convert_val(const char *const arg, int *iptr, const char* fail);
// these commands do help and list functions
command_function_t help;
command_function_t list;
// command list elements
struct command_list_t {
	const char* name;
	command_function_t* function;
	const char* help;
	int min_args;
	int max_args;			// can make this large for unlimited arguments
};

#endif // COMMAND_LINE_H
