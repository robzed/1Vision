/*
 *  command_line.cpp
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

#include "command_line.h"
#include "pc_uart.h"

#ifdef TERMINAL_BUILD_TEST_FRAMEWORK
// ---- for mac build ----
#include "terminal_build_test_framework.h"

#else 
// ---- for embedded build ---- 
#include "command_line_commands.h"
#endif


//
// Make the command list
//

//const command_list_t basic_command_list[] = {
//	{"help", help, "Type 'help command'. Also see list", 1, 2},
//	{"list", list, "Lists commands", 1, 1},
//};
	
//
// How big is that command list?
//
//unsigned int basic_size()
//{
//	return sizeof(basic_command_list)/sizeof(command_list_t);
//}

//
// constructors
//
Command_line::Command_line()
//:
//size(&basic_size),
//cmds(basic_command_list)
{
	// basic_command_list;
	new_command_line();
}

void Command_line::new_command_line()
{
	input_line_pos = 0;
	input_line[0] = 0;		// zero terminate
	write_string(">");
}

//
// run
//
void Command_line::run()
{
	support_periodic_routines();
	
	if(key_ready())
	{
		int c = read_char();
		//write_int(c);
		//write_string("< ");
		if(c == 13 || c == 10)
		{
			//write_string(input_line);
			int result = execute(input_line);
			if(result)
			{
				write_line("Command Failed: ", result);
			}
			new_command_line();
		}
		else if(c == 8 || c == 127)
		{
			// debug line
			//write_char('<');
			// end debug line
			if(input_line_pos != 0)
			{
				input_line_pos--;
				input_line[input_line_pos] = 0;
			}
		}
		else
		{
			// debug lines
			//if(c >= 'a' && c <= 'z')
			//{
			//	write_char(c - 'a' + 'A');
			//}
			// end debug lines
			if(input_line_pos > input_line_size_max)	// i.e. 81
			{
				write_line("Exceeded Input line size");
				input_line_pos = 0;
				input_line[0] = 0;		// zero terminate
				
			}
			else
			{
				input_line[input_line_pos] = static_cast<char>(c);
				input_line_pos++;
				input_line[input_line_pos] = 0;
			}
		}
	}
}

//
// execute
//
int Command_line::execute(char *line_to_execute)
{	
	const int number_of_args = 8;
	char *(argc[number_of_args]);
	int argv = /*util.*/split_line(line_to_execute, argc ,number_of_args);
	int result = -1;	// something goes wrong we get -1

	if(argv==0)
	{
		write_line("No command");
		result = 0;		// but this is no error
	}
	else if(argv < 0)
	{
		write_line("Too many args");
	}
	else
	{
		const command_list_t* cmd = find_command(argc[0]);
		if(cmd == 0 || cmd->function == 0)
		{
			write_line("Command not found");
		}
		else
		{
			if(argv != 0 && argv < cmd->min_args)
			{
				write_line("Too few arguments");
				write_line("Help is:");
				write_line(cmd->help);
			}
			else // there are enough arguments
			{
				if(argv > cmd->max_args)
				{
					write_line("Ignoring extra arguments");
					argv = cmd->max_args;
				}
				result = cmd->function(argv, argc);
			}
		}
	}
	
	return result;
}

//
// find_command
//
const command_list_t* Command_line::find_command(const char *function_name)
{
	const command_list_t* command = 0;		// no function by default
	for(unsigned int i=0; i<size(); i++)
	{
		if(are_strings_equal(cmds[i].name, function_name))
		{
			command = &cmds[i];
			break;
		}
	}
	
	return command;
}

//
// are_strings_equal
//
bool /*String_utilities::*/are_strings_equal(const char *s1, const char *s2)
{
	bool result = false;
	while(1)
	{
		char c1 = *s1; 
		char c2 = *s2;
		if(c1 != c2) { result = false; break; }		// any characters don't match (including zero terminator) strings don't match
		if(c1 == 0) { result = true; break; }
		s1++; s2++;
	}
	
	return result;
}

// see Zex for more advanced splitting/stripping utilities
int /*String_utilities::*/split_line(char *line_to_split, char *vectors_to_char_string[], int max_vectors)
{
	// GOALS:
	//  - split into whitespace seperated text
	//  - get rid of whitespace at the beginning or end
	//  - don't do anything special with double quotes at the moment (they bind whitespace seperated words in Zex)
	//  - simple to debug without debugger
	
	int args_found = 0;
	char c;
	//char** current_vector = vectors_to_char_string;
	enum word_mode_t { looking_for_start, looking_for_end } word_mode = looking_for_start;
	while((c = *line_to_split) != 0)
	{
		if(c <= 32) // whitespace!
		{
			if(word_mode == looking_for_end)
			{
				// terminate the word
				*line_to_split = 0;		// zero terminate line we are splitting where first whitespace is
				word_mode = looking_for_start;
			}
		}
		else // not whitespace
		{
			if(word_mode == looking_for_start)
			{
				if(args_found == max_vectors)
				{
					// oops ... too many arguments to store
					args_found = -1;
					break;
				}
				word_mode = looking_for_end;
				*vectors_to_char_string = line_to_split;
				vectors_to_char_string++;
				args_found++;
			}
			
		}
		line_to_split++;		// next character
	}	
	
	return args_found;
}


//
// help
//
int help(int argc, const char *const argv[])
{
	Command_line temp;

	const command_list_t *cmd;
	if(argc == 2)		
	{
		cmd = temp.find_command(argv[1]);
	}
	else
	{
		cmd = temp.find_command("help");
	}

	if(cmd == 0)
	{
		write_line("No help on that command");
	}
	else
	{
		write_line(cmd->help);
	}
	
	return 0;
}


//
// list
//
int list(int argc, const char *const argv[])
{
	write_line("Commands understood:");

	int j=0;
	for(unsigned int i=0; i<size(); i++)
	{
		write_string(cmds[i].name);
		write_string("\t");
		j++;
		if(j==7)
		{
			j=0;
			write_cr();
		}
	}
	write_cr();
	
	return 0;
}


//
// **** HELPER FUNCTIONS ****
//

// convert a value to integer
bool convert_val(const char* in, int* out)
{
	bool neg=false;
	
	if(*in == '-')
	{
		neg=true;
		in++;
	}

	int result = 0;
	bool success = true;
	while(1)
	{
		char c = *in;
		in++;
		if(c==0) { break; }
		if(c>='0' && c<='9')
		{
			result = result*10 + (c-'0');
		}
		else
		{
			success = false;
			break;
		}
	}

	if(!neg) {
		*out = result;
	} else {
		*out = -result;
	}
	return success;
}

//
// Convert value that prints an error on fail
//
bool convert_val(const char *const arg, int *iptr, const char* fail)
{
	bool success = convert_val(arg, iptr);
	if(!success)
	{
		write_string("Invalid ");
		write_line(fail);
	}
	
	return success;
}


