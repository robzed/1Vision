/*
 *  line_list_custom_vector.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 25/11/2008.
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

#include "line_list_custom_vector.h"
#include "language_support.h"


void line_list_custom_vector::push_back(line& this_line)
{
	bool result = push_back2(this_line);
	
	if(result == false)
	{
		halt("Add of vector failed: no more lines");
	}
}

int line_list_custom_vector::push_back_deleting(line& this_line, int reject_y_size, int biggest_end_y)
{
	// simple case - standard (fast) allocation technique works
	if(push_back2(this_line))
	{
		return count-1;
	}
	
	// we need to scan through the database to check if there are any that are 
	// ASSUMPTIONS:
	// 1. end.y >= start.y
	// 2. We extend lines in the y direction, therefore that's what we are checking that (end.y-start.y >= minimum_y_size)
	//
	line* ln = line_data;
	for(int i=0; i<count; i++)
	{
		int starty = ln->start.y;
		int endy = ln->end.y;
		
		if(endy <= biggest_end_y)	// this one ended a while ago - therefore hasn't been extended to the previous line ... looks a good candidate for killing
		{
			if((endy-starty) <= reject_y_size)
			{
#ifdef RED_EXTRACT_BUILD
				number_of_redefines[i]++;
				//printf("%i Redefined(count=%i)- previously(%i,%i)(%i,%i) - now(%i,%i)(%i,%i) - biggest_end_y=%i\n",
				//i, number_of_redefines[i],
				//line_data[i].start.x, line_data[i].start.y, line_data[i].end.x, line_data[i].end.y,
				//this_line.start.x, this_line.start.y, this_line.end.x, this_line.end.y,
				//	biggest_end_y);
				
				total_lines_generated++;
#endif
				line_data[i] = this_line;		// copy line
				
				return i;
			}
		}
		ln++;
	}
	
	return -1;	// no candidates available
	
}

bool line_list_custom_vector::push_back2(line& this_line)
{
	if(count >= max_lines)
	{
#ifndef RED_EXTRACT_BUILD
		//halt("#max-ln");
		//		write_line("max-ln");			// halt maybe a bit brutal? just continue and not add lines?
		return false;
#else
		//PrintAlert("line_list_custom_vector - Tried to create too many lines");
		return false;
		//exit(-1);
#endif
	}
	
	line_data[count] = this_line;		// copy line
	count++;
#ifdef RED_EXTRACT_BUILD
	total_lines_generated++;
#endif
	return true;
}

line& line_list_custom_vector::operator[](unsigned int i)
{
	if(i >= count)
	{
#ifndef RED_EXTRACT_BUILD
		halt("llcv-i out of bounds");
#else
		PrintAlert("line_list_custom_vector - index out of bounds");
		exit(-1);
#endif
	}
	
	return line_data[i];
}

line_list_custom_vector::line_list_custom_vector()
: count(0)
{
#ifdef RED_EXTRACT_BUILD
	total_lines_generated = 0;
	for(int i = 0; i<max_lines; i++)
	{
		number_of_redefines[i]=0;
	}
#endif
}

