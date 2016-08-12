/*
 *  heading_calculation.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 12/04/2009.
 *  Copyright (C) 2009 Rob Probin.
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

#include "heading_calculation.h"
#include "arctan.h"
#include "language_support.h"
#include "sine_cosine.h"

#include "pc_uart.h"

//
// Heading ... straight ahead is 0
//
int get_angle_for_single_line(aline& l)
{
	int width_z = l.end_wz - l.start_wz;
	int width_x = l.end_wx - l.start_wx;
	int angle = fast_arctan(width_x, width_z);
	
	//write_int(width_z);
	//write_string(" ", width_x);
	//write_line("  angle(deg) =", trig_angle_to_degrees(angle));
	
	return angle;
}

int get_heading(advanced_line_vector& lines, int longest_index)
{
	if(longest_index==-1)
	{
		halt("No heading");
	}
	int heading = get_angle_for_single_line(lines[longest_index]);
	
	// ensure we are between -45 and +45
	if(heading < angle_minus45degrees) { heading+=angle90degrees; }
	else if(heading > angle45degrees) { heading-=angle90degrees; }

	//write_line("heading1=", heading);
	return heading;
}
//int get_heading(advanced_line_vector& lines, int* longest_indexes)
//{
//	// there are four states - no max lines, 1 long line, 2 long lines, 3 long lines
//	if(longest_indexes[0] == -1)	// no long lines
//	{
//		return 1000;
//	}
//	else if(longest_indexes[1] == -1)	// 1 line long
//	{
//		int heading1 = get_heading_for_single_line(lines[longest_indexes[0]]);
//		write_line("heading1 = ", heading1);
//		return heading1;
//	}
//	else if(longest_indexes[2] == -1)
//	{
//		int heading1 = get_heading_for_single_line(lines[longest_indexes[0]]);
//		int heading2 = get_heading_for_single_line(lines[longest_indexes[1]]);
//		write_line("heading = ", heading1);
//		write_line("heading2 = ", heading2);
//		write_line("combined heading = ", (heading1+heading2)/2);
//		return (heading1+heading2)/2;
//	}
//	
//	// must be 3 long indexes
//	int heading1 = get_heading_for_single_line(lines[longest_indexes[0]]);
//	int heading2 = get_heading_for_single_line(lines[longest_indexes[1]]);
//	int heading3 = get_heading_for_single_line(lines[longest_indexes[2]]);
//	
//	write_line("heading = ", heading1);
//	write_line("heading2 = ", heading2);
//	write_line("heading3 = ", heading3);
//	write_line("combined heading = ", (heading1+heading2+heading3)/3);
//	return (heading1+heading2+heading3)/3;
//}

int get_heading(advanced_line_vector& lines, int longest_index, int longest_index2)
{
	if(longest_index==-1)
	{
		halt("longest=-1");
	}
	int heading1 = get_angle_for_single_line(lines[longest_index]);	
	int heading2 = -100000; // make sure it get's rejected if it's out of range
	if(longest_index2!=-1)
	{
		heading2 = get_angle_for_single_line(lines[longest_index2]);
	}

	// ensure these are within +/- 20 first (before we do front facing calcs)
	// this means they are within the same direction
	//write_string("components heading2=", heading1); write_line(" ", heading2);
	int heading;
	const int max_offset = trig_number_of_entries/(360/20);
	if(heading2 > (heading1-max_offset) && heading2 < (heading1+max_offset))
	{
		// it is within 20 degrees
		heading = (heading1+heading2) / 2;
	}
	else
	{
#define TRY_ALIGNING_TWO_HEADING_LINES false

#if TRY_ALIGNING_TWO_HEADING_LINES == false
		// outside of 20 degrees, reject
		heading = heading1;
		//write_line("rejected heading2");
#else
		if(heading2 < heading1) { heading2+=angle90degrees; }
		else if(heading2 > heading1) { heading2-=angle90degrees; }

		if(heading2 > (heading1-max_offset) && heading2 < (heading1+max_offset))
		{
			// it is within 20 degrees
			heading = (heading1+heading2) / 2;
		}
		else
		{
			// outside of 20 degrees, reject
			heading = heading1;
			//write_line("rejected heading2");
		}
#endif
	}
	
	if(heading < angle_minus45degrees) { heading+=angle90degrees; }
	else if(heading > angle45degrees) { heading-=angle90degrees; }

	//write_line("heading2=", heading);

	return heading;
}

//
// Find longest of various types
//

void longest_three(line_list_custom_vector& lines1, line_list_custom_vector& lines2, int *longest_i)
{
	int max_length[3];
	
	for(int i=0; i<3; i++)
	{
		max_length[i] = 0;
		longest_i[i] = -1;
	}
	
	uint16_t number = lines1.size();
	for(int i = 0; i < number; i++)
	{
		int width_x = lines1[i].end.x - lines1[i].start.x;
		int width_y = lines1[i].end.y - lines1[i].start.y;
		int current_length = width_x * width_x + width_y * width_y;
		//write_int(i); write_line(" = ", current_length);
		
		if(current_length > max_length[0]) {
			// move down the next two largest
			longest_i[2] = longest_i[1]; max_length[2] = max_length[1];
			longest_i[1] = longest_i[0]; max_length[1] = max_length[0];
			// now store the biggest
			longest_i[0] = i;            max_length[0] = current_length;
		} else if(current_length > max_length[1]) {
			// move down the next next
			longest_i[2] = longest_i[1]; max_length[2] = max_length[1];
			longest_i[1] = i;            max_length[1] = current_length;
		} else if(current_length > max_length[2]) {
			longest_i[2] = i;            max_length[2] = current_length;
		}
	}
	
	number = lines2.size();
	for(int i = 0; i < number; i++)
	{
		int width_x = lines2[i].end.x - lines2[i].start.x;
		int width_y = lines2[i].end.y - lines2[i].start.y;
		int current_length = width_x * width_x + width_y * width_y;
		//write_int(i); write_line(" = ", current_length);
		
		if(current_length > max_length[0]) {
			// move down the next two largest
			longest_i[2] = longest_i[1]; max_length[2] = max_length[1];
			longest_i[1] = longest_i[0]; max_length[1] = max_length[0];
			// now store the biggest
			longest_i[0] = i+1000;       max_length[0] = current_length;
		} else if(current_length > max_length[1]) {
			// move down the next next
			longest_i[2] = longest_i[1]; max_length[2] = max_length[1];
			longest_i[1] = i+1000;       max_length[1] = current_length;
		} else if(current_length > max_length[2]) {
			longest_i[2] = i+1000;       max_length[2] = current_length;
		}
	}
	
}
void longest_three(advanced_line_vector& lines, int *longest_i)
{
	int max_length[3];
	
	for(int i=0; i<3; i++)
	{
		max_length[i] = 0;
		longest_i[i] = -1;
	}
	
	uint16_t number = lines.size();
	for(int i = 0; i < number; i++)
	{
		if(lines[i].out_of_bounds) { continue; }
		int width_x = lines[i].end_x - lines[i].start_x;
		int width_y = lines[i].end_y - lines[i].start_y;
		int current_length = width_x * width_x + width_y * width_y;
		
		//write_int(i); write_line(" = ", current_length);
		
		if(current_length > max_length[0]) {
			// move down the next two largest
			longest_i[2] = longest_i[1]; max_length[2] = max_length[1];
			longest_i[1] = longest_i[0]; max_length[1] = max_length[0];
			// now store the biggest
			longest_i[0] = i;            max_length[0] = current_length;
		} else if(current_length > max_length[1]) {
			// move down the next next
			longest_i[2] = longest_i[1]; max_length[2] = max_length[1];
			longest_i[1] = i;            max_length[1] = current_length;
		} else if(current_length > max_length[2]) {
			longest_i[2] = i;            max_length[2] = current_length;
		}
	}
	
}
int longest_vertical(line_list_custom_vector& lines1, line_list_custom_vector& lines2)
{
	int max_length = 0;
	int longest_i = -1;
	
	uint16_t number = lines1.size();
	for(int i = 0; i < number; i++)
	{
		int width_y = abs(lines1[i].end.y - lines1[i].start.y);
		
		if(width_y > max_length) {
			longest_i = i;
			max_length = width_y;
		}
	}
	
	number = lines2.size();
	for(int i = 0; i < number; i++)
	{
		int width_y = abs(lines2[i].end.y - lines2[i].start.y);
		
		if(width_y > max_length) {
			longest_i = i;
			max_length = width_y;
		}
	}
	
	return longest_i;
}
int longest_vertical(advanced_line_vector& lines)
{
	int max_length = 0;
	int longest_i = -1;
	
	uint16_t number = lines.size();
	for(int i = 0; i < number; i++)
	{
		if(lines[i].out_of_bounds) { continue; }
		
		int width_y = abs(lines[i].end_y - lines[i].start_y);
		
		if(width_y > max_length) {
			longest_i = i;
			max_length = width_y;
		}
	}
	
	return longest_i;
}

void two_longest_vertical(advanced_line_vector& lines, int *longest_list)
{
	int max_length = 0;
	int longest_i = -1;
	int max_length2 = 0;
	int longest_i2 = -1;
	
	uint16_t number = lines.size();
	for(int i = 0; i < number; i++)
	{
		if(lines[i].out_of_bounds) { continue; }
		int width_y = abs(lines[i].end_y - lines[i].start_y);
		
		if(width_y > max_length) {
			// another very big one ... copy old to second, then get details
			longest_i2 = longest_i; max_length2 = max_length;
			longest_i = i; max_length = width_y;
		} else if(width_y > max_length2) {
			// bigger than the second, but not the first
			longest_i2 = i;
			max_length2 = width_y;
		}
	}
	
	*longest_list = longest_i;
	longest_list[1] = longest_i2;
}



