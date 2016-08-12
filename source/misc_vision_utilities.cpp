/*
 *  misc_vision_utilities.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 13/04/2009.
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
 */

#include "misc_vision_utilities.h"
#include "sine_cosine.h"
#include "hardware_manager.h"
#include "language_support.h"
#include "Real_World.h"


void vvappend(advanced_line_vector& target, line_list_custom_vector& source)
{
	int size = source.size();
	for(int i=0; i<size; i++)
	{
		aline temp;
		line& s= source[i];
		temp.start_x = s.start.x;
		temp.start_y = s.start.y;
		temp.end_x = s.end.x;
		temp.end_y = s.end.y;
		target.push_back(temp);
	}
}



bool get_blue_compare()
{
	stored_parameters* stored = 0;
	Hardware_Manager* hw= Hardware_Manager::Instance();
	stored = hw->get_params();
	if(stored==0)
	{
		halt("blue:stored=0");
	}
	return stored->get_is_blue_compare_on();
}


// Reject lines that are greater than 5 cells away
// Sometime in the future this should also reject lines outside the actual maze.
void reject_lines(advanced_line_vector& ln)
{
	//if(ln.size() == 0) return;
	//aline* line = ln[0];
	int size = ln.size();
	const int max_z = 5*wallLength_in_mm;
	for(int i = 0; i < size; i++)
	{
		int start_wz = ln[i].start_wz;
		int end_wz = ln[i].end_wz;
		if(start_wz > max_z && end_wz > max_z)
		{
			//write_string("max=", max_z);
			//write_string("start_wz=", start_wz);
			//write_line("end_wz=", end_wz);
			ln[i].out_of_bounds = 1;
		}
		else
		{
			ln[i].out_of_bounds = 0;
		}
	}
}


void rotate_plan_view(advanced_line_vector& rotated_out, const advanced_line_vector& lines_in, int angle)
{
	int number = lines_in.size();
	const aline* line = &(lines_in[0]);
	for(int i = 0; i < number; i++)
	{
		// get data from source
		int start_wx = line->start_wx;
		int start_wz = line->start_wz;
		int end_wx = line->end_wx;
		int end_wz = line->end_wz;
		
		// rotate coordinates
		// x' = x cos f - y sin f
		// y' = y cos f + x sin f
		int cosA = fast_cosine(angle);
		int sinA = fast_sine(angle);
		
		// first pair of coordinates
		int new_start_wx = (start_wx * cosA - start_wz * sinA)/trig_scale_factor;
		int new_start_wz = (start_wz * cosA + start_wx * sinA)/trig_scale_factor;
		// second part of coordinates
		int new_end_wx = (end_wx * cosA - end_wz * sinA)/trig_scale_factor;
		int new_end_wz = (end_wz * cosA + end_wx * sinA)/trig_scale_factor;
		
		// add to destination
		aline rotated_line;
		rotated_line.start_wx = new_start_wx;
		rotated_line.start_wz = new_start_wz;
		rotated_line.end_wx = new_end_wx;
		rotated_line.end_wz = new_end_wz;
		rotated_out.push_back(rotated_line);
		
		// move to next line
		line++;
	}
	
}
void rotate_plan_view(advanced_line_vector& lines, int angle)
{
	int number = lines.size();
	aline* line = &(lines[0]);
	for(int i = 0; i < number; i++)
	{
		// get data from source
		int start_wx = line->start_wx;
		int start_wz = line->start_wz;
		int end_wx = line->end_wx;
		int end_wz = line->end_wz;
		
		// rotate coordinates
		// x' = x cos f - y sin f
		// y' = y cos f + x sin f
		int cosA = fast_cosine(angle);
		int sinA = fast_sine(angle);
		
		// first pair of coordinates
		int new_start_wx = (start_wx * cosA - start_wz * sinA)/trig_scale_factor;
		int new_start_wz = (start_wz * cosA + start_wx * sinA)/trig_scale_factor;
		// second part of coordinates
		int new_end_wx = (end_wx * cosA - end_wz * sinA)/trig_scale_factor;
		int new_end_wz = (end_wz * cosA + end_wx * sinA)/trig_scale_factor;
		
		line->start_wx = new_start_wx;
		line->start_wz = new_start_wz;
		line->end_wx = new_end_wx;
		line->end_wz = new_end_wz;
		
		// move to next line
		line++;
	}
	
}

void copy_2d_and_bounds(advanced_line_vector& lines_out, const advanced_line_vector& lines_in)
{
	int number = lines_in.size();
	int number2 = lines_out.size();
	if(number != number2) { halt("bad size copy_2d"); }
	
	aline* lo = &(lines_out[0]);
	const aline* li = &(lines_in[0]);
	for(int i = 0; i < number; i++)
	{
		lo->start_x = li->start_x;
		lo->start_y = li->start_y;
		lo->end_x = li->end_x;
		lo->end_y = li->end_y;
		
		lo->out_of_bounds = li->out_of_bounds;
		lo++; li++;
	}
	
}

#if 0
void evaluate_scene(int estimated_x, int estimated_z)
{
	line_list_custom_vector& lines1 = *new line_list_custom_vector;
	line_list_custom_vector& lines2 = *new line_list_custom_vector;
	IPC_get_lines ipc(&ipc_serial, static_cast<byte>(level), static_cast<byte>(get_blue_compare()), lines1, lines2);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("follower failed - ipc failed");
		wf_died = true;
	}
	else
	{
		//write_line("completed");
		advanced_line_vector& lines = *new advanced_line_vector();
		vvappend(lines, lines1);			// waste of time, but due to historical reasons...
		vvappend(lines, lines2);
		
		// convert to 3d
		make_lines_plan_view(lines);
		
		// compare lines 60 vs. lines 6
		//...now we need to invalidate lines that are outside the map as we see it
		// do we need to clip lines that pass outside the scene?
		// for now we just:
		// (a) reject lines where both points are beyond 5 cells forward. We probably 
		// can't see this far anyway. This is in the untranslated z direction.
		// (b) reject lines that are off the map in the left-right direction beyond
		// the map edge. We don't do this yet.
		reject_lines(lines);
		
		int longest_indexes[3] = {-1, -1, -1};
		// do this on the 2d data to determine the biggest resolution data
		two_longest_vertical(lines, longest_indexes);
		//longest_indexes[0] = longest_vertical(lines);
		//longest_three(lines, longest_indexes);
		
#define TWO_LINE_ARRAYS
#ifdef TWO_LINE_ARRAYS
		advanced_line_vector& rotated_lines = *new advanced_line_vector();
#endif
		//write_line("Free=", total_free());
		//write_line("^Bk=", largest_free());
		//write_line("stk=", user_stack_free());
		
		int raw_angle = 9999;
		if(longest_indexes[0] != -1)
		{
			raw_angle = get_heading(lines, longest_indexes[0], longest_indexes[1]);
			//write_string("r= ", raw_angle);
			//write_line(" ang=", trig_angle_to_degrees(raw_angle));
			heading = raw_angle;
			
#ifdef TWO_LINE_ARRAYS
			rotate_plan_view(rotated_lines, lines, raw_angle);
			copy_2d_and_bounds(rotated_lines, lines);
#else
			rotate_plan_view(lines, raw_angle);
#endif
		}
		
		//get_position(lines, longest_indexes);
#ifdef TWO_LINE_ARRAYS
		PositionCalculation position(longest_indexes[0], rotated_lines);	
#else
		PositionCalculation position(longest_indexes[0], lines);	
#endif
		
		// rotated lines
		//write_lines_plan(rotated_lines, longest_indexes);
		int offset_x = estimated_x;
		int offset_z = estimated_z;
		if(position.x_valid()) { offset_x = position.get_cell_offset_x(); }
		if(position.z_valid()) { offset_z = position.get_cell_offset_z(); }
		updated_local_x = offset_x;
		updated_local_z = offset_z;
		
		// XXXXXXXXXXXXXXXXX
		//LocalMap map;
#ifdef TWO_LINE_ARRAYS
		interpret_lines(rotated_lines, map, offset_x, offset_z);
#endif
		// XXXXXXXXXXXXXXXXX
		
#if 0
		write_string("#!2");		// 2 is the type for basic detection map plus maze image
		
		void common_write_detection_boxes(int inside_cell_x, int inside_cell_z);
		common_write_detection_boxes(offset_x, offset_z);
		
		write64_16(5);
		for(int z=0; z<5; z++)
		{
			uint16_t front_map = 0;
			uint16_t left_map = 0;
			for(int x=-2; x<3; x++)
			{
				front_map = front_map << 1;
				if(map.forward_wall(x, z)) { front_map |= 1; }
				left_map = left_map << 1;
				if(map.left_wall(x, z)) { left_map |= 1; }
			}
			write64_16(front_map);
			write64_16(left_map);
		}
		
		// show all the lines
		write64_16(rotated_lines.size());
		for(int i = 0; i < rotated_lines.size(); i++)
		{
			write64_16(rotated_lines[i].start_wx);
			write64_16(rotated_lines[i].start_wz);
			write64_16(rotated_lines[i].end_wx);
			write64_16(rotated_lines[i].end_wz);
		}
		
		// show longest lines
		write64_16(3);
		for(int i=0; i<3; i++)
		{
			write64_16(longest_indexes[i]);
		}
		
		// write out the estimated position
		if(position.x_valid()) { write64_16(position.get_cell_offset_x()); }
		else { write64_16(9999); }
		if(position.z_valid()) { write64_16(position.get_cell_offset_z()); }
		else { write64_16(9999); }
		
		int heading_degrees = 9999;
		if(raw_angle != 9999) { heading_degrees = trig_angle_to_degrees(raw_angle); }
		
		write64_16(heading_degrees);	// write out the angle as well
		
		write_string("!!");
#endif
		
#ifdef TWO_LINE_ARRAYS
		delete& rotated_lines;
#endif
		delete& lines;
		
	}
	delete& lines1;
	delete& lines2;
}

#endif




