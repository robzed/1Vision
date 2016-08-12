/*
 *  position_calculation.cpp
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
 */

#include "position_calculation.h"
#include "pc_uart.h"
#include "language_support.h"

int horizontal_position_line_find(const advanced_line_vector& lines)
{
	//write_line("horizontal_position_line_find()");
	int best_index = -1;
	int lowest_score = 1000000000;
	
	uint16_t number = lines.size();

	for(int i = 0; i < number; i++)
	{
		//write_line("line:", i);
		const aline& l = lines[i];

		if(l.out_of_bounds) { continue; }			// ignore
		
		int width_wx = abs(l.end_wx - l.start_wx);
		int width_wz = abs(l.end_wz - l.start_wz);
		//write_string("line:", i); write_string("width_wx", width_wx); write_string("width_wz", width_wz); write_line("wx>wz", width_wx>width_wz);
		// must be horizontal (i.e. abs(x) > abs(z)) (i.e. bearing > 45 or bearing <-45)
		if(width_wx > width_wz) {
			int approx_2d_length = abs(l.end_x - l.start_x) + abs(l.start_y - l.end_y);
			// Lines must be >20 pixels in 2d
			// REASON: Small lines are terrible for position calculation
			//write_line("approx 2d length=", approx_2d_length);
			if(approx_2d_length > 20)
			{
				// Characteristics to score against
				//
				// 1. we like the lines that have a lower average z (closer)
				// REASON: Ensure accuracy by making as close to the camera as possible.
				//
				// 2. we like the lines that are nearer to the center line (low x)
				// REASON: avoid camera distortion
				//
				// 3. we like lines that are long in 2d (high width in x + width_z)
				// REASON: Provides better accuracy
				// 
				//           2d>>0-320, aver=30  x>>0-720, aver=120   y>>0-1000, average=360
//				int score = (320-approx_2d_length) + width_wx + width_wz;
				int score = (320-approx_2d_length) + abs(l.end_wx) + abs(l.start_wx) + abs(l.end_wz) + abs(l.start_wz);
				//write_line("line score=", score);
				if(score<lowest_score)
				{
					//write_line("lowest score so far!");
					best_index = i;
					lowest_score = score;
				}
			}
		}
	}
	
	return best_index;
}

int vertical_position_line_find(const advanced_line_vector& lines)
{
	//write_line("vertical_position_line_find()");
	int best_index = -1;
	int lowest_score = 1000000000;
	
	uint16_t number = lines.size();
	
	for(int i = 0; i < number; i++)
	{
		if(lines[i].out_of_bounds) { continue; }			// ignore
		
		const aline& l = lines[i];
		int width_wx = abs(l.end_wx - l.start_wx);
		int width_wz = abs(l.end_wz - l.start_wz);
		//write_string("line:", i); write_string("width_wx", width_wx); write_string("width_wz", width_wz); write_line("wx>wz", width_wx>width_wz);
		
		// must be vertical (i.e. abs(z) > abs(x)) (i.e. bearing > 45 or bearing <-45)
		if(width_wz > width_wx) {
			int approx_2d_length = abs(l.end_x - l.start_x) + abs(l.start_y - l.end_y);
			// Lines must be >20 pixels in 2d
			// REASON: Small lines are terrible for position calculation
			//write_line("approx 2d length=", approx_2d_length);
			if(approx_2d_length > 20)
			{
				// Characteristics to score against
				//
				// 1. we like the lines that have a lower average z (closer)
				// REASON: Ensure accuracy by making as close to the camera as possible.
				//
				// 2. we like the lines that are nearer to the center line (low x)
				// REASON: avoid camera distortion
				//
				// 3. we like lines that are long in 2d (high width in x + width_z)
				// REASON: Provides better accuracy
				// 
				//           2d>>0-320, aver=30  x>>0-720, aver=120   y>>0-1000, average=360
				int score = (320-approx_2d_length) + abs(l.end_wx) + abs(l.start_wx) + abs(l.end_wz) + abs(l.start_wz);
				//write_line("line score=", score);
				if(score<lowest_score)
				{
					//write_line("lowest score so far!");
					best_index = i;
					lowest_score = score;
				}
			}
		}
	}
	
	return best_index;
}



//PositionCalculation::PositionCalculation(int longest_vertical_index, int second_longest_vert_index, const advanced_line_vector& lines)
PositionCalculation::PositionCalculation(int longest_vertical_index, const advanced_line_vector& rotated_lines)
{
	//
	// Check if we have a bearing/heading line... if not we probably can't do anything
	// 
	if(longest_vertical_index == -1)
	{
		// no longest vertical probably means no (in bound) lines at all
		x_offset_valid = false;
		z_offset_valid = false;
		return;
	}
	
	// 
	// is the heading line +45/-45 degrees?
	//
	const aline& l = rotated_lines[longest_vertical_index];
	int width_x = abs(l.end_wx - l.start_wx);
	int width_z = abs(l.end_wz - l.start_wz);

	
	if(width_z >= width_x) // 45 degrees or more vertical (this is the normal case)
	{
		// heading is in local Z direction
		cell_offset_x = ((l.start_wx + l.end_wx)/2) % 180;
		if(cell_offset_x < 0) { cell_offset_x = -90-cell_offset_x; }	// adjust for left and right walls
		else {  cell_offset_x = 90-cell_offset_x; }

		//write_line("Heading more vertical, offset x = ", cell_offset_x);
		x_offset_valid = true;
		
		int hori = horizontal_position_line_find(rotated_lines);
		if(hori!=-1)
		{
			cell_offset_z = 180 - (((rotated_lines[hori].start_wz + rotated_lines[hori].end_wz)/2) % 180);
			//write_line("Horizontal, offset z = ", cell_offset_z);
			z_offset_valid = true;
		}
		else
		{
			z_offset_valid = false;
		}
	}
	else // more horizontal
	{
		// heading is in local X direction
		cell_offset_z = 180-(((l.start_wz + l.end_wz)/2) % 180);
		//write_line("Heading more horizontal, offset z = ", cell_offset_z);
		z_offset_valid = true;
		
		int vert = vertical_position_line_find(rotated_lines);
		if(vert!=-1)
		{
			cell_offset_x = ((rotated_lines[vert].start_wx + rotated_lines[vert].end_wx)/2) % 180;
			if(cell_offset_x < 0) { cell_offset_x = -90-cell_offset_x; }	// adjust for left and right walls
			else {  cell_offset_x = 90-cell_offset_x; }
			//write_line("Vertical, offset x = ", cell_offset_x);
			x_offset_valid = true;
		}
		else
		{
			x_offset_valid = false;
		}
	}
	
	//if(second_longest_vert_index == -1)
	//{
	//}
	//else
	//{
	// need to see what direction second line is in
	//	cell_offset_x = (lines[longest_vertical_index].start_wx + lines[longest_vertical_index].end_wx);
	//
	//	cell_offset_x += lines[second_longest_vert_index].start_wx + lines[second_longest_vert_index].end_wx;
	// won't work ... need to mod 180 each one
	//	cell_offset_x /= 4;
	//	cell_offset_x %= 180;
	//	x_offset_valid = true;
	//}

	
	//
	// Calculate z offset
	//
	// we need to find a line that's at 
	
	
	// decide
}


int PositionCalculation::get_cell_offset_x()
{
	if(x_offset_valid == false)
	{
		halt("invalid xoffset");
	}
	
	return cell_offset_x;
}

int PositionCalculation::get_cell_offset_z()
{
	if(z_offset_valid == false)
	{
		halt("invalid zoffset");
	}
	
	return cell_offset_z;
}


bool PositionCalculation::x_valid()
{
	return x_offset_valid;
}

bool PositionCalculation::z_valid()
{
	return z_offset_valid;
}


