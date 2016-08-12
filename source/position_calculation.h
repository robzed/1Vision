/*
 *  position_calculation.h
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

#ifndef POSITION_CALCULATION
#define POSITION_CALCULATION

#include "advanced_line_vector.h"

class PositionCalculation {
public:
	//PositionCalculation(int longest_vertical_index, int second_longest_vert_index, const advanced_line_vector& lines);
	PositionCalculation(int longest_vertical_index, const advanced_line_vector& rotated_lines);
	int get_cell_offset_x();	
	int get_cell_offset_z();
	bool x_valid();
	bool z_valid();
private:
	int cell_offset_x;
	int cell_offset_z;
	bool z_offset_valid;
	bool x_offset_valid;
};


#endif


