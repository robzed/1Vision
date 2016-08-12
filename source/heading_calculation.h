/*
 *  heading_calculation.h
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

#ifndef HEADING_CALCULATION_H
#define HEADING_CALCULATION_H

#include "line_list_custom_vector.h"
#include "advanced_line_vector.h"

int get_angle_for_single_line(aline& l);		

// note: these adjust headings to be between -45 and +45
int get_heading(advanced_line_vector& lines, int longest_index);			// longest_index shouldn't be -1
int get_heading(advanced_line_vector& lines, int longest_index, int longest_index2);	// longest_index shouldn't be -1

// support routines
void longest_three(line_list_custom_vector& lines1, line_list_custom_vector& lines2, int *longest_i);
void longest_three(advanced_line_vector& lines, int *longest_i);
int longest_vertical(line_list_custom_vector& lines1, line_list_custom_vector& lines2);
int longest_vertical(advanced_line_vector& lines);
void two_longest_vertical(advanced_line_vector& lines, int *longest_list);


#endif

