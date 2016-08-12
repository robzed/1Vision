/*
 *  misc_vision_utilities.h
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


#ifndef MISC_VISION_UTILITIES_H
#define MISC_VISION_UTILITIES_H

#include "advanced_line_vector.h"
#include "line_list_custom_vector.h"

void vvappend(advanced_line_vector& target, line_list_custom_vector& source);
bool get_blue_compare();
void reject_lines(advanced_line_vector& ln);
void rotate_plan_view(advanced_line_vector& rotated_out, const advanced_line_vector& lines_in, int angle);
void rotate_plan_view(advanced_line_vector& lines, int angle); // rotate in place
void copy_2d_and_bounds(advanced_line_vector& lines_out, const advanced_line_vector& lines_in);

void evaluate_scene(int estimated_x, int estimated_z);

#endif


