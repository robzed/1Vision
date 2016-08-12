/*
 *  reverse_perspective_projection.h
 *  robot_core
 *
 *  Created by Rob Probin on 24/11/2008.
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
 */

#ifndef REVERSE_PERSPECTIVE_PROJECTION_H
#define REVERSE_PERSPECTIVE_PROJECTION_H

#include "line_list_custom_vector.h"
#include "advanced_line_vector.h"

void make_lines_plan_view(line_list_custom_vector& lines);
void make_lines_plan_view(advanced_line_vector& lines);


#endif // REVERSE_PERSPECTIVE_PROJECTION_H

