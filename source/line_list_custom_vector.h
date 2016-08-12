/*
 *  line_list_custom_vector.h
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

#ifndef LINE_LIST_CUSTOM_VECTOR_H
#define LINE_LIST_CUSTOM_VECTOR_H

#include "line_2d.h"

class line_list_custom_vector {
public:
	void push_back(line& this_line);		// if it fails, with too many lines, this will halt the processor. This is generally very bad.
	bool push_back2(line& this_line);		// this will return false if it cannot find a simple line allocation (e.g. at the end)
	int push_back_deleting(line& this_line, int reject_y_size, int biggest_end_y);		// this will return -1 if it cannot find a spare line but will otherwise scour the line list for something small
	line& operator[](unsigned int i);
	uint16_t size() { return count; }
	line_list_custom_vector();
#ifdef RED_EXTRACT_BUILD
	int get_redefines(int i) { return number_of_redefines[i]; }
	int get_total_lines_generated() { return total_lines_generated; }
#endif
private:
	enum { max_lines = 600 };	// if you increase this, change the constant in advanced_line_vector.h
	uint16_t count;
	line line_data[max_lines];
#ifdef RED_EXTRACT_BUILD
	int number_of_redefines[max_lines];
	int total_lines_generated;
#endif
};


#endif // LINE_LIST_CUSTOM_VECTOR_H

