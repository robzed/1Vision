/*
 *  advanced_line_vector.h
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

#ifndef ADVANCED_LINE_VECTOR
#define ADVANCED_LINE_VECTOR

// not decided to have built in functions yet....
//#include "line_list_custom_vector.h"
#include "stdint.h"

struct aline {
	// image coords
	int16_t start_x;
	int16_t start_y;
	int16_t end_x;
	int16_t end_y;
	// world coords
	int16_t start_wx;
	int16_t start_wz;
	int16_t end_wx;
	int16_t end_wz;	
	//
	uint8_t out_of_bounds; // 1=yes, 0=no
};


class advanced_line_vector {
public:
	void push_back(aline& this_line);		// if it fails, with too many lines, this will halt the processor. This is generally very bad.
	//void push_back(line& this_line);
	aline& operator[](unsigned int i);
	const aline& operator[](unsigned int i) const;		// inspector version
	uint16_t size() const { return count; }
	advanced_line_vector();
	//advanced_line_vector(line_list_custom_vector& linev);
	//void push_back(line_list_custom_vector& linev);
private:
	enum { max_lines = 1200 };	// twice the number in line_list_custom_vector
	uint16_t count;
	aline line_data[max_lines];
};


#endif // ADVANCED_LINE_VECTOR


