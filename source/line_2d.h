/*
 *  line_2d.h
 *  red_extract_cocoa
 *
 *  Created by Rob Probin on 12/05/2008.
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

#ifndef LINE_2D_H
#define LINE_2D_H

#include "point_2d.h"

class line {
public:
	point start;
	point end;
	
	line(int x, int y) 
	{ 
		//		cout<<"Create Line@"  << hex << (int)this  << dec << " at x=" << x << " y=" << y << endl;
		start.x = x; start.y = y; end.x = x; end.y = y; 
	}
	void change_end(int x, int y) { end.x = x; end.y = y; }
	line()
	{ 
		start.x = 0; start.y = 0; end.x = 255; end.y = 255; 
		//		cout<<"Create Line@"  << hex << (int)this  << dec << endl;
	}
};



#endif
