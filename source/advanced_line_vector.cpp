/*
 *  advanced_line_vector.cpp
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

#include "advanced_line_vector.h"
#include "language_support.h"


void advanced_line_vector::push_back(aline& this_line)
{
	if(count >= max_lines)
	{
		halt("advanced line vector push_back");
	}
	
	line_data[count] = this_line;		// copy line
	count++;
}


aline& advanced_line_vector::operator[](unsigned int i)
{
	if(i >= count)
	{
		halt("a-line-vec out of bounds");
	}
	
	return line_data[i];
}

const aline& advanced_line_vector::operator[](unsigned int i) const
{
	if(i >= count)
	{
		halt("a-line-vec out of bounds");
	}
	
	return line_data[i];
}

advanced_line_vector::advanced_line_vector()
: count(0)
{
}


//#include <vector>
//#include "pc_uart.h"
//
//void test_function()
//{
//	vector<int>(10) test;
//	
//	for(int i=0; i<10; i++)
//	{
//		test[i] = i*2;
//	}
//	for(int i=0; i<10; i++)
//	{
//		test[i] += test[10-i];
//	}
//	for(int i=0; i<10; i++)
//	{
//		write_int(i); write_cr();
//	}
//}
