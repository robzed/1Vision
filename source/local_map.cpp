/*
 *  local_map.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 10/04/2009.
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

#include "local_map.h"
#include "language_support.h"
#include "pc_uart.h"

LocalMap::LocalMap()
{
	for(int x=0; x<number_of_z; x++)
	{
		for(int z=0; z<number_of_x; z++)
		{
			vert[z][x] = 0;
			hori[z][x] = 0;
		}
	}
}

bool LocalMap::forward_wall(int robot_cell_x, int robot_cell_z) const
{
	robot_cell_x += center_pos_x;
	if(robot_cell_x < 0 || robot_cell_x >= number_of_x || robot_cell_z < 0 || robot_cell_z >= number_of_z)
	{
		return false;
	}
	return hori[robot_cell_z][robot_cell_x];
}

bool LocalMap::right_wall(int robot_cell_x, int robot_cell_z) const
{
	robot_cell_x += center_pos_x;
	robot_cell_x++;
	if(robot_cell_x < 0 || robot_cell_x >= number_of_x || robot_cell_z < 0 || robot_cell_z >= number_of_z)
	{
		return false;
	}
	return vert[robot_cell_z][robot_cell_x];
}

bool LocalMap::left_wall(int robot_cell_x, int robot_cell_z) const
{
	robot_cell_x += center_pos_x;
	if(robot_cell_x < 0 || robot_cell_x >= number_of_x || robot_cell_z < 0 || robot_cell_z >= number_of_z)
	{
		return false;
	}
	return vert[robot_cell_z][robot_cell_x];
}

bool LocalMap::back_wall(int robot_cell_x, int robot_cell_z) const
{
	robot_cell_x += center_pos_x;
	robot_cell_z--;
	if(robot_cell_x < 0 || robot_cell_x >= number_of_x || robot_cell_z < 0 || robot_cell_z >= number_of_z)
	{
		return false;
	}
	return hori[robot_cell_z][robot_cell_x];
}

void LocalMap::set_forward_wall(int robot_cell_x, int robot_cell_z)
{
	robot_cell_x += center_pos_x;
	if(robot_cell_x < 0 || robot_cell_x >= number_of_x || robot_cell_z < 0 || robot_cell_z >= number_of_z)
	{
		halt("local map position out of range");
	}
	//write_string("set front x=", robot_cell_x);
	//write_line(" z=", robot_cell_z);
	hori[robot_cell_z][robot_cell_x] = true;
}

void LocalMap::set_right_wall(int robot_cell_x, int robot_cell_z)
{
	robot_cell_x += center_pos_x;
	robot_cell_x++;
	if(robot_cell_x < 0 || robot_cell_x >= number_of_x || robot_cell_z < 0 || robot_cell_z >= number_of_z)
	{
		halt("local map position out of range");
	}
	vert[robot_cell_z][robot_cell_x] = true;
}

void LocalMap::set_left_wall(int robot_cell_x, int robot_cell_z)
{
	robot_cell_x += center_pos_x;
	if(robot_cell_x < 0 || robot_cell_x >= number_of_x || robot_cell_z < 0 || robot_cell_z >= number_of_z)
	{
		halt("local map position out of range");
	}
	//write_string("set left x=", robot_cell_x);
	//write_line(" z=", robot_cell_z);
	vert[robot_cell_z][robot_cell_x] = true;
}

void LocalMap::set_back_wall(int robot_cell_x, int robot_cell_z)
{
	robot_cell_x += center_pos_x;
	robot_cell_z--;
	if(robot_cell_x < 0 || robot_cell_x >= number_of_x || robot_cell_z < 0 || robot_cell_z >= number_of_z)
	{
		halt("local map position out of range");
	}
	hori[robot_cell_z][robot_cell_x] = true;
}

