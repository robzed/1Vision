/*
 *  follower_map.cpp
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
 *
 */

#include "follower_map.h"
#include "language_support.h"
#include "pc_uart.h"


#define DECAY_WALLS
const unsigned char wall_seen_value = 4;
const unsigned char wall_ceiling = 250;

void FollowerMap::decay_walls()
{
#ifdef DECAY_WALLS
	for(int z = 0; z < number_of_z; z++)
	{
		for(int x = 0; x < number_of_x; x++)
		{
			if(vert[z][x])
			{
				vert[z][x]--;
			}
			if(hori[z][x])
			{
				hori[z][x]--;
			}
		}
	}
#endif
}


FollowerMap::FollowerMap()
: robot_direction_heading(0)
{
	for(int x=0; x<number_of_x; x++)
	{
		for(int z=0; z<number_of_z; z++)
		{
			vert[z][x] = 0;
			hori[z][x] = 0;
		}
	}
}

void FollowerMap::move_forward()
{
	if(robot_direction_heading == 0) { scroll_down(); }		// robot north/up
	else if(robot_direction_heading == 1) { scroll_left(); }	// robot east/right
	else if(robot_direction_heading == 2) { scroll_up(); }	// robot south/down
	else if(robot_direction_heading == 3) { scroll_right(); } // robot west/left
	else { halt("FM:dir"); }	
}

// move high Z to low Z (move map south which is the same as robot north)
void FollowerMap::scroll_down()
{
	for(int z=0; z<(number_of_z-1); z++)
	{
		for(int x=0; x<number_of_x; x++)
		{
			vert[z][x] = vert[z+1][x];
			hori[z][x] = hori[z+1][x];
		}
	}
	// blank the top line
	for(int x=0; x<number_of_x; x++)
	{
		vert[number_of_z-1][x] = 0;
		hori[number_of_z-1][x] = 0;
	}
}

// move low Z to high Z (move map north which is the same as robot south)
void FollowerMap::scroll_up()
{
	for(int z=(number_of_z-1); z>=0; z--)
	{
		for(int x=0; x<number_of_x; x++)
		{
			vert[z][x] = vert[z-1][x];
			hori[z][x] = hori[z-1][x];
		}
	}
	// blank the bottom line
	for(int x=0; x<number_of_x; x++)
	{
		vert[0][x] = 0;
		hori[0][x] = 0;
	}
}

// move low X to high X (move map east which is the same as robot west)
void FollowerMap::scroll_right()
{
	for(int x=(number_of_x-1); x>=0; x--)
	{
		for(int z=0; z<number_of_z; z++)
		{
			vert[z][x] = vert[z][x-1];
			hori[z][x] = hori[z][x-1];
		}
	}
	// blank the left line
	for(int z=0; z<number_of_z; z++)
	{
		vert[z][0] = 0;
		hori[z][0] = 0;
	}
}
// move high X to low X (move map west which is the same as robot east)
void FollowerMap::scroll_left()
{
	for(int x=0; x<(number_of_x-1); x++)
	{
		for(int z=0; z<number_of_x; z++)
		{
			vert[z][x] = vert[z][x+1];
			hori[z][x] = hori[z][x+1];
		}
	}
	// blank the right line
	for(int z=0; z<number_of_z; z++)
	{
		vert[z][number_of_x-1] = 0;
		hori[z][number_of_x-1] = 0;
	}
}

void FollowerMap::rotate_right90()
{
	robot_direction_heading++;
	robot_direction_heading &= 0x03;		// make sure it wraps around
}

void FollowerMap::rotate_left90()
{
	robot_direction_heading--;
	robot_direction_heading &= 0x03;		// make sure it wraps around
}

void FollowerMap::rotate_180()
{
	robot_direction_heading+=2;
	robot_direction_heading &= 0x03;		// make sure it wraps around
}


void FollowerMap::print()
{
	for(int z=(number_of_z-1); z>=0; z--)
	{
		// print the horizontal line
		for(int x=0; x<number_of_x; x++)
		{
			write_char('+');
			if(hori[z][x]) { write_string("--"); }
			else { write_string("  "); }				
		}
		write_line("#", z);
		// print the vertical line
		for(int x=0; x<number_of_x; x++)
		{
			if(vert[z][x]) { write_char('|'); }
			else { write_char(' '); }
			write_string("  ");
		}
		write_line("#", z);
	}
	write_line("Direction=", robot_direction_heading);
}

bool FollowerMap::is(int robot_relative_x, int robot_relative_z, uint8_t robot_relative_direction) const
{
	int map_cell_x; int map_cell_z;
	if(robot_direction_heading==0)
	{
		// translate local X and Z into map X and Z
		// north facing is easy - no transformation
		map_cell_x = robot_relative_x;
		map_cell_z = robot_relative_z;
	}
	else if(robot_direction_heading==1)
	{
		// translate local X and Z into map X and Z
		// east facing ... +X for robot is -Z for map, +Z for robot is +X for map
		map_cell_x = robot_relative_z;
		map_cell_z = -robot_relative_x;
	}
	else if(robot_direction_heading==2)
	{
		// translate local X and Z into map X and Z
		// south facing ... +X for robot is -X for map, +Z for robot is -Z for map
		map_cell_x = -robot_relative_x;
		map_cell_z = -robot_relative_z;
	}
	else // if(robot_direction_heading==3)
	{
		// translate local X and Z into map X and Z
		// west facing ... +X for robot is +Z for map, +Z for robot is -X for map
		map_cell_x = -robot_relative_z;
		map_cell_z = robot_relative_x;
	}
	
	map_cell_x += center_pos_x;
	map_cell_z += center_pos_z;

	int combined_direction = robot_relative_direction + robot_direction_heading;
	combined_direction &= 0x03;
	if(combined_direction==0)
	{
		if(map_cell_x < 0 || map_cell_x >= number_of_x || map_cell_z < 0 || map_cell_z >= number_of_z)
		{
			return false;
		}
		return hori[map_cell_z][map_cell_x];
	}
	else if(combined_direction==1)
	{
		map_cell_x++;
		if(map_cell_x < 0 || map_cell_x >= number_of_x || map_cell_z < 0 || map_cell_z >= number_of_z)
		{
			return false;
		}
		return vert[map_cell_z][map_cell_x];
	}
	else if(combined_direction==2)
	{
		map_cell_z--;
		if(map_cell_x < 0 || map_cell_x >= number_of_x || map_cell_z < 0 || map_cell_z >= number_of_z)
		{
			return false;
		}
		return hori[map_cell_z][map_cell_x];
	}
	else // if(combined_direction==3)
	{
		if(map_cell_x < 0 || map_cell_x >= number_of_x || map_cell_z < 0 || map_cell_z >= number_of_z)
		{
			return false;
		}
		return vert[map_cell_z][map_cell_x];
	}
}


bool FollowerMap::forward_wall(int robot_cell_x, int robot_cell_z) const
{
	return is(robot_cell_x, robot_cell_z, 0);
}

bool FollowerMap::right_wall(int robot_cell_x, int robot_cell_z) const
{
	return is(robot_cell_x, robot_cell_z, 1);
}

bool FollowerMap::left_wall(int robot_cell_x, int robot_cell_z) const
{
	return is(robot_cell_x, robot_cell_z, 3);
}

bool FollowerMap::back_wall(int robot_cell_x, int robot_cell_z) const
{
	return is(robot_cell_x, robot_cell_z, 2);
}

void FollowerMap::set(int robot_relative_x, int robot_relative_z, uint8_t robot_relative_direction)
{
	int map_cell_x; int map_cell_z;

	if(robot_direction_heading==0)
	{
		// translate local X and Z into map X and Z
		// north facing is easy - no transformation
		map_cell_x = robot_relative_x;
		map_cell_z = robot_relative_z;
	}
	else if(robot_direction_heading==1)
	{
		// translate local X and Z into map X and Z
		// east facing ... +X for robot is -Z for map, +Z for robot is +X for map
		map_cell_x = robot_relative_z;
		map_cell_z = -robot_relative_x;
	}
	else if(robot_direction_heading==2)
	{
		// translate local X and Z into map X and Z
		// south facing ... +X for robot is -X for map, +Z for robot is -Z for map
		map_cell_x = -robot_relative_x;
		map_cell_z = -robot_relative_z;
	}
	else // if(robot_direction_heading==3)
	{
		// translate local X and Z into map X and Z
		// west facing ... +X for robot is +Z for map, +Z for robot is -X for map
		map_cell_x = -robot_relative_z;
		map_cell_z = robot_relative_x;
	}

	map_cell_x += center_pos_x;
	map_cell_z += center_pos_z;

	int combined_direction = robot_relative_direction + robot_direction_heading;
	combined_direction &= 0x03;
	if(combined_direction==0)
	{
		if(map_cell_x < 0 || map_cell_x >= number_of_x || map_cell_z < 0 || map_cell_z >= number_of_z)
		{
			return;
		}
#ifndef DECAY_WALLS
		hori[map_cell_z][map_cell_x] = true;		
#else
		if(hori[map_cell_z][map_cell_x] < wall_ceiling)
		{
			hori[map_cell_z][map_cell_x] + wall_seen_value;
		}
#endif
	}
	else if(combined_direction==1)
	{
		map_cell_x++;
		if(map_cell_x < 0 || map_cell_x >= number_of_x || map_cell_z < 0 || map_cell_z >= number_of_z)
		{
			return;
		}
#ifndef DECAY_WALLS
		vert[map_cell_z][map_cell_x] = true;
#else
		if(vert[map_cell_z][map_cell_x] < wall_ceiling)
		{
			vert[map_cell_z][map_cell_x] += wall_seen_value;
		}
#endif
	}
	else if(combined_direction==2)
	{
		map_cell_z--;
		if(map_cell_x < 0 || map_cell_x >= number_of_x || map_cell_z < 0 || map_cell_z >= number_of_z)
		{
			return;
		}
#ifndef DECAY_WALLS
		hori[map_cell_z][map_cell_x] = true;
#else
		if(hori[map_cell_z][map_cell_x] < wall_ceiling)
		{
			hori[map_cell_z][map_cell_x] += wall_seen_value;
		}
#endif
	}
	else // if(combined_direction==3)
	{
		if(map_cell_x < 0 || map_cell_x >= number_of_x || map_cell_z < 0 || map_cell_z >= number_of_z)
		{
			return;
		}
#ifndef DECAY_WALLS
		vert[map_cell_z][map_cell_x] = true;
#else
		if(vert[map_cell_z][map_cell_x] < wall_ceiling)
		{
			vert[map_cell_z][map_cell_x] += wall_seen_value;
		}
#endif
	}
}

void FollowerMap::set_forward_wall(int robot_cell_x, int robot_cell_z)
{
	set(robot_cell_x, robot_cell_z, 0);
}

void FollowerMap::set_right_wall(int robot_cell_x, int robot_cell_z)
{
	set(robot_cell_x, robot_cell_z, 1);
}

void FollowerMap::set_left_wall(int robot_cell_x, int robot_cell_z)
{
	set(robot_cell_x, robot_cell_z, 3);
}

void FollowerMap::set_back_wall(int robot_cell_x, int robot_cell_z)
{
	set(robot_cell_x, robot_cell_z, 2);
}

