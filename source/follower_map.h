/*
 *  follower_map.h
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

#ifndef FOLLOWER_MAP_H
#define FOLLOWER_MAP_H

#include "local_map_interface.h"
#include "stdint.h"

class FollowerMap : public LocalMapInterface {

public:
	virtual bool forward_wall(int robot_cell_x, int robot_cell_z) const;
	virtual bool right_wall(int robot_cell_x, int robot_cell_z) const;
	virtual bool left_wall(int robot_cell_x, int robot_cell_z) const;
	virtual bool back_wall(int robot_cell_x, int robot_cell_z) const;
	
	virtual void set_forward_wall(int robot_cell_x, int robot_cell_z);
	virtual void set_right_wall(int robot_cell_x, int robot_cell_z);
	virtual void set_left_wall(int robot_cell_x, int robot_cell_z);
	virtual void set_back_wall(int robot_cell_x, int robot_cell_z);

	// extra features
	void print();
	void move_forward();
	void rotate_right90();	// clockwise
	void rotate_left90();	// anticlockwise
	void rotate_180();
	void decay_walls();
	
	FollowerMap();
private:
	void scroll_down();
	void scroll_up();
	void scroll_right();
	void scroll_left();
	void set(int robot_relative_x, int robot_relative_z, uint8_t robot_relative_direction);
	bool is(int robot_relative_x, int robot_relative_z, uint8_t robot_relative_direction) const;
	
	static const int number_of_z = 5;
	static const int number_of_x = 5;
	static const int center_pos_x = number_of_x/2;
	static const int center_pos_z = number_of_z/2;
	uint8_t vert[number_of_z][number_of_x];
	uint8_t hori[number_of_z][number_of_x];
	uint8_t robot_direction_heading;		// 0=+Z(north) 1=+X(east) 2=-Z(south) 3=-X(west)
};



#endif


