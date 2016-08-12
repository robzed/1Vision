/*
 *  local_map_interface.h
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

#ifndef LOCAL_MAP_INTERFACE_H
#define LOCAL_MAP_INTERFACE_H

class LocalMapInterface {
public:
	virtual bool forward_wall(int robot_cell_x, int robot_cell_z) const = 0;
	virtual bool right_wall(int robot_cell_x, int robot_cell_z) const = 0;
	virtual bool left_wall(int robot_cell_x, int robot_cell_z) const = 0;
	virtual bool back_wall(int robot_cell_x, int robot_cell_z) const = 0;
	
	virtual void set_forward_wall(int robot_cell_x, int robot_cell_z) = 0;
	virtual void set_right_wall(int robot_cell_x, int robot_cell_z) = 0;
	virtual void set_left_wall(int robot_cell_x, int robot_cell_z) = 0;
	virtual void set_back_wall(int robot_cell_x, int robot_cell_z) = 0;
	
	virtual ~LocalMapInterface() { };
};

#endif


