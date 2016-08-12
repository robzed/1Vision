/*
 *  simple_maze_solver_engine.h
 *  robot_core
 *
 *  Created by Rob Probin on 24/11/2010.
 *  Copyright (C) 2010 Rob Probin.
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

#ifndef SIMPLE_MAZE_SOLVER_ENGINE_H
#define SIMPLE_MAZE_SOLVER_ENGINE_H

#include "ipc_serial_driver.h"
#include "pos_adjust.h"
#include "follower_map.h"

class SimpleMazeSolverEngine {
public:
	SimpleMazeSolverEngine(int red_level_in, bool debug=false);
	void run();
private:
	void evaluate_scene(int estimated_x, int estimated_z);
	void move_forward(int distance_in_um);
	void rotate_raw(int raw_angle);
	void rotate_deg(int angle_deg);
	int decide_next_move();
	void do_next_move(int move);
	void print_status();
	void update_location();
	
	// data
	int level;		// red level
	ipc_serial_driver ipc_serial;
	int updated_local_x;			// from vision
	int updated_local_z;			// from vision
	bool ms_died;
	Pos_adjust2 p;

	int heading;			// from vision
	FollowerMap map;
	bool just_turned_left_flag;
	bool debug_mode;
};



#endif
