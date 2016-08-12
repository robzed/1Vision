/*
 *  wall_detector.h
 *  robot_core
 *
 *  Created by Rob Probin on 08/04/2009.
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

#ifndef WALL_DETECTOR_H
#define WALL_DETECTOR_H

#include "advanced_line_vector.h"
#include "local_map_interface.h"

//class WallDetector {
//
//public:
//	WallDetector();
//	~WallDetector();
//	void interpret_lines(const advanced_line_vector& lines, int robot_x_mm_in_cell, int robot_z_mm_in_cell);
//	
//	// absence of lines does not necessary mean absence of walls, especially in further away cases
//	// or in cases where we haven't got detectors on those wall positions
//	bool forward_wall(int robot_cell_x, int robot_cell_z);
//	bool right_wall(int robot_cell_x, int robot_cell_z);
//	bool left_wall(int robot_cell_x, int robot_cell_z);
//	bool back_wall(int robot_cell_x, int robot_cell_z);
//private:
//	
//};

void interpret_lines(const advanced_line_vector& lines, LocalMapInterface& map, int robot_x_mm_in_cell, int robot_z_mm_in_cell);



//
// Allow access to the detection box rectangles for debug purposes
//
int num_detection_boxes();

struct detection_box_rect {
	short cell_x;
	short cell_z;
	
	int min_x;
	int min_z;
	int max_x;
	int max_z;	
};

void get_detection_box(detection_box_rect& output, int index, int robot_x_mm_in_cell, int robot_z_mm_in_cell);

int wall_detector_unit_tests();

#endif

