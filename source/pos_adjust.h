/*
 *  pos_adjust.h
 *  robot_core
 *
 *  Created by Rob Probin on 26/11/2006.
 *  Copyright (C) 2006 Rob Probin.
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

#ifndef POS_ADJUST_H
#define POS_ADJUST_H

#include "speed_adjust.h"

class Mode_t {
public:
	enum mode_t { INACTIVE, MOVE_STRAIGHT_STOP, MOVE_STRAIGHT_GO, MOVE_STRAIGHT_STOP_SMALL_DISTANCE };
	void change(mode_t new_mode);
	mode_t get();
	Mode_t(mode_t start_mode);
private:
	mode_t mode_internal;
};


class Pos_adjust2 {
	
public:
	void update();
//	Pos_adjust2(Speed_adjust2* speed_obj1, Speed_adjust2* speed_obj2);
	Pos_adjust2();
//	~Pos_adjust2();
	void move_straight_with_stop(int distance_in_um);
	void turn_on_spot_from_to_stop(int angle_in_degrees);	// -ve = clockwise, +ve = anticlockwise
	void move_straight_with_no_stop(int distance_in_um);
	//void move_back_with_stop(int distance_in_um);
	bool is_finished();
	
	// +ve = anticlockwise, -ve = clockwise
	void get_current_distance_from_baseline(int &distance_um_out, int &rotation_degrees_out);
	
private:
	// methods
	void update_move_straight_stop(bool short_distance);
	
	// data
	Speed_adjust2 s1;
	Speed_adjust2 s2;
//	bool speed_adjust_owned_by_us;
	
	int baseline1;
	int baseline2;
	
	// both of these are the same for a straight line
	int target_distance1_from_baseline;
	int target_distance2_from_baseline;
	// for a curve the ratio of the above determines the curve	

	//int slowdown_distance;		// how far to end before we start slowing down
	bool finished_flag;
	Mode_t mode;
	enum rotation_flag_enum { NO_ROTATION=0, ANTICLOCKWISE=1, CLOCKWISE=-1 };
	signed char rotation_flag;	// 0=no rotation +1=anticlockwise, -1=clockwise
	
	char reduced_state;			// for debugging
};


#endif // POS_ADJUST_H
