/*
 *  simple_wall_follower_engine.cpp
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
 */

#include "simple_wall_follower_engine.h"
#include "pc_uart.h"
#include "language_support.h"
#include "advanced_line_vector.h"
#include "local_map.h"
#include "heading_calculation.h"
#include "position_calculation.h"
#include "interprocessor_comms.h"
#include "wall_detector.h"
#include "reverse_perspective_projection.h"
#include "misc_vision_utilities.h"
#include "arctan.h"
#include "hardware_manager.h"
#include "Real_World.h"
#include "sine_cosine.h"

static const int dx = 2;

SimpleWallFollowerEngine::SimpleWallFollowerEngine(int red_level_in, bool debug)
: level(red_level_in), updated_local_x(0), updated_local_z(66), 
wf_died(false), heading(0), just_turned_left_flag(false), debug_mode(debug)
{
	map.set_left_wall(0, 0);
	map.set_right_wall(0, 0);
	map.set_back_wall(0, 0);
	map.print(); write_cr();
}

//struct {
//	int bearing;
//	int estimated_x_position;
//	int estimated_y_position;
//};


void SimpleWallFollowerEngine::update_location()
{
	if(debug_mode) { 
		write_string(" Pre motors: ");
		print_status();
	}

	int distance_um;
	int rotation;
	p.get_current_distance_from_baseline(distance_um, rotation);
	int distance_mm = distance_um / 1000;
	if(debug_mode) {
	write_string("dist change=", distance_mm); write_line(" rotation change=", rotation);
	}
	
	//
	// update distance
	//
	updated_local_z += distance_mm;

	while(updated_local_z > wallLength_in_mm)
	{
		updated_local_z -= wallLength_in_mm;
	}
	
	//
	// update rotation
	//
	
	heading += convert_to_trig_angle(rotation);
	
	// have we rotated local_x and local_z here?
	while(heading > angle45degrees)
	{
		heading -= angle90degrees;
	}
	while(heading < angle_minus45degrees)
	{
		heading += angle90degrees;
	}

	if(debug_mode) { 
		write_string(" Post motors: ");
		print_status();
	}
}

void SimpleWallFollowerEngine::move_forward(int distance_in_um)
{
	if(debug_mode) { write_line("move forward ", distance_in_um/1000); }
	p.move_straight_with_stop(distance_in_um);
	while(! p.is_finished())
	{
		p.update();
	}
	// update position & heading here
	update_location();
}

void SimpleWallFollowerEngine::rotate_raw(int raw_angle)
{
	if(debug_mode) { write_line("rotate raw ", raw_angle); }
	p.turn_on_spot_from_to_stop(trig_angle_to_degrees(raw_angle));
	while(! p.is_finished())
	{
		p.update();
	}
	// update position & heading here
	update_location();
}
void SimpleWallFollowerEngine::rotate_deg(int angle_deg)
{
	if(debug_mode) { write_line("rotate deg ", angle_deg); }
	p.turn_on_spot_from_to_stop(angle_deg);
	while(! p.is_finished())
	{
		p.update();
	}
	// update position & heading here
	update_location();
}


// return codes: -1 = turn left, 0 = straight, 1 = turn right, 2 = turn 180, -2 = cannot proceed, stop
int SimpleWallFollowerEngine::decide_next_move()
{
	int return_value = -2;
	if(just_turned_left_flag)
	{
		just_turned_left_flag = false;

		// don't try to turn left again because we just turned left
		if(!map.forward_wall(0,0)) { return_value = 0; }
		else if(!map.right_wall(0,0)) { return_value = 1; }
		else if(!map.back_wall(0,0)) { return_value = 2; }
		else write_line("No exits after left turn");
	}
	else	// we haven't turned left ... must mean we either turned right (because of a wall, or went straight)
	{
		// try to turn left first
		if(!map.left_wall(0,0)) { just_turned_left_flag = true; return_value = -1; }
		else if(!map.forward_wall(0,0)) { return_value = 0; }
		else if(!map.right_wall(0,0)) { return_value = 1; }
		else if(!map.back_wall(0,0)) { return_value = 2; }
		else write_line("No exits");
	}

	if(return_value == 0)
	{
		// check for right wall if next cell contains a left wall and a forward wall before we get there
		// this is to avoid a problem with the limited right hand field of view
		if(map.left_wall(0,1) && map.forward_wall(0,1))
		{
			rotate_deg(-20);
			
			evaluate_scene(updated_local_x, updated_local_z);
			if(debug_mode) { print_status(); map.print(); write_cr(); }
			rotate_deg(20);
		}
		
	}
	return return_value;
	
//	// wall follow next move. Try to follow the wall into the next cell...
//	if(!map_forward_wall(0,0))
//	{
//		// no wall directly in front of us, check for left wall follower algorithm
//		if(!map_left_wall(0,1)) { return 0; }
//		if(!map_forward_wall(0,1)) { return 0; }
//		if(!map_right_wall(0,1)) { return 0; }
//
//		// the next area is a box
//		// left wall follower would go right at this point
//		if(map_right_wall(0,0)) { return 1; }
//		if(map_back_wall(0,0)) { return 2; }
//		if(map_left_wall(0,0)) { return -1; }
//	}
//
//	//
//	// wall in front of us
//	//
//	return 1;
}

void SimpleWallFollowerEngine::do_next_move(int move)
{	
	// need to update map here... (forward, left, right, 180)
	// also compensate for bearing and absolute position
	//
	if(move == 0)
	{
		// do position correction here in local X...
		// if no local x position correction, then do heading correction
		if(abs(updated_local_x) > 10)
		{
			// we need to adjust heading to compensate for this
			int angle = fast_arctan(updated_local_x, 180);
			if(debug_mode) { write_line(">>>> x adjust angle=", trig_angle_to_degrees(angle)); }
			if(debug_mode) { write_line(">>>> actual angle=", trig_angle_to_degrees(angle-heading)); }
			//if(debug_mode) { write_line(">>>> actual angle/2=", trig_angle_to_degrees(angle-heading)/2); }
			rotate_raw(angle-heading);
		}
		else
		{
			while(abs(heading) > min_adjust_angle)
			{
				if(debug_mode) { write_line("heading adjustment", trig_angle_to_degrees(heading)); }
				rotate_raw(-heading/2);
				evaluate_scene(updated_local_x, updated_local_z);
				print_status();
			}
		}
		
		// do position correcion here in local Z...
		
		int move_distance = 180000+(90000-updated_local_z*1000);
		write_line("move distance = ", move_distance);
		//int move_distance = 180000;

		//move_forward(move_distance);	// compensate for wrong Z
		//
		// These lines are a hack to compensate for the poor movement. 
		// Moving 90mm is more effective than moving 180mm. We also need to 'add 
		// a bit on' to comensate for the internal vs. actual (skidding?)
		move_distance += move_distance >> 4;	// add on a sixteenth. slightly less than skidding.
		move_distance >>= 1;			// divide into two moves
		move_forward(move_distance);
		move_forward(move_distance);
		// end of hack
		
		map.move_forward();
	}
	else if(move == 1) 
	{
		rotate_raw(angle_minus90degrees-heading);	// correct heading whilst we move
		map.rotate_right90();
	}
	else if(move == -1)
	{
		rotate_raw(angle90degrees-heading);		// correct heading whilst we move
		map.rotate_left90();
	}
	else if(move == 2)
	{
		rotate_raw(angle180degrees-heading);	// correct heading whilst we move
		map.rotate_180();
	}
	else 
	{
		write_line("do next move=?");
		wf_died = true; 
	}
}

void SimpleWallFollowerEngine::print_status()
{
	write_string("local x=", updated_local_x); write_string(" local z=", updated_local_z); 
	write_string(" heading=", heading); write_string(" (", trig_angle_to_degrees(heading)); write_line(" degrees)");
}

void SimpleWallFollowerEngine::run()
{
	Hardware_Manager* hw= Hardware_Manager::Instance();
	Button *b1 = hw->button1();
	Button *b2 = hw->button2();
	Button *b3 = hw->button3();

	// wait for one second taking some photos
	for(int i=0; i<5; i++)
	{
		evaluate_scene(updated_local_x, updated_local_z);
		delay(200);
	}
	
	evaluate_scene(updated_local_x, updated_local_z);
	if(debug_mode) { print_status(); map.print(); write_cr(); }
	move_forward(30000);

	evaluate_scene(updated_local_x, updated_local_z);
	if(debug_mode) { print_status(); map.print(); write_cr(); }
	rotate_deg(-20);

	evaluate_scene(updated_local_x, updated_local_z);
	if(debug_mode) { print_status(); map.print(); write_cr(); }
	rotate_deg(20);
	
	evaluate_scene(updated_local_x, updated_local_z);
	if(debug_mode) { print_status(); map.print(); write_cr(); }
	rotate_raw(-heading/2);
		
//	while( ! key_ready() && !wf_died && !b1->pressed() && !b2->pressed() && !b3->pressed())
	while(!wf_died && !b1->pressed() && !b2->pressed() && !b3->pressed())
	{
		evaluate_scene(updated_local_x, updated_local_z);
		if(debug_mode) { print_status(); }
		int move = decide_next_move();
		
		if(debug_mode)
		{
			write_line("next move = ", move);
			map.print(); write_cr(); write_line("Press 'q' to quit, 's' to step");
			while(!key_ready()) { }
			int c = read_char();
			if( c != 's' && c != 'S') break;
			map.decay_walls();
		}
		do_next_move(move);

	}
	
	if(wf_died)
	{
		// do beeping here, or something
		write_line("Wall Follower FAILED");
	}
}

void SimpleWallFollowerEngine::evaluate_scene(int estimated_x, int estimated_z)
{
	line_list_custom_vector& lines1 = *new line_list_custom_vector;
	line_list_custom_vector& lines2 = *new line_list_custom_vector;
	IPC_get_lines ipc(&ipc_serial, static_cast<byte>(level), static_cast<byte>(get_blue_compare()), lines1, lines2);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("follower failed - ipc failed");
		wf_died = true;
	}
	else
	{
		//write_line("completed");
		advanced_line_vector& lines = *new advanced_line_vector();
		vvappend(lines, lines1);			// waste of time, but due to historical reasons...
		vvappend(lines, lines2);
		
		// convert to 3d
		make_lines_plan_view(lines);
		
		// compare lines 60 vs. lines 6
		//...now we need to invalidate lines that are outside the map as we see it
		// do we need to clip lines that pass outside the scene?
		// for now we just:
		// (a) reject lines where both points are beyond 5 cells forward. We probably 
		// can't see this far anyway. This is in the untranslated z direction.
		// (b) reject lines that are off the map in the left-right direction beyond
		// the map edge. We don't do this yet.
		reject_lines(lines);
		
		int longest_indexes[3] = {-1, -1, -1};
		// do this on the 2d data to determine the biggest resolution data
		two_longest_vertical(lines, longest_indexes);
		//longest_indexes[0] = longest_vertical(lines);
		//longest_three(lines, longest_indexes);
		
#define TWO_LINE_ARRAYS
#ifdef TWO_LINE_ARRAYS
		advanced_line_vector& rotated_lines = *new advanced_line_vector();
#endif
		//write_line("Free=", total_free());
		//write_line("^Bk=", largest_free());
		//write_line("stk=", user_stack_free());
		
		int raw_angle = 9999;
		if(longest_indexes[0] != -1)
		{
			raw_angle = get_heading(lines, longest_indexes[0], longest_indexes[1]);
			//write_string("r= ", raw_angle);
			//write_line(" ang=", trig_angle_to_degrees(raw_angle));
			heading = raw_angle;
			
#ifdef TWO_LINE_ARRAYS
			rotate_plan_view(rotated_lines, lines, raw_angle);
			copy_2d_and_bounds(rotated_lines, lines);
#else
			rotate_plan_view(lines, raw_angle);
#endif
		}
		
		//get_position(lines, longest_indexes);
#ifdef TWO_LINE_ARRAYS
		PositionCalculation position(longest_indexes[0], rotated_lines);	
#else
		PositionCalculation position(longest_indexes[0], lines);	
#endif

		// rotated lines
		//write_lines_plan(rotated_lines, longest_indexes);
		int offset_x = estimated_x;
		int offset_z = estimated_z;
		if(position.x_valid()) { offset_x = position.get_cell_offset_x(); }
		if(position.z_valid()) { offset_z = position.get_cell_offset_z(); }
		updated_local_x = offset_x;
		updated_local_z = offset_z;
		
		// XXXXXXXXXXXXXXXXX
		//LocalMap map;
#ifdef TWO_LINE_ARRAYS
		interpret_lines(rotated_lines, map, offset_x, offset_z);
#endif
		// XXXXXXXXXXXXXXXXX
		
#if 0
		write_string("#!2");		// 2 is the type for basic detection map plus maze image
		
		void common_write_detection_boxes(int inside_cell_x, int inside_cell_z);
		common_write_detection_boxes(offset_x, offset_z);
		
		write64_16(5);
		for(int z=0; z<5; z++)
		{
			uint16_t front_map = 0;
			uint16_t left_map = 0;
			for(int x=-2; x<3; x++)
			{
				front_map = front_map << 1;
				if(map.forward_wall(x, z)) { front_map |= 1; }
				left_map = left_map << 1;
				if(map.left_wall(x, z)) { left_map |= 1; }
			}
			write64_16(front_map);
			write64_16(left_map);
		}
		
		// show all the lines
		write64_16(rotated_lines.size());
		for(int i = 0; i < rotated_lines.size(); i++)
		{
			write64_16(rotated_lines[i].start_wx);
			write64_16(rotated_lines[i].start_wz);
			write64_16(rotated_lines[i].end_wx);
			write64_16(rotated_lines[i].end_wz);
		}
		
		// show longest lines
		write64_16(3);
		for(int i=0; i<3; i++)
		{
			write64_16(longest_indexes[i]);
		}
		
		// write out the estimated position
		if(position.x_valid()) { write64_16(position.get_cell_offset_x()); }
		else { write64_16(9999); }
		if(position.z_valid()) { write64_16(position.get_cell_offset_z()); }
		else { write64_16(9999); }
		
		int heading_degrees = 9999;
		if(raw_angle != 9999) { heading_degrees = trig_angle_to_degrees(raw_angle); }
		
		write64_16(heading_degrees);	// write out the angle as well
		
		write_string("!!");
#endif

#ifdef TWO_LINE_ARRAYS
		delete& rotated_lines;
#endif
		delete& lines;
		
	}
	delete& lines1;
	delete& lines2;
}
