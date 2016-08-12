/*
 *  wall_detector.cpp
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

#include "wall_detector.h"
#include "pc_uart.h"
#include "Real_World.h"
#include "language_support.h"

// Map of areas
//	1001	1000	1010
//	0001	0000	0010
//	0101	0100	0110
const int LEFT = 0x01;
const int RIGHT = 0x02;
const int BOTTOM = 0x04;
const int TOP = 0x08;
const int BOTH_RIGHT_AND_LEFT = LEFT+RIGHT;
const int BOTH_TOP_AND_BOTTOM = TOP+BOTTOM;

// compute outcodes
// based on Cohen-Sutherland http://en.wikipedia.org/wiki/Cohen-Sutherland
// and Fast-Clip http://en.wikipedia.org/wiki/Line_clipping
//
static inline int outcode(int x, int y, int xmin, int ymin, int xmax, int ymax)
{
	int code = 0;
	if (y > ymax) { code = BOTTOM; }
	else if (y < ymin) { code = TOP; }

	if (x > xmax) { code += RIGHT; }
	else if (x < xmin) { code += LEFT; }

	return code;
}

enum { NO_HIT, YES_HIT, POSSIBLE_HIT };
static int base_hits_rectangle(const aline& line_in, int min_x, int min_z, int max_x, int max_z)
{
	int start = outcode(line_in.start_wx, line_in.start_wz, min_x, min_z, max_x, max_z);
	int end = outcode(line_in.end_wx, line_in.end_wz, min_x, min_z, max_x, max_z);
	
	//write_line("start=", start);
	//write_line("end=", end);
	if(start&end) return NO_HIT;		// definately not crossing
	if(start==0 || end==0) return YES_HIT;		// start or end is in center, then definate hit
	
	int or_both = start|end;
	//write_line("or both=", or_both);
	if((or_both&(BOTH_RIGHT_AND_LEFT))==0) return YES_HIT;		// must be in center column
	if((or_both&(BOTH_TOP_AND_BOTTOM))==0) return YES_HIT;		// must be in center line
	
	// we now have a case where there is some diagonal line that may hit the square
	// we only process this one if no other lines hit this square
		
	// a simple sub-divide might help here ... not proper division if /2
	return POSSIBLE_HIT;
}

static int hits_rectangle(const aline& line_in, int min_x, int min_z, int max_x, int max_z)
{
	int result = base_hits_rectangle(line_in, min_x, min_z, max_x, max_z);
	if(result == POSSIBLE_HIT)
	{
		//write_line("possible");
		// a simple sub-divide might help here ... not proper division if /2
		int mid_x = (line_in.start_wx + line_in.end_wx)/2;
		int mid_z = (line_in.start_wz + line_in.end_wz)/2;
		aline split;
		split.start_wx = line_in.start_wx;
		split.start_wz = line_in.start_wz;
		split.end_wx = mid_x;
		split.end_wz = mid_z;
		result = base_hits_rectangle(split, min_x, min_z, max_x, max_z);
		//write_line("first retry =", result);
		if(result != YES_HIT)
		{ // no hit (or only possible for) that segment
			split.start_wx = line_in.end_wx;
			split.start_wz = line_in.end_wz;
			int result2 = base_hits_rectangle(split, min_x, min_z, max_x, max_z);
			//write_line("second retry =", result2);
			// options: possible+yes, possible+possible, possible+no, no+yes, no+possible, no+no
			// two interesting cases are: anything+yes(2 off) and no+no
			if(result2==YES_HIT)
			{	// still couldn't resolve a definate hit
				result = YES_HIT;
			} else if(result == NO_HIT && result2 == NO_HIT)
			{
				result = NO_HIT;
			}
			else	// remainder result for possible+possible, possible+no, no+possible
			{
				result = POSSIBLE_HIT;
			}
		}
	}
	return result;
	
}

bool rectangle_hit(const advanced_line_vector& lines, int min_x, int min_z, int max_x, int max_z)
{
	//const int max_possibles = 20;		// we only record a sub-set of the possible hits. This should be fine.
	//int possibles[max_possibles];
	//count++;
	
	int num = lines.size();
	for(int i=0; i<num; i++)
	{
		const aline& line = lines[i];
		int result = hits_rectangle(line, min_x, min_z, max_x, max_z);
		if(result == YES_HIT)
		{
			return true;
		}
		if(result == POSSIBLE_HIT)
		{
			write_line("Possible Hit? line=",i);
			//write_line("swx=", lines.start_wx);
			//write_line("swz=", lines.start_wz);
			//write_line("ewx=", lines.end_wx);
			//write_line("ewz=", lines.end_wz);
			//write_line("min_x=", min_x)
			
		//	if(count!=max_possibles)
		//	{
		//		possibles[count] == i;
		//		count++;
		//	}
		}
	}
	//if(count == 0) return NO_HIT;
	
	// we need the check the possibles
	//for(int i=0; i<count; i++)
	//{
	//	const aline& line = lines[i];
	//	
	//}
	
	// ignore possible on the assumption we don't care about those
	return false;
}



// X = 0 is column we are in as the robot
// Z = 1 is the first column we will detect because of the camera angle
// RCC = relative cell count, integer offset from the robot in cells
// All left-right (X) walls are left walls
// All forward-back (Z) walls are forward walls
// No north/south/east/west terminology because these walls are local to the robot
class WallDetectorPosition {
public:
	// we would switch this at run-time with seperate classes ... but for various reasons, we don't
	// reason include: one switch on vertical in interpret_lines() below, initialisation of wall_positions[]
	// none particularly compelling but develop fast is highest priority
	int vert_min_x() const;
	int vert_min_z() const;
	int vert_max_x() const;
	int vert_max_z() const;
	int hori_min_x() const;
	int hori_min_z() const;
	int hori_max_x() const;
	int hori_max_z() const;
	int min_x() const;
	int min_z() const;
	int max_x() const;
	int max_z() const;
	bool vertical() const;
	int cell_x() const;
	int cell_z() const;

//private:		// all these are private bits of data ... but we make them public to allow initialisation
	int8_t RCC_x;
	int8_t RCC_z;
	uint8_t vertical_direction;
	int8_t x_offset;
	int8_t z_offset;
};

inline bool WallDetectorPosition::vertical() const
{
	return vertical_direction;
}

//
// Vertical walls have their longest edge running away from us
// Left hand coordinate system for x, y, z. +Y is up, +Z is into the picture and away from the robot - in front of us
// Vertical walls are always considered on the left hand wall, for the purposes of this table.
const int v_detector_start_z = 40;							// 30 in front of cell lowest Z
const int v_detector_end_z = wallLength_in_mm - 40;			// 30 behind cell highest Z
const int v_detector_wall_width = 80;
const int v_detector_start_x = -(v_detector_wall_width/2) - (wallLength_in_mm/2);	// 30 to the left of left-hand wall
const int v_detector_end_x = (v_detector_wall_width/2) - (wallLength_in_mm/2);		// 30 to the right of left-hand wall
inline int WallDetectorPosition::vert_min_x() const
{
	return wallLength_in_mm*RCC_x + x_offset + v_detector_start_x;
}
inline int WallDetectorPosition::vert_max_x() const
{
	return wallLength_in_mm*RCC_x + x_offset + v_detector_end_x;
}
inline int WallDetectorPosition::vert_min_z() const
{
	return wallLength_in_mm*RCC_z + z_offset + v_detector_start_z;
}
inline int WallDetectorPosition::vert_max_z() const
{
	return wallLength_in_mm*RCC_z + z_offset + v_detector_end_z;
}

//
// Horizontal walls have their longest edge running left to right from the robot perspective.
// Horizontal walls are always considered on the furthest wall away from the robot ('local north').
const int h_detector_start_z = wallLength_in_mm - 40;		// 30 behind furthest wall in cell
const int h_detector_end_z = wallLength_in_mm + 40;			// 30 ahead of furthest wall in call
const int h_detector_wall_width = 80;
const int h_detector_start_x = (h_detector_wall_width/2) - (wallLength_in_mm/2);	// 30 to the right of left-hand wall
const int h_detector_end_x = -(h_detector_wall_width/2) + (wallLength_in_mm/2);	// 30 to the left of right-hand wall
inline int WallDetectorPosition::hori_min_x() const
{
	return wallLength_in_mm*RCC_x + x_offset + h_detector_start_x;
}
inline int WallDetectorPosition::hori_max_x() const
{
	return wallLength_in_mm*RCC_x + x_offset + h_detector_end_x;
}
inline int WallDetectorPosition::hori_min_z() const
{
	return wallLength_in_mm*RCC_z + z_offset + h_detector_start_z;
}
inline int WallDetectorPosition::hori_max_z() const
{
	return wallLength_in_mm*RCC_z + z_offset + h_detector_end_z;
}

//
// Switched version
//
int WallDetectorPosition::min_x() const
{
	return vertical_direction ? vert_min_x() : hori_min_x();
}
int WallDetectorPosition::min_z() const
{
	return vertical_direction ? vert_min_z() : hori_min_z();
}
int WallDetectorPosition::max_x() const
{
	return vertical_direction ? vert_max_x() : hori_max_x();
}
int WallDetectorPosition::max_z() const
{
	return vertical_direction ? vert_max_z() : hori_max_z();
}

int WallDetectorPosition::cell_x() const
{
	return RCC_x;
}
int WallDetectorPosition::cell_z() const
{
	return RCC_z;
}

static const WallDetectorPosition wall_positions[] = 
{
// Note: we can't see Z=0 at all because of camera angle
//
//	RCC	RCC	vertical	offset from expected
//	x	z	direction	x		z
	{ 0,	1,	true,		0,		0, },
	{ 0,	2,	true,		0,		0, },
	{ 0,	3,  true,		0,		0, },
	{ 1,	1,	true,		0,		0, },
	{ 1,	2,	true,		0,		0, },
	{ 1,	3,  true,		0,		0, },
	{ -1,	3,  true,		0,		0, },

// horizontal candiates
	{ 0,	1,	false,		0,		0, },
	{ 0,	2,	false,		0,		0, },
	{ 0,	3,	false,		0,		0, },
// column to left
	{ -1,	1,	false,		0,		0, },
	{ -1,	2,	false,		0,		0, },
// column to right
	{ 1,	1,	false,		0,		0, },
	{ 1,	2,	false,		0,		0, },
};


void interpret_lines(const advanced_line_vector& lines, LocalMapInterface& map, int robot_x_mm_in_cell, int robot_z_mm_in_cell)
{
	const int num_entries = sizeof(wall_positions)/sizeof(WallDetectorPosition);
	const WallDetectorPosition* wp = wall_positions;
	for (int i=0; i< num_entries; i++)
	{
		if(wp->vertical())
		{
			int min_x = wp->vert_min_x() - robot_x_mm_in_cell;
			int min_z = wp->vert_min_z() - robot_z_mm_in_cell;
			int max_x = wp->vert_max_x() - robot_x_mm_in_cell;
			int max_z = wp->vert_max_z() - robot_z_mm_in_cell;
			bool hit = rectangle_hit(lines, min_x, min_z, max_x, max_z);
			
			if(hit) { map.set_left_wall(wp->cell_x(), wp->cell_z()); }
		}
		else // horizontal
		{
			int min_x = wp->hori_min_x() - robot_x_mm_in_cell;
			int min_z = wp->hori_min_z() - robot_z_mm_in_cell;
			int max_x = wp->hori_max_x() - robot_x_mm_in_cell;
			int max_z = wp->hori_max_z() - robot_z_mm_in_cell;
			bool hit = rectangle_hit(lines, min_x, min_z, max_x, max_z);
			
			if(hit) { map.set_forward_wall(wp->cell_x(), wp->cell_z()); }
		}
		
		wp++;
	}
	
}

//
// Functions to allow read-only acces
//
int num_detection_boxes()
{
	return sizeof(wall_positions)/sizeof(WallDetectorPosition);
}

void get_detection_box(detection_box_rect& output, int index, int robot_x_mm_in_cell, int robot_z_mm_in_cell)
{
	if(index < 0 || index >= num_detection_boxes())
	{
		halt("Detection box index out of range");
	}
	output.cell_x = wall_positions[index].cell_x(); 
	output.cell_z = wall_positions[index].cell_z();

	output.min_x = wall_positions[index].min_x() - robot_x_mm_in_cell;
	output.min_z = wall_positions[index].min_z() - robot_z_mm_in_cell;
	output.max_x = wall_positions[index].max_x() - robot_x_mm_in_cell;
	output.max_z = wall_positions[index].max_z() - robot_z_mm_in_cell;
}


//#define WALL_DETECTOR_UNIT_TESTS_BUILT
#ifdef WALL_DETECTOR_UNIT_TESTS_BUILT
static bool hits_rect_fails(int x1, int z1, int x2, int z2, int expected)
{
	aline line;
	line.start_wx = x1; line.start_wz  = z1; line.end_wx = x2; line.end_wz = z2;
	if(hits_rectangle(line, 100, 100, 200, 200) != expected) { 
		return true; 
	}
	return false;
}

static bool outcode_fails(int x, int z, int expected)
{
	int result = outcode(x, z, 100, 100, 200, 200);
	if(result != expected) { 
		write_string("outcode=",result);
		write_line(" expected=",expected);
		return true; 
	}
	return false;
}
#endif

int wall_detector_unit_tests()
{
#ifdef WALL_DETECTOR_UNIT_TESTS_BUILT
	// top row
	if(outcode_fails(0, 0, TOP+LEFT)) return 1;
	if(outcode_fails(150, 0, TOP)) return 2;
	if(outcode_fails(300, 0, TOP+RIGHT)) return 3;
	// middle row
	if(outcode_fails(0, 150, LEFT)) return 4;
	if(outcode_fails(150, 150, 0)) return 5;
	if(outcode_fails(300, 150, RIGHT)) return 6;
	// bottom row
	if(outcode_fails(0, 300, BOTTOM+LEFT)) return 7;
	if(outcode_fails(150, 300, BOTTOM)) return 8;
	if(outcode_fails(300, 300, BOTTOM+RIGHT)) return 9;
	//
	// hits_rect tests
	//
	if(hits_rect_fails(0, 0, 80, 80, NO_HIT)) return 101;		// top-left
	if(hits_rect_fails(110, 110, 180, 180, YES_HIT)) return 102;	// center
	if(hits_rect_fails(110, 0, 180, 300, YES_HIT)) return 103;		// top center to bottom center
	if(hits_rect_fails(0, 150, 180, 180, YES_HIT)) return 104;		// left to center
	if(hits_rect_fails(150, 150, 180, 180, YES_HIT)) return 104;		// center to right center
#define top_left 0, 0
#define top 150, 0
#define top_right 300, 0
#define left 0, 150
#define center 150, 150
#define right 300, 150
#define bottom_left 0, 300
#define bottom 150, 300
#define bottom_right 300, 300
	if(hits_rect_fails(bottom_left, bottom_right, NO_HIT)) return 105;
	if(hits_rect_fails(center, bottom_right, YES_HIT)) return 106;
	if(hits_rect_fails(top_left, center, YES_HIT)) return 107;
	if(hits_rect_fails(bottom, bottom_right, NO_HIT)) return 108;

	// with a simple algorithm these will all be possible bits ... the divide by 2 makes them definates
	if(hits_rect_fails(bottom_left, top_right, YES_HIT)) return 200;
	if(hits_rect_fails(top_left, bottom_right, YES_HIT)) return 201;
	if(hits_rect_fails(99, 150, 150, 99, YES_HIT)) return 202;
	if(hits_rect_fails(0, 101, 101, 0, NO_HIT)) return 203;	// interesting that this one is resolved as definately no
	if(hits_rect_fails(0, 0, 300, 500, POSSIBLE_HIT)) return 204;	// center point = 150, 250 ... cannot resolve
	return 0;
#else
	return -1000;
#endif
}
