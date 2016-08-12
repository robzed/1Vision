/*
 *  reverse_perspective_projection.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 24/11/2008.
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

#include "reverse_perspective_projection.h"
#include "Real_World.h"
#include "Zw_calculate.h"

//
// Copied from RedExtract
//
// TODO: Clean up comments and just leave valid ones!
// 
static void convert_to_map_coords(int x_image, int y_image, int* x_world_rel_robot, int* z_world_rel_robot)
{
	//const int HorizonYp = 0;			// in pixels from top of frame
	
	
	//const int NearPlaneHighY = wallHeightfromCamera; //wallHeight;	// in pixels from top of frame
	//const int Y = NearPlaneHighY;
	//int D = image_to_eyepoint_D;
	//D += d_offset;
	
	//y_image -= (HorizonYp + horizon_offset);			// adjust y image to take account of vanishing point ('horizon')
	//x_image -= center_of_image_X;
	
	// process Yp
	//if(y_image <= HorizonYp)
	//{
	// cannot be part of the maze, it's above the horizon
	//	return false;
	//}
	
	int Zw;
	//if(d_offset)
	//{
	//	y_image+=10+ horizon_offset;
	//	*z_world_rel_robot = ((wallHeightfromCamera_in_mm * raw_image_to_eyepoint_in_mm) / (y_image*pixel_size_in_mm)) - raw_image_to_eyepoint_in_mm;
	//}
	//*z_world_rel_robot = ((Y * D) / y_image) - D;
	// it's actually more complex than this ... see the spreadsheet "reverse_pers_z_calculation"
	//
	// The formula we are using is Zw = (YpD - YpYwS - DYwS) / (DS - YpC)
	// {{{should one of the Sin be Cos? i.e. (Yp*D-D*Yw*C-Yp*Yw*S) / (D*S - Yp*C) }}}
	// Notice we know everything at this point and the only variable is Yp.
	// We therefore have encoded the whole formula into a look-up table with Yp (0 to 239) as the input and Zw as the output.
	//
	// It is not dependant on Xp.
	//else
	{
		Zw = get_Zw_in_mm(y_image);
		*z_world_rel_robot = Zw;
	}
	
	// process Xp
	// This is easier ...
	// X directly relates to Xp but with Yp higher on image (less Yp in our case with origin at top, left)
	// then it 
	// X = Xp*Y / Yp
	
	
	//if(d_offset)
	//{
	//	x_image -= center_of_image_X;
	//	*x_world_rel_robot = (x_image/pixel_size_in_mm) * wallHeightfromCamera_in_mm / (y_image/pixel_size_in_mm);
	//}
	// *x_world_rel_robot = x_image * Y / y_image;
	// it's actually more complicated than the formula above would suggest, since this doesn't contain the 
	// rotation around the x axis due to the camera pointing down 25 degrees.
	//
	// The formula we are using is Xw = Xp ((Zw * C - Yw * S) + D ) / D
	// Notice: the variables in this case are: Zw and Xp. 
	// Since we have just calculated Zw and have Xp we can use this. Xp must be centeralised to the center of the image.
	//
	//else
#ifndef WORLD_RELATED_CALCULATION_AS_INTEGERS
	{
		x_image -= center_of_image_X;
		
		int Xw = x_image * ((Zw*CosA_scaled - Yw_multiplied_SinA_scaled_in_mm) + image_to_eyepoint_D_scaled_in_mm);
		//cout << x_image << " " << CosA_scaled << " " << Yw_multiplied_SinA_scaled_in_mm << " " << image_to_eyepoint_D_scaled_in_mm << " " << Xw << endl;
		Xw = Xw / image_to_eyepoint_D_scaled_in_mm_div_pixel_size;	// div pixel size to compensate for x_image in pixels
		//cout << image_to_eyepoint_D_scaled_in_mm_div_pixel_size << endl;
		*x_world_rel_robot = Xw;
	}
#else
	{
		x_image -= center_of_image_X;
		
		int Xw = x_image * ((Zw*CosA_scaled - Yw_multiplied_SinA_in_um) + image_to_eyepoint_D_in_um); // max = 320 * ((997 * +/-1000  -  +/-135,000) + 5867) = 364,117,440 scaled by 1000
		//cout << x_image << " " << CosA_scaled << " " << Yw_multiplied_SinA_scaled_in_mm << " " << image_to_eyepoint_D_scaled_in_mm << " " << Xw << endl;
		Xw = Xw / image_to_eyepoint_D_in_um_div_pixel_size_mm;	// div pixel size to compensate for x_image in pixels   max=386,000, scaled by 1000
		//cout << image_to_eyepoint_D_scaled_in_mm_div_pixel_size << endl;
		*x_world_rel_robot = Xw;
	}
#endif
	
}

void make_lines_plan_view(line_list_custom_vector& lines)
{
	int size = lines.size();
	if(size == 0) return;	// nothing to do
	
	line* lines_p= &(lines[0]);		// assumption about storage

	for(int i=0; i < size; i++)
	{
		int x1 = lines_p->start.x;
		int y1 = lines_p->start.y;
		int x2 = lines_p->end.x;
		int y2 = lines_p->end.y;
		
		int xw1, zw1, xw2, zw2;
		convert_to_map_coords(x1, y1, &xw1, &zw1);
		convert_to_map_coords(x2, y2, &xw2, &zw2);
		
		// conversion to 2d not needed - will be done in RealBasic app
		//int iy1, iy2, ix1, ix2;
		//convert_to_2d(xw1, zw1, &ix1, &iy1);
		//convert_to_2d(xw2, zw2, &ix2, &iy2);
		//draw(ix1,iy1,ix2,iy2, carray2[colour_index%8], carray2[colour_index%8]);
		
		// write these back into the vector
		lines_p->start.x = xw1;
		lines_p->start.y = zw1;
		
		lines_p->end.x = xw2;
		lines_p->end.y = zw2;
		
		lines_p++;
	}
}

void make_lines_plan_view(advanced_line_vector& lines)
{
	int size = lines.size();
	if(size == 0) return;	// nothing to do
	
	aline* lines_p= &(lines[0]);		// assumption about storage
	
	for(int i=0; i < size; i++)
	{
		int x1 = lines_p->start_x;
		int y1 = lines_p->start_y;
		int x2 = lines_p->end_x;
		int y2 = lines_p->end_y;
		
		int xw1, zw1, xw2, zw2;
		convert_to_map_coords(x1, y1, &xw1, &zw1);
		convert_to_map_coords(x2, y2, &xw2, &zw2);
		
		// conversion to 2d not needed - will be done in RealBasic app
		//int iy1, iy2, ix1, ix2;
		//convert_to_2d(xw1, zw1, &ix1, &iy1);
		//convert_to_2d(xw2, zw2, &ix2, &iy2);
		//draw(ix1,iy1,ix2,iy2, carray2[colour_index%8], carray2[colour_index%8]);
		
		// write these back into the vector
		lines_p->start_wx = xw1;
		lines_p->start_wz = zw1;
		
		lines_p->end_wx = xw2;
		lines_p->end_wz = zw2;
		
		lines_p++;
	}
}

