/*
 *  Real_World.cpp
 *  red_extract_cocoa
 *
 *  Created by Rob Probin on 18/01/2009.
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

#include "Real_World.h"
#include "Zw_calculate.h"
#include "sine_cosine.h"

#if !defined(WORLD_RELATED_CONSTANTS_AS_CONSTANTS) && !defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)

double pixel_size_in_mm = 15.2e-3;
int cameraHeight_in_mm = 185;
int wallHeight_in_mm = 50;

int cameraHeight_in_pixels = 185/pixel_size_in_mm;
int wallHeight_in_pixels = 50/pixel_size_in_mm;				// should be in pixels

int wallHeightfromCamera_in_pixels = cameraHeight_in_pixels-wallHeight_in_pixels;	// this is Yw, and equals 135mm

int wallLength_in_mm = 180;
int wallThickness_in_mm = 12;
int wallLength_in_pixels = wallLength_in_mm/pixel_size_in_mm;
int wallThickness_in_pixels = wallThickness_in_mm/pixel_size_in_mm;


int center_of_image_X = (processed_image_width_in_pixels/2); // + 20;					// should be width/2 ... but isn't
int center_of_image_Y = (processed_image_height_in_pixels/2);

double raw_image_to_eyepoint_in_pixels = 386.27416998;			// D = 160 / tan (45/2) = 386.27416998 pixels
double raw_image_to_eyepoint_in_mm = raw_image_to_eyepoint_in_pixels*pixel_size_in_mm;

int image_to_eyepoint_D_in_pixels = raw_image_to_eyepoint_in_pixels;

// Cos(A)			Sin(A) for angle of 25 degrees.
// 0.90630794279	0.42261792772
// Camera to wall = -135
// 
//const double CosA = 0.90630794279;		// for 25 degree look down from Horizon
//const double SinA = 0.42261792772;
double CosA = 0.42261792772;		// for 90-25 degree look down from Horizon
double SinA = 0.90630794279;

int wallHeightfromCamera_in_mm = cameraHeight_in_mm-wallHeight_in_mm;

int scale_factor = 1024;		// to allow calculation to proceed using fixed point arithmetic
									// at 1m this gives 173342720 as max value (160*1058x1024)
									// doesn't have to be power of 2 since we require a proper divide anyway
									// Although theoretically we could avoid this divide if this*D = power of 2
									// and use a shift instead.
int Yw_multiplied_SinA_scaled_in_mm = -(SinA * wallHeightfromCamera_in_mm * scale_factor);
int CosA_scaled = CosA * scale_factor;
int image_to_eyepoint_D_scaled_in_mm = raw_image_to_eyepoint_in_mm * scale_factor;
int image_to_eyepoint_D_scaled_in_mm_div_pixel_size = (raw_image_to_eyepoint_in_mm * scale_factor)/pixel_size_in_mm;

#elif defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)

int center_of_image_X;					// should be width/2 ... but isn't

int CosA_scaled;
int SinA_scaled;

int Yw_multiplied_SinA_in_um;

#endif

class Real_World_Constants {
public:
	Real_World_Constants();
	void update_read_world_constants(int camera_angle, int x_offset);
private:
	void update();
	int RWC_camera_angle; int RWC_x_offset;
};

static Real_World_Constants my_copy;


Real_World_Constants::Real_World_Constants()
: RWC_camera_angle(205), RWC_x_offset(196)
{
	update();
}

void Real_World_Constant_Update(int camera_angle, int x_offset)
{
	my_copy.update_read_world_constants(camera_angle, x_offset);
}


void Real_World_Constants::update_read_world_constants(int camera_angle, int x_offset)
{
	RWC_camera_angle = camera_angle;
	RWC_x_offset = x_offset;
	update();
}

#if !defined(WORLD_RELATED_CONSTANTS_AS_CONSTANTS) && !defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)
#include <cmath>
using std::cos;
using std::sin;
#endif

void Real_World_Constants::update()
{
#if !defined(WORLD_RELATED_CONSTANTS_AS_CONSTANTS) && !defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)
	pixel_size_in_mm = 15.2e-3;
	cameraHeight_in_mm = 185;
	wallHeight_in_mm = 50 + RWC_wall_height_adjustment;
	
	cameraHeight_in_pixels = 185/pixel_size_in_mm;
	wallHeight_in_pixels = 50/pixel_size_in_mm;				// should be in pixels
	
	wallHeightfromCamera_in_pixels = cameraHeight_in_pixels-wallHeight_in_pixels;	// this is Yw, and equals 135mm
	
	wallLength_in_mm = 180;
	wallThickness_in_mm = 12;
	wallLength_in_pixels = wallLength_in_mm/pixel_size_in_mm;
	wallThickness_in_pixels = wallThickness_in_mm/pixel_size_in_mm;
	
	
	center_of_image_X = 196; //(processed_image_width_in_pixels/2); // + 20;					// should be width/2 ... but isn't
	center_of_image_Y = (processed_image_height_in_pixels/2);
	
	raw_image_to_eyepoint_in_pixels = 393; // 386.27416998 + RWC_d_offset;			// D = 160 / tan (45/2) = 386.27416998 pixels
	raw_image_to_eyepoint_in_mm = raw_image_to_eyepoint_in_pixels*pixel_size_in_mm;
	
	image_to_eyepoint_D_in_pixels = raw_image_to_eyepoint_in_pixels;
	
	if(RWC_camera_angle_adjust == 0)
	{
		// Cos(A)			Sin(A) for angle of 25 degrees.
		// 0.90630794279	0.42261792772
		// Camera to wall = -135
		// 
		//const double CosA = 0.90630794279;		// for 25 degree look down from Horizon
		//const double SinA = 0.42261792772;
		CosA = 0.42261792772;		// for 90-25 degree look down from Horizon
		SinA = 0.90630794279;
	}
	else
	{
		double angle_degrees = (90-25)+RWC_camera_angle_adjust;
		double angle_radians = (angle_degrees/180) * 3.141592653589793238462643;
		CosA = cos(angle_radians);
		SinA = sin(angle_radians);
	}
	{
	//double angle_degrees = (90-20.5)+RWC_camera_angle_adjust;
	double angle_degrees = (20.5)+RWC_camera_angle_adjust;
	double angle_radians = (angle_degrees/180) * 3.141592653589793238462643;
	CosA = cos(angle_radians);
	SinA = sin(angle_radians);
	}
	
	wallHeightfromCamera_in_mm = cameraHeight_in_mm-wallHeight_in_mm;
	
	scale_factor = 1024;		// to allow calculation to proceed using fixed point arithmetic
	// at 1m this gives 173342720 as max value (160*1058x1024)
	// doesn't have to be power of 2 since we require a proper divide anyway
	// Although theoretically we could avoid this divide if this*D = power of 2
	// and use a shift instead.
	Yw_multiplied_SinA_scaled_in_mm = -(SinA * wallHeightfromCamera_in_mm * scale_factor);
	CosA_scaled = CosA * scale_factor;
	image_to_eyepoint_D_scaled_in_mm = raw_image_to_eyepoint_in_mm * scale_factor;
	image_to_eyepoint_D_scaled_in_mm_div_pixel_size = (raw_image_to_eyepoint_in_mm * scale_factor)/pixel_size_in_mm;	

	update_Zw_table();
#elif defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)
	center_of_image_X = RWC_x_offset; //(processed_image_width_in_pixels/2); // + 20;					// should be width/2 ... but isn't

	int camera_angle_in_tenth_degrees = RWC_camera_angle;
	int trig_angle = convert_to_trig_angle_from_tenths(camera_angle_in_tenth_degrees);
	CosA_scaled = fast_cosine(trig_angle);
	SinA_scaled = fast_sine(trig_angle);
	void write_line(const char*, int);
	write_line("Center of Image X>>", center_of_image_X);
	write_line("CAM ANGLE>>", camera_angle_in_tenth_degrees);
	write_line("TRIG_ANGLE>>", trig_angle);
	write_line("COS A>>", CosA_scaled);
	write_line("SIN A>>", SinA_scaled);

	Yw_multiplied_SinA_in_um = -(SinA_scaled * wallHeightfromCamera_in_mm);		// max +/-1000 and 135 = +/-135,000 (scaled by 1000 and 1)

	update_Zw_table();
#endif
}



