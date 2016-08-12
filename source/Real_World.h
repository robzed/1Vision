/*
 *  Real_World.h
 *  red_extract_cocoa
 *
 *  Created by Rob Probin on 12/05/2008.
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

#ifndef REAL_WORLD_H
#define REAL_WORLD_H

const int processed_image_width_in_pixels = 320;
const int processed_image_height_in_pixels = 240;

//
// these two options select the type of adjustments we can do at run-time
//
//#define WORLD_RELATED_CONSTANTS_AS_CONSTANTS
#define WORLD_RELATED_CALCULATION_AS_INTEGERS



#ifdef WORLD_RELATED_CONSTANTS_AS_CONSTANTS


const double pixel_size_in_mm = 15.2e-3;
const int cameraHeight_in_mm = 185;
const int wallHeight_in_mm = 50;

const int cameraHeight_in_pixels = 185/pixel_size_in_mm;
const int wallHeight_in_pixels = 50/pixel_size_in_mm;				// should be in pixels

const int wallHeightfromCamera_in_pixels = cameraHeight_in_pixels-wallHeight_in_pixels;	// this is Yw, and equals 135mm

const int wallLength_in_mm = 180;
const int wallThickness_in_mm = 12;
const int wallLength_in_pixels = wallLength_in_mm/pixel_size_in_mm;
const int wallThickness_in_pixels = wallThickness_in_mm/pixel_size_in_mm;


const int center_of_image_X = (processed_image_width_in_pixels/2); // + 20;					// should be width/2 ... but isn't
const int center_of_image_Y = (processed_image_height_in_pixels/2);

const double raw_image_to_eyepoint_in_pixels = 386.27416998;			// D = 160 / tan (45/2) = 386.27416998 pixels
const double raw_image_to_eyepoint_in_mm = raw_image_to_eyepoint_in_pixels*pixel_size_in_mm;

const int image_to_eyepoint_D_in_pixels = raw_image_to_eyepoint_in_pixels;

// Cos(A)			Sin(A) for angle of 25 degrees.
// 0.90630794279	0.42261792772
// Camera to wall = -135
// 
//const double CosA = 0.90630794279;		// for 25 degree look down from Horizon
//const double SinA = 0.42261792772;
const double CosA = 0.42261792772;		// for 90-25 degree look down from Horizon
const double SinA = 0.90630794279;

const int wallHeightfromCamera_in_mm = cameraHeight_in_mm-wallHeight_in_mm;

const int scale_factor = 1024;		// to allow calculation to proceed using fixed point arithmetic
									// at 1m this gives 173342720 as max value (160*1058x1024)
									// doesn't have to be power of 2 since we require a proper divide anyway
									// Although theoretically we could avoid this divide if this*D = power of 2
									// and use a shift instead.
const int Yw_multiplied_SinA_scaled_in_mm = -(SinA * wallHeightfromCamera_in_mm * scale_factor);
const int CosA_scaled = CosA * scale_factor;
const int image_to_eyepoint_D_scaled_in_mm = raw_image_to_eyepoint_in_mm * scale_factor;
const int image_to_eyepoint_D_scaled_in_mm_div_pixel_size = (raw_image_to_eyepoint_in_mm * scale_factor)/pixel_size_in_mm;


#elif !defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)

extern double pixel_size_in_mm; // = 15.2e-3;
extern int cameraHeight_in_mm; // = 185;
extern int wallHeight_in_mm; // = 50;

extern int cameraHeight_in_pixels; // = 185/pixel_size_in_mm;
extern int wallHeight_in_pixels; // = 50/pixel_size_in_mm;				// should be in pixels

extern int wallHeightfromCamera_in_pixels; // = cameraHeight_in_pixels-wallHeight_in_pixels;	// this is Yw, and equals 135mm

extern int wallLength_in_mm; // = 180;
extern int wallThickness_in_mm; // = 12;
extern int wallLength_in_pixels; // = wallLength_in_mm/pixel_size_in_mm;
extern int wallThickness_in_pixels; // = wallThickness_in_mm/pixel_size_in_mm;


extern int center_of_image_X; // = (processed_image_width_in_pixels/2); // + 20;					// should be width/2 ... but isn't
extern int center_of_image_Y; // = (processed_image_height_in_pixels/2);

extern double raw_image_to_eyepoint_in_pixels; // = 386.27416998;			// D = 160 / tan (45/2) = 386.27416998 pixels
extern double raw_image_to_eyepoint_in_mm; // = raw_image_to_eyepoint_in_pixels*pixel_size_in_mm;

extern int image_to_eyepoint_D_in_pixels; // = raw_image_to_eyepoint_in_pixels;

// Cos(A)			Sin(A) for angle of 25 degrees.
// 0.90630794279	0.42261792772
// Camera to wall = -135
// 
//extern double CosA = 0.90630794279;		// for 25 degree look down from Horizon
//extern double SinA = 0.42261792772;
extern double CosA; // = 0.42261792772;		// for 90-25 degree look down from Horizon
extern double SinA; // = 0.90630794279;

extern int wallHeightfromCamera_in_mm; // = cameraHeight_in_mm-wallHeight_in_mm;

extern int scale_factor; // = 1024;		// to allow calculation to proceed using fixed point arithmetic
// at 1m this gives 173342720 as max value (160*1058x1024)
// doesn't have to be power of 2 since we require a proper divide anyway
// Although theoretically we could avoid this divide if this*D = power of 2
// and use a shift instead.
extern int Yw_multiplied_SinA_scaled_in_mm; // = -(SinA * wallHeightfromCamera_in_mm * scale_factor);
extern int CosA_scaled; // = CosA * scale_factor;
extern int image_to_eyepoint_D_scaled_in_mm; // = raw_image_to_eyepoint_in_mm * scale_factor;
extern int image_to_eyepoint_D_scaled_in_mm_div_pixel_size; // = (raw_image_to_eyepoint_in_mm * scale_factor)/pixel_size_in_mm;

#elif defined(WORLD_RELATED_CALCULATION_AS_INTEGERS)

//const double pixel_size_in_mm = 15.2e-3;
//const int pixel_size_in_nm = 15200;			// 0.0152mm = 15.2 um = 15200 nm
const int pixel_size_in_xm = 152;			// 0.0152mm = 15.2 um = 15200 nm

const int cameraHeight_in_mm = 185;
const int wallHeight_in_mm = 50;

//const int cameraHeight_in_pixels = 185/pixel_size_in_mm;
//const int wallHeight_in_pixels = 50/pixel_size_in_mm;				// should be in pixels

//const int wallHeightfromCamera_in_pixels = cameraHeight_in_pixels-wallHeight_in_pixels;	// this is Yw, and equals 135mm

const int wallLength_in_mm = 180;
const int wallThickness_in_mm = 12;
//const int wallLength_in_pixels = wallLength_in_mm/pixel_size_in_mm;
//const int wallThickness_in_pixels = wallThickness_in_mm/pixel_size_in_mm;

extern int center_of_image_X; // = (processed_image_width_in_pixels/2); // + 20;					// should be width/2 ... but isn't
const int center_of_image_Y = (processed_image_height_in_pixels/2);

//const double raw_image_to_eyepoint_in_pixels = 386.27416998;			// D = 160 / tan (45/2) = 386.27416998 pixels
const int raw_image_to_eyepoint_in_pixels = 386;			// NOTE: If you alter this value you need to check that integer calculations don't overflow!!!

//const double raw_image_to_eyepoint_in_mm = raw_image_to_eyepoint_in_pixels*pixel_size_in_mm;
const int raw_image_to_eyepoint_in_um = (raw_image_to_eyepoint_in_pixels * pixel_size_in_xm)/10;

//const int image_to_eyepoint_D_in_pixels = raw_image_to_eyepoint_in_pixels;

// Cos(A)			Sin(A) for angle of 25 degrees.
// 0.90630794279	0.42261792772
// Camera to wall = -135
// 
//extern double CosA = 0.90630794279;		// for 25 degree look down from Horizon
//extern double SinA = 0.42261792772;
extern int CosA_scaled; // = CosA * scale_factor;
extern int SinA_scaled; // = SinA * scale_factor;

const int wallHeightfromCamera_in_mm = cameraHeight_in_mm-wallHeight_in_mm;

//const int scale_factor = 1000;		// to allow calculation to proceed using fixed point arithmetic

// at 1m this gives 173342720 as max value (160*1058x1024)
// doesn't have to be power of 2 since we require a proper divide anyway
// Although theoretically we could avoid this divide if this*D = power of 2
// and use a shift instead.
extern int Yw_multiplied_SinA_in_um; // = -(SinA * wallHeightfromCamera_in_mm * scale_factor);

//extern int CosA_scaled; // = CosA * scale_factor;

//const int image_to_eyepoint_D_scaled_in_mm = raw_image_to_eyepoint_in_mm * scale_factor;
//const int image_to_eyepoint_D_scaled_in_mm_div_pixel_size = (raw_image_to_eyepoint_in_mm * scale_factor)/pixel_size_in_mm;
const int image_to_eyepoint_D_in_um = raw_image_to_eyepoint_in_um;
const int image_to_eyepoint_D_in_um_div_pixel_size_mm = raw_image_to_eyepoint_in_pixels*1000; // (raw_image_to_eyepoint_in_um)/pixel_size_in_mm;

#endif

void Real_World_Constant_Update(int camera_angle, int x_offset);

#endif
