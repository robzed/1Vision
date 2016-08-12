// arctan.cpp
// Created by Rob Probin 28th March 2009
/*
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
#include "arctan.h"
#include "language_support.h"

// Arctan table
// Generated automatically by 'write_arctan_table.py'
// Number of Entries = 513, Scale Factor = 2048
const int number_of_positive_values = 256;
const int scale_factor = arctan_full_circle_angle;
const short arctan_table[2*number_of_positive_values+1] = {
-256, -255, -254, -254, -253, -252, -252, -251, -250, -250, -249, -248, -248, -247, -246, -246, 	// indexes = 0 to 15
-245, -244, -244, -243, -242, -242, -241, -240, -239, -239, -238, -237, -237, -236, -235, -235, 	// indexes = 16 to 31
-234, -233, -232, -232, -231, -230, -229, -229, -228, -227, -226, -226, -225, -224, -223, -223, 	// indexes = 32 to 47
-222, -221, -220, -220, -219, -218, -217, -216, -216, -215, -214, -213, -212, -212, -211, -210, 	// indexes = 48 to 63
-209, -208, -208, -207, -206, -205, -204, -203, -203, -202, -201, -200, -199, -198, -198, -197, 	// indexes = 64 to 79
-196, -195, -194, -193, -192, -191, -191, -190, -189, -188, -187, -186, -185, -184, -183, -182, 	// indexes = 80 to 95
-182, -181, -180, -179, -178, -177, -176, -175, -174, -173, -172, -171, -170, -169, -168, -167, 	// indexes = 96 to 111
-167, -166, -165, -164, -163, -162, -161, -160, -159, -158, -157, -156, -155, -154, -153, -152, 	// indexes = 112 to 127
-151, -150, -149, -148, -147, -145, -144, -143, -142, -141, -140, -139, -138, -137, -136, -135, 	// indexes = 128 to 143
-134, -133, -132, -131, -130, -129, -127, -126, -125, -124, -123, -122, -121, -120, -119, -118, 	// indexes = 144 to 159
-116, -115, -114, -113, -112, -111, -110, -109, -107, -106, -105, -104, -103, -102, -101, -99, 	// indexes = 160 to 175
-98, -97, -96, -95, -94, -92, -91, -90, -89, -88, -87, -85, -84, -83, -82, -81, 	// indexes = 176 to 191
-79, -78, -77, -76, -75, -73, -72, -71, -70, -68, -67, -66, -65, -64, -62, -61, 	// indexes = 192 to 207
-60, -59, -57, -56, -55, -54, -53, -51, -50, -49, -48, -46, -45, -44, -43, -41, 	// indexes = 208 to 223
-40, -39, -38, -36, -35, -34, -32, -31, -30, -29, -27, -26, -25, -24, -22, -21, 	// indexes = 224 to 239
-20, -19, -17, -16, -15, -13, -12, -11, -10, -8, -7, -6, -5, -3, -2, -1, 	// indexes = 240 to 255
0,	// index = 256
1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 19, 20, 	// indexes = 257 to 272
21, 22, 24, 25, 26, 27, 29, 30, 31, 32, 34, 35, 36, 38, 39, 40, 	// indexes = 273 to 288
41, 43, 44, 45, 46, 48, 49, 50, 51, 53, 54, 55, 56, 57, 59, 60, 	// indexes = 289 to 304
61, 62, 64, 65, 66, 67, 68, 70, 71, 72, 73, 75, 76, 77, 78, 79, 	// indexes = 305 to 320
81, 82, 83, 84, 85, 87, 88, 89, 90, 91, 92, 94, 95, 96, 97, 98, 	// indexes = 321 to 336
99, 101, 102, 103, 104, 105, 106, 107, 109, 110, 111, 112, 113, 114, 115, 116, 	// indexes = 337 to 352
118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 129, 130, 131, 132, 133, 134, 	// indexes = 353 to 368
135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 147, 148, 149, 150, 151, 	// indexes = 369 to 384
152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 	// indexes = 385 to 400
167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 	// indexes = 401 to 416
182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 191, 192, 193, 194, 195, 196, 	// indexes = 417 to 432
197, 198, 198, 199, 200, 201, 202, 203, 203, 204, 205, 206, 207, 208, 208, 209, 	// indexes = 433 to 448
210, 211, 212, 212, 213, 214, 215, 216, 216, 217, 218, 219, 220, 220, 221, 222, 	// indexes = 449 to 464
223, 223, 224, 225, 226, 226, 227, 228, 229, 229, 230, 231, 232, 232, 233, 234, 	// indexes = 465 to 480
235, 235, 236, 237, 237, 238, 239, 239, 240, 241, 242, 242, 243, 244, 244, 245, 	// indexes = 481 to 496
246, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 254, 254, 255, 256, 	// indexes = 497 to 512
};
const short * const arctan_table_center = &arctan_table[number_of_positive_values];


int fast_arctan(int numerator, int demoninator)
{
	int angle = 0x0000feed;
	
	if(demoninator == 0)
	{
		// if numerator and demoninator are both 0, we have problems anyway, just return -90 degrees anyway!
		// otherwise -90 degrees
		angle = -scale_factor/4;
	}
	else if(abs(numerator) < abs(demoninator)) // this means numerator/demoninator between -1 and 1
	{
		int ratio = (numerator*number_of_positive_values) / demoninator;
		angle = arctan_table[number_of_positive_values+ratio];
	}
	else // abs(demoninator) < abs(numerator)
	{
		int ratio = (demoninator*number_of_positive_values) / numerator;
		if(ratio > 0)
		{
			angle = (scale_factor/4) - arctan_table[number_of_positive_values+ratio];
		}
		else // ratio < 0
		{
			angle = -(scale_factor/4) - arctan_table[number_of_positive_values+ratio];
		}
	}
	
	return angle;
}

#include "pc_uart.h"

static int arctan_single_test(int p1, int p2, int expected_result_in_degrees)
{
	int result = fast_arctan(p1, p2);
	result = (result * 360)/scale_factor;
	int expected = expected_result_in_degrees;
	if(result != expected)
	{
		write_string("Arctan test (", p1); write_string(" ", p2); write_string(" expected(deg)=", expected_result_in_degrees); write_line(" result(deg)=",  result);
		return 1;
	}
	
	return 0;
}
void arctan_tests()
{
	// vertical/horizontal line tests
	int failures = 0;
	failures += arctan_single_test(0, 10, 0);
	failures += arctan_single_test(10, 0, -90);
	failures += arctan_single_test(0, -10, 0);
	failures += arctan_single_test(-10, 0, -90);
	
	// twelve angles
	failures += arctan_single_test(1, 10, 5);
	failures += arctan_single_test(-1, 10, -5);
	failures += arctan_single_test(1, -10, -5);
	failures += arctan_single_test(-1, -10, 5);

	failures += arctan_single_test(2, 10, 11);
	failures += arctan_single_test(-2, 10, -11);
	failures += arctan_single_test(2, -10, -11);
	failures += arctan_single_test(-2, -10, 11);
	
	failures += arctan_single_test(10, 1, 84);
	failures += arctan_single_test(-10, 1, -84);
	failures += arctan_single_test(10, -1, -84);
	failures += arctan_single_test(-10, -1, 84);
	
	// +/-45 degrees
	failures += arctan_single_test(10, 10, 45);
	failures += arctan_single_test(10, -10, -45);
	failures += arctan_single_test(-10, 10, -45);
	failures += arctan_single_test(-10, -10, 45);

	if(scale_factor != arctan_full_circle_angle)
	{
		failures += 1;
		write_line("arctan: H/S angles");
	}
	
	if(failures)
	{
		write_line("ARCTAN FAILURES=", failures);
	}
	//else
	//{
	//	write_line("Arctan tests pass");
	//}
	
}


