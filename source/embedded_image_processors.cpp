/*
 *  embedded_image_processors.cpp
 *  red_extract_cocoa
 *
 *  Created by Rob Probin on 19/06/2008.
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

#include "embedded_image_processors.h"

#ifdef RED_EXTRACT_BUILD
#include <iostream>
using namespace std;

void PrintAlert(const char* message);
void PrintAlert(const char* message, int arg_value);
void PrintAlert(const char* message, int arg_val1, int arg_val2);

void halt(char* halt_message)
{
	PrintAlert(halt_message);
	exit(-1);
}
void write_line(const char* message)
{
	printf("%s\n",message);
}
void write_line(const char* message, int i)
{
	printf("%s%i\n",message, i);
}
#else
#include "language_support.h"
#include "pc_uart.h"
#endif



class Single_Segment;
class Single_Line_embedded {
public:
	Single_Segment *line;
	int number() { return number_of_segments_in_this_line; }
	
	//void set_first_segment(Single_Segment *first_seg_in_line)
	//{
	//	line = first_seg_in_line;
	//	number_of_segments_in_this_line = 1;
	//}
	//void set_number(uint16_t number_of_segments)
	//{
	//	number_of_segments_in_this_line = number_of_segments;
	//}
	Single_Line_embedded () : number_of_segments_in_this_line(0)
#ifdef RED_EXTRACT_BUILD
	,special_debug(0xfeedbee5)
#endif
	{
	}
	bool add_segment(uint16_t start, uint16_t end);
	
private:
	uint16_t number_of_segments_in_this_line;
#ifdef RED_EXTRACT_BUILD
	unsigned int special_debug;
#endif
};
uint16_t number_on_line(Single_Line_embedded* ref)
{
	return ref->number();
}

//#define OLD_BITWISE_FILTER 1
#ifdef OLD_BITWISE_FILTER
int IPG_get_pixel(byte* source, int x, int y)
{
	if(x<0 || x>=processed_image_width) return 0;
	if(y<0 || y>=processed_image_height) return 0;
	
	source += x>>3;
	source += y*processed_row_bytes;
	
	byte mask = 0x80 >> (x&0x07);
	
	return ((*source)&mask)?1:0;
}

int BF_get_pixel_weight(byte* source, int x, int y, byte mask)
{
	int weight = 0;
	// there are 9 points here. They carry different weightings.
	// cross
	weight += 100*IPG_get_pixel(source, x, y);
	weight += 2*IPG_get_pixel(source, x+1, y);
	weight += 2*IPG_get_pixel(source, x, y+1);
	weight += 2*IPG_get_pixel(source, x, y-1);
	weight += 2*IPG_get_pixel(source, x-1, y);
	
	// diagonals
	weight += IPG_get_pixel(source, x+1, y+1);
	weight += IPG_get_pixel(source, x-1, y+1);
	weight += IPG_get_pixel(source, x+1, y-1);
	weight += IPG_get_pixel(source, x-1, y-1);
	
	return weight;
	
	// weight of 7 means a square of 4 pixels, for instance.
	// but a diagonal cross means a weight of 6.	
}

void camera_image_port::bitwise_filter(byte* source, byte* destination)
{
	byte mask = 0x80;
	for(int y=0; y<processed_image_height; y++)
	{
		for(int x=0; x<processed_image_width; x++)
		{
			int w = BF_get_pixel_weight(source, x, y, mask);
			if(w >= 106)
			{
				*destination |= mask;
			}
			else
			{
				*destination &= ~mask;
			}
			
			// move one pixel left
			if(mask==0x01)
			{
				destination++;
				//source++;
				mask = 0x80;
			}
			else
			{
				mask >>= 1;
			}
		}
	}
}
#else

//
//
// To avoid extra complexity we add a line before and after the image which are blanked out.
//
void fast_bitwise_filter(byte* source, byte* destination)
{
	// Other methods of speeding it up...
	//
	// Optimization level for Camera build ... currently -Os ... which is like -O2 without things that increase size. Could try -O2 and -O3 for speed.
	// Run from ram?? Probably faster than from flash? Especially for longest running routines in terms of time.
	// Turn off interrupts in the camera - I think we don't need them (timer based, for example)
	//
	// *** Don't forget to time each change!!! *** Assume nothing, measure everything!!!
	//
	// Comparison of RAM vs. Flash, ARM vs. Thumb: http://www.myvoice.nl/electronics/timing-programs.php
	// Keil vs. gcc benchmarking: http://www.compuphase.com/dhrystone.htm
	//
		
	// blank the spare top and bottom lines ... we do this to avoid having to have special first line and last line code that doesn't 
	// check outside of range. This is technically slower, but probably doesn't matter that much (well under 120th of processing power)
	byte* source2 = source+processed_row_bytes*(processed_image_height+1);	// +1 takes us to the first line, +240 would take us past the image to the last line
	for(int x=0; x < processed_row_bytes; x++)
	{
		*source = 0;
		*source2 = 0;
		source++; source2++;
	}
	// at the end, source will point at the beginning of the image
	
	//
	// now do the bitwise filter for each line
	//
	for(int y=0; y<processed_image_height; y++)
	{
		byte next_column_weight = 0;			// next column of three pixels
		byte current_column_weight = 0;		// this column of three pixels
		byte last_column_weight = 0;		// previous column of three pixels
		
		// for each line we need to pre-load the data with (x) source data
		byte mask = 0x80;
		byte source_data = *source;
		byte above_source_data = *(source-processed_row_bytes);
		byte below_source_data = *(source+processed_row_bytes);
		
		// no change for last and current ... they are still zero at the moment
		next_column_weight = ((source_data & mask)?(64+2):0) + ((above_source_data & mask)?1:0) + ((below_source_data & mask)?1:0);
		
		mask >>= 1;
		
		// for 8 destination bits ... do this loop
		for(int x=0; x < ((processed_image_width/8)-1); x++)
		{	
			byte dest_data = 0;
			for(int bit=0; bit<8; bit++)
			{
				last_column_weight = current_column_weight / 2;		// the last is taken from the current
				current_column_weight = next_column_weight * 2;			// the current is taken from the next
				next_column_weight = ((source_data & mask)?(64+2):0) + ((above_source_data & mask)?1:0) + ((below_source_data & mask)?1:0);
				
				dest_data <<= 1;
				
				int w = (last_column_weight & 0x07) + current_column_weight + (next_column_weight & 0x07);
				//int w = BF_get_pixel_weight(source, x, y, mask);
				if(w >= 128+4+6)			// center pixel will cause an offset of 128+4. We want +6 in surrounding pixels.
				{
					//dest_data++;
					dest_data |= 1;
				}
				if(bit==6)
				{		// next loop around will be bit 7 ... here we need to step along to the next byte since the source is one ahead of the destination
					mask = 0x80;
					source++;
					source_data = *source;
					above_source_data = *(source-processed_row_bytes);
					below_source_data = *(source+processed_row_bytes);
				}
				else	// 
				{
					mask >>= 1;
				}
				
			}
			
			// after the 8 bit loop
			*destination = dest_data;			
			destination++;
		}
		
		// the final loops around must be slightly different because final 'next' cannot read anything
		byte dest_data = 0;
		for(int bit=0; bit<8; bit++)
		{
			last_column_weight = current_column_weight / 2;		// the last is taken from the current
			current_column_weight = next_column_weight * 2;			// the current is taken from the next
			next_column_weight = ((source_data & mask)?(64+2):0) + ((above_source_data & mask)?1:0) + ((below_source_data & mask)?1:0);
			
			dest_data <<= 1;		
			
			int w = (last_column_weight & 0x07) + current_column_weight + (next_column_weight & 0x07);
			//int w = BF_get_pixel_weight(source, x, y, mask);
			if(w >= 128+4+6)			// center pixel will cause an offset of 128+4. We want +6 in surrounding pixels.
			{
				dest_data++;
			}
			
			if(bit==6)
			{		// next loop around will be bit 7 ... here we need to step along to the next byte since the source is one ahead of the destination
				mask = 0x80;
				source++;
				// end of line is blank
				source_data = 0;
				above_source_data = 0;
				below_source_data = 0;
			}
			else	// 
			{
				mask >>= 1;
			}
			
		}
		
		*destination = dest_data;			
		destination++;
		
	} // end of Y scanning
	
}
#endif



// format:
//
// 1. this line count (16 bits)
// 2. start x (16 bits)
// 3. end x (16 bits)
//    :::
// n. start x (16 bits)
// n+1. end x (16 bits)
//
// repeats for each line

class Horizonal_Pixel_Runner {
	
public:
	Horizonal_Pixel_Runner(byte* source);
	uint16_t on_red_get_next_black(uint16_t x);
	uint16_t on_black_get_next_red(uint16_t x);
	void get_lines_first_data() { data = *current_source; }
	byte get_current_pixel() { return mask&data; }
private:
	byte* current_source;
	byte mask;				// could calculate from y
	byte data;				// could read from *current_source
};


Horizonal_Pixel_Runner::Horizonal_Pixel_Runner(byte* source)
{
#ifndef WIN32
#warning "sort this byte storage out! ... is it fast to have a single function with parameter references?"
#endif
	current_source = source;
	mask = 0x80;
	data = *current_source;
}


//
// Assuming x is on the red pixel, find the first black pixel
//
// NOTES ABOUT END OF LINE: Might be obvious (in hindsight) but the end of the 
// line can only occur when the mask rolls over from 0x01 to 0x80...
// (assuming width%8 = 0, which it does)
//
uint16_t Horizonal_Pixel_Runner::on_red_get_next_black(uint16_t x)
{	
	//
	// does the red end in this byte?
	//
	while(1)
	{
		x++;					// next pixel position
		mask >>= 1;				// always start on the red pixel - no point in checking that one!
		if(mask==0)
		{
			break;				// ran out byte
		}
		
		if((mask&data)==0)		// if this pixel is zero, the last x was the last red
		{
			return x;
		}
	}
	
	// need to start a new byte
	mask = 0x80;
	current_source++;
	if(x >= processed_image_width) return x;	// check for the end of the line
	data = *current_source;			// get new byte
	
	//
	// does next bytes contain all red?
	//
	while(data == 0xff)				// while this pixel is all red...
	{
		x+=8;						// we know the next 8 pixels are red
		current_source++;			// need the next byte
		if(x >= processed_image_width) return x;	// check for the end of the line
		data = *current_source;		// get new byte
	}
	
	// drop out at first non-all-red byte and check pixels in this one
	
	// where does the red end in this byte?
	do
	{		
		if((mask&data)==0)
		{
			return x;
		}
		
		x++;
		mask >>= 1;
		
	} while(mask!=0);// this means we haven't found a zero bit in this byte ... this cannot happen ... but stop anyway.
	
	// what do here in this illegal situation? Assume it clears in the next byte.
	mask = 0x80;
	current_source++;
	if(x >= processed_image_width) return x;	// check for the end of the line
	data = *current_source;
	
	return x;
}

//
// Assuming x is on the black pixel, find the first red pixel
//
// NOTES ABOUT END OF LINE: Might be obvious (in hindsight) but the end of the 
// line can only occur when the mask rolls over from 0x01 to 0x80...
// (assuming width%8 = 0, which it does)
//
uint16_t Horizonal_Pixel_Runner::on_black_get_next_red(uint16_t x)
{
	//
	// does the black end in this byte?
	//
	while(1)
	{
		x++;					// next pixel position
		mask >>= 1;				// always start on the black pixel - no point in checking that one!
		if(mask==0)
		{
			break;				// ran out byte
		}
		
		if(mask&data)		// if this pixel is set, the last x was the last black
		{
			return x;		// return first red(set) pixel position
		}
	}
	
	// need to start a new byte
	mask = 0x80;
	current_source++;
	if(x >= processed_image_width) return x;	// check for the end of the line
	data = *current_source;			// get new byte
	
	//
	// does next bytes contain all black?
	//
	while(data == 0x00)				// while this pixel is all black...
	{
		x+=8;						// we know the next 8 pixels are black
		current_source++;			// need the next byte
		if(x >= processed_image_width) return x;	// check for the end of the line
		data = *current_source;		// get new byte
	}
	
	// drop out at first non-all-black byte and check pixels in this one
	
	// where does the black end in this byte?
	do
	{		
		if(mask&data)
		{
			return x;
		}
		
		x++;
		mask >>= 1;
		
	} while(mask!=0);// this means we haven't found a one bit in this byte ... this cannot happen ... but stop anyway.
	
	// what do here in this illegal situation? Assume it sets a pixel in the next byte.
	mask = 0x80;
	current_source++;
	if(x >= processed_image_width) return x;	// check for the end of the line
	data = *current_source;
	
	return x;
}


//
// This scans the image across the way (from left to right)
//
void embedded_scan_horizontal(byte* source, Single_Line_embedded* line_of_segs, int minimum_seg)
{
	Horizonal_Pixel_Runner find(source);
	
	for(int y=0; y < processed_image_height; y++)
	{
		//uint16_t* seg_column_start = segs; 
		//segs++;			// leave space for count
		//int column_segment_count = 0;
		
		int x = 0;
		
		find.get_lines_first_data();
		
		if(find.get_current_pixel()==0)
		{
			x = find.on_black_get_next_red(x);			
		}
		
		while(x < processed_image_width)
		{
			// find how long the red pixels are
			uint16_t start = x;
			x = find.on_red_get_next_black(x);
			uint16_t end = x-1;
			
			// only record significant lines
			if(end-start >= minimum_seg)
			{
				//segs.add_segment(y, start, end);
				//*segs = start; segs++;
				//*segs = end; segs++;
				//column_segment_count++;
				//max_number_segs--;
				//if(max_number_segs <= 0)
				//printf("y=%i ... ", y);
				if(line_of_segs->add_segment(start, end) == false)
				{
					//*seg_column_start = column_segment_count;
					//for(; y < processed_image_height; y++)
					//{   // fill out the rest of y with zero lengths
					//	*segs = 0; *segs++;
					//}
					//write_line("Fail to add segment embedded_scan_horizontal");
					return;
				}
			}
			
			if(x >= processed_image_width)
			{
				break;
			}
			
			// now skip the black section (no pixel)
			x = find.on_black_get_next_red(x);
			
		} // end of this line of pixels (row)
		
		//*seg_column_start = column_segment_count;
		line_of_segs++;
		
	} // next line (row) of pixels
}

#ifndef WIN32
#warning "should this be inline? is it faster? measure time"
#endif
static inline int vertical_on_red_get_next_black(byte*& source, uint8_t mask, int y)
{	
#ifndef WIN32
#warning "look at code with int& y ... is it faster?"
#endif
	uint8_t value;
	
	do			// on red, get next black
	{
		// we know we are on red, so skip it on first pass		
		source += processed_row_bytes;				// next line
		y++;
		if(y >= processed_image_height)
		{
			break;
		}
		value = (*source) & mask;		// get pixel
	} while(value);
	
	return y;
}

#ifndef WIN32
#warning "should this be inline? is it faster? measure time"
#endif
static inline int vertical_on_black_get_next_red(byte*& source, uint8_t mask, int y)
{	
#ifndef WIN32
#warning "look at code with int& y ... is it faster?"
#endif
	
	uint8_t value;
	
	do
	{
		// we know we are on black, so skip it on first pass		
		source += processed_row_bytes;				// next line
		y++;
		if(y >= processed_image_height)
		{
			break;
		}
		value = (*source) & mask;		// get pixel
	} while(value==0);
	
	return y;
}

//
// This scans the image down the way (from top to bottom)
//
//void embedded_scan_vertical(byte* source, uint16_t* segs, int minimum_seg, int max_number_segs)
void embedded_scan_vertical(byte* source, Single_Line_embedded* line_of_segs, int minimum_seg)
{
	uint8_t mask = 0x80;		// will get re-initialised
	
	for(int x=0; x < processed_image_width; x++)
	{		
		//uint16_t* seg_column_start = segs; 
		//segs++;			// leave space for count
		//int column_segment_count = 0;
		
		int y = 0;
		
		if(!((*source)&mask))
		{
			
			y = vertical_on_black_get_next_red(source, mask, y);			
		}
		
		while(y < processed_image_height)
		{
			// find how long the red pixels are
			uint16_t start = y;
			y = vertical_on_red_get_next_black(source, mask, y);
			uint16_t end = y-1;
			
			// only record significant lines
			if(end-start >= minimum_seg)
			{
				//*segs = start; segs++;
				//*segs = end; segs++;
				//column_segment_count++;
				//max_number_segs--;
				//printf("x=%i ... ", x);
				if(line_of_segs->add_segment(start, end) == false)
				{
					//*seg_column_start = column_segment_count;
					//for(; y < processed_image_height; y++)
					//{   // fill out the rest of y with zero lengths
					//	*segs = 0; *segs++;
					//}
					//write_line("Fail to add segment embedded_scan_vertical");
					return;
				}
			}

			if(y >= processed_image_height)
			{
				break;
			}
			
			// now skip the black section (no pixel)
			y = vertical_on_black_get_next_red(source, mask, y);
			
		} // end of this line of pixels (row)
		
		//*seg_column_start = column_segment_count;
		source -= processed_row_bytes*processed_image_height;		// get back to top of column
		
		mask >>= 1;
		if(mask==0)
		{
			mask = 0x80;
			source++;
		}
		
		line_of_segs++;						// next line

	} // next column of pixels to the right
	
}



// calculation of RAM for current segment count:
// 
// original picture data = (320*240)/8 = 9600
// original picture buffer (extra 2 lines) = 9680
// bitwise filtered picture buffer size = 9680
// (Note: maybe only one requires to be two lines bigger for fast bitwise filter?)
// Both of these can be freed once we get to the process segment stage
// 
// vsegment buffer size = 9600 bytes    (These are fairly arbitary)
// hsegment buffer size = 9600 bytes
//
// Number of segments vsegment (240 lines) = 2240 segment pairs (4 bytes per segment)
// Number of segments hsegment (320 lines) = 2280 segment pairs (4 bytes per segment)
//
// Can't rearrange to only have one buffer ... since we would need bitwise image otherwise.
// But might need to watch memory fragementation. Order of new/delete important.
//
// First requires = 24728 bytes
// Second requires 25080 bytes
//
// Only require one at a time.
//
// Total new memory use for image processing max = 44280.
// This leaves about 15.5K for everything else. Static memory use is currently less than 2K, not sure what heap
// memory is in camera ... but could probably increase segment space ... speed permitting.
// 
// Might only need vertical or horizontal when moving around? Optimisation ... leave till later
// Can also increase minimum line size (both 2d and segment). Optimisation ... leave till later.
// 
//
// for segement data with extra stuff we require:
// (320 lines)
// 2280 segment pairs (4 bytes per segment ... 2 bytes start, 2 bytes stop) = 9120 bytets
// plus 11 bytes per segment = 25080
// total storage for segments = 34,200 (ignoring alignment issues)
// additional storage required for image-line-of-segments pointers = 320*4 (for pointers) + 320*2 (for count) = 1920
// 


// Revised memory use for 240 line case (horizontal)
//
// segment pool 10 segments per line allowed * 240 lines * 16 bytes per class = 38400 bytes
// line buffer = 240 * 8 bytes = 1920 bytes
// image buffer = 9600 bytes
// TOTAL = 49920 bytes
//
//
// Revised memory use for 320 line case (vertical)
//
// segment pool 10 segments per line allowed * 320 lines * 16 bytes per class = 51200 bytes
// line buffer = 320 * 8 bytes = 2560 bytes
// image buffer = 9600 bytes
// TOTAL = 63360 bytes
//
// AVAILABLE = 63644 bytes
//
// additionally 2d line array (for 50) = 408 bytes
//
// Probably actions & conclusion: 
//
//  1. test segment usage on different scenes
//  2. lower segment pool for 320 case (only). 
//  3. Increase line buffer for both to 100. 
//  4. Add graceful check in for out of lines. 
//  5. Reuse lines that are too short.

/*  ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ? ?

quick to do:
============
[done] - get old single shot code compiling
[done] - test old code
[done] - wire up embedded image processes .cpp - get compiling
[done] - wire up embedded image processes .cpp - get up to and including segment scanners working
[done]- wire up line finder
[done]	--> 1. Create routine that allocate pool of segments
[done]	--> 2. Create routine that ports from old format to new format of segment data for line finder
[done] - 2d coordinates sent code (minimum time possible)
[done] - 2d coordinates receive code
[done] -- 2. Very small lines are not being rejected (e.g. 1 pixel in size)
 (is this separate code in the standard red-extract application?) - yes 
[done] - stop limit of lines<<<
[done]  - occasional locking master(?) when using pic 4 75 (allocating too much on stack on master!)
[done]... we could use some of the segment pool space to increase the number of lines
 
 
 [done] --- 1. Cross line right across vertical image
 [done] (look at images from line lines that exhibit this)
 
 [done] [A1]
 [done] ... part of the problem could be that if the line list is limited and it returns, the caller assumes we've added - there is no protocol for
     couldn't add line. Even if we add more lines there is still the situation where we might run out. Therefore the
     line creator *must* have a protocol that the upper level respects and uses to avoid adding more lines.
 
 [done] ... STILL this can't explain all occurances because sometimes we didn't run out of lines (since originally it was halt ... 
 and it looked as though the very first one had a problem
 
 [A2]
[done] ... look at utilisation of segment pool
 
[done] ... alternatively, as the next sentence says .. we could reclaim know dead lines.
[done] -- kull more lines more quickly - so we don't run out of lines (or find more space for lines!). 

 
[done] ... need a way of viewing what the robot does see ... a graphical terminal with dtr/rts control (or connect a serial 
 interface without line control like the extension lead!!!

// Do minimum in this list to get the thing working ... fix up later
[done]//  1. test segment usage on different scenes
[done]//  2. lower segment pool for 320 case (only). 
[done]//  3. Increase line buffer for both to 100. 
[done]//  4. Add graceful check in for out of lines. 
[done]//  5. Reuse lines that are too short.
 
[done] - get 2d-plan_view working (2d-3d conversion)
[done]- check memory usage for embedded platform

**Stage 1**
- calculate robot position and heading
- position & heading from shaft encoders
- orientate based on current knowledge - adjust based on vision, update orientation
- add walls into map 
- 2d plan view scaling (too small)??
- 3d clip for things outside the maze
- move based on knowledge
- maze solve to c++ from real basic
  
**Stage 2**
- what happens if there is walls where we think there shouldn't be or no wall where we think there should be? confidence distance? start lost sweep?
- put timeout in line transfer code in both transmitter and receiver

- Code that warns the user from the main processor when the line buffer becomes full
- illumination ... work out how we find correct brightness ... automatically or some other way (wait a minute for instance)
- check speed of new code on actual target

- SPEED OPTIMISATION: Only initialise segments that get added!!!!
- SPEED OPTIMISATION: Don't keep new/deleting data ... and figure alternate way of initialising classes
- SPEED OPTIMISATION: What's faster - transmitting all segments down or filtering for only those greater than three?
- SPEED OPTIMISATION: Maybe we can get rid of all line initialisation in empty constructor for array construction of line vector?
- SPEED OPTIMISATION: Search for too short lines when we run out only from point of last search... (can loop around before giving up if worried about
 running out too soon).
[space optimisation] - y coord only needs to be a byte, both in line vector and in segments - but will packing make this redundant?
[line optimisation] - maybe we should scan from the bottom on the horizontal scanning so that far away objects get last shot at the segment pool?
[line optmisation] - we could also only scan specfic lines and save state and go back to get the next line ... and feed the line-by-line segments 
 to the process lines to make into 2d lines? The maximum number of segments per line is 320/2= 160. Lot less memory.

 ****
[done] >>>> 1. memory for 320 line version runs out...  thoughts: Single_Segment uses 6 which is rounded up to 8, or just reduce segment pool...
[done] >>>> 2. Interprocessor comms - how?
 ****

not to do at moment:
====================
- think about new GUI

notes: camera processor is overloaded - get narrowest data across to main processor and do 3d, map stuff there


*/



//class Process_Segment_Data {
//	uint16_t bound_to_line;				// can't use line* ... pointer might move when vector is expanded
//	uint16_t before_count;
//	uint16_t minimum_center_abs_offset;
//	uint16_t index_of_previous;
//	char good_line_gradent_at_this_point;
//	uint16_t index_of_next;
//}; // 11 bytes per segment required for process_segments as it stands


// embedded_process_segments
//
// Possible things that need changing
//  * vector of line, especially the 'push_back'
//  * Image_Vector_List_embedded, access to seg, p-> p->before
//


	

	
typedef char tiny_bool;

class Single_Segment {
	
public:
	uint16_t start;
	uint16_t end;
	uint16_t before_count;
	uint16_t minimum_center_abs_offset;
	int16_t index_of_previous;
	int16_t index_of_next;
	int16_t bound_to_line;				// can't use line* ... pointer might move when vector is expanded
	tiny_bool good_line_gradent_at_this_point;
	// 15 bytes per segment required for process_segments as it stands (11 without start and end)
	int8_t line_gradient;
	
	int length() { return end-start+1; }
	
	Single_Segment()//int start_in, int end_in) 
	: 
//	start(start_in), end(end_in),
	before_count(0),
	//minimum_center_abs_offset(123),		// initialisation not required (before count flags this)
	//index_of_previous(-1),				// initialisation not required (before count flags this)
	index_of_next(-1),					// -1 flags not used
	bound_to_line(-1),					// -1 means not bound
	good_line_gradent_at_this_point(false),
	line_gradient(0)					// 0 = no direction, by default
	{ }

	void define_seg(uint16_t start_in, uint16_t end_in)
	{
		start = start_in; end = end_in;
	}
};

uint16_t get_segment_start(Single_Line_embedded* ref, int segment_index)
{
	return ref->line[segment_index].start;
}
uint16_t get_segment_end(Single_Line_embedded* ref, int segment_index)
{
	return ref->line[segment_index].end;
}

Single_Segment* segment_pool_base_pointer = 0;
Single_Segment* segment_pool_current_pointer;
uint16_t number_in_segment_pool = 0;
Single_Segment* get_segment_from_pool()
{
	if(number_in_segment_pool == 0)
	{
		return 0;
		//halt("Seg pool empty");
	}
	number_in_segment_pool--;
	Single_Segment* ret_val = segment_pool_current_pointer;
	segment_pool_current_pointer++;
	return ret_val;
}
int get_number_left_in_pool()
{
	return number_in_segment_pool;
}
void create_segment_pool(uint16_t number_in_pool)
{
	if(segment_pool_base_pointer != 0)
	{
		halt("Segment pool already");
	}

	Single_Segment* segment_pool_start = new Single_Segment[number_in_pool];
	segment_pool_current_pointer = segment_pool_start;
	segment_pool_base_pointer = segment_pool_start;
	number_in_segment_pool = number_in_pool;
}
void delete_segment_pool()
{
	if(segment_pool_base_pointer == 0)
	{
		halt("No segment pool allocated");
	}
	delete[] segment_pool_base_pointer;
	segment_pool_base_pointer = 0;
	number_in_segment_pool = 0;
}






bool Single_Line_embedded::add_segment(uint16_t start, uint16_t end)
{
#ifdef RED_EXTRACT_BUILD
	if(special_debug!=0xfeedbee5)
	{
		halt("NOT REAL Single_Line ... ");
	}
#endif
	
	Single_Segment* segment = get_segment_from_pool();
	if(segment)
	{
		//printf("ptr=%i #=%i start=%i end=%i\n",reinterpret_cast<int>(segment),static_cast<int>(number_of_segments_in_this_line+1),static_cast<int>(start),static_cast<int>(end));
		segment->define_seg(start, end);
		if(number_of_segments_in_this_line==0)
		{
			line = segment;
		}
		number_of_segments_in_this_line++;
		return true;
	}
	return false;
}


// base type
class Image_Vector_List_embedded {
public:
	Single_Line_embedded* image;			// pointer to Single_Line_embedded array - storage not here

	~Image_Vector_List_embedded()
	{
	}
	Single_Line_embedded* get_single_line_array(int index);
	Single_Line_embedded* get_single_line_array();
	
protected:		// stops these being generated
	Image_Vector_List_embedded()
	{
	}
private:
};

// types you actually want!
class Image_Vector_List_vert: public Image_Vector_List_embedded {
public:
	Image_Vector_List_vert()
	{
		image = segments_for_each_line;
	}
	~Image_Vector_List_vert()
	{
	}	
private:
	Single_Line_embedded segments_for_each_line[processed_image_width];
};

class Image_Vector_List_hori: public Image_Vector_List_embedded {
public:
	Image_Vector_List_hori()
	{
		image = segments_for_each_line;
	}
	~Image_Vector_List_hori()
	{
	}
private:
	Single_Line_embedded segments_for_each_line[processed_image_height];
};

Image_Vector_List_embedded* create_vertical_line_buffer()
{
	return new Image_Vector_List_vert;
}
Image_Vector_List_embedded* create_horizontal_line_buffer()
{
	return new Image_Vector_List_hori;
}
void delete_line_buffer(Image_Vector_List_embedded* buffer)
{
	delete buffer;
}
	
Single_Line_embedded* Image_Vector_List_embedded::get_single_line_array()
{
	return image;
}
Single_Line_embedded* Image_Vector_List_embedded::get_single_line_array(int index)
{
	return image+index;
}
Single_Line_embedded* get_single_line_array(Image_Vector_List_embedded* image_list)
{
	return image_list->get_single_line_array();
}
Single_Line_embedded* get_single_line_array(Image_Vector_List_embedded* image_list, int index)
{
	return image_list->get_single_line_array(index);
}


// ----------------------------------------------------------------------
bool overlap(int x1s, int x1e, int x2s, int x2e)
#ifdef RED_EXTRACT_BUILD
;
#else
{
	// we assume x1e >= x1s and x2e >= x2s
	
	if(x1e<x2s)		// end of line 1 is before start of line 2
	{
		return false;
	}
	if(x2e<x1s)		// end of line 2 is before start of line 1
	{
		return false;
	}
	
	
	return true;
}
#endif




// ----------------------------------------------------------------------
inline int signof(int v)		// also called signum function
{
	// v > 0 ... true-false = 1-0 = 1
	// v < 0 ... false-true = 0-1 = -1
	// v == 0 ... false-false = 0-0 = 0
	return (v>0)-(v<0);			
	// alternatives...
	
	//return (a == 0) ? 0 : (a<0 ? -1 : 1);
	
//	if(before_center > this_center)
//	{
//		//offset = before_center-this_center;		// abs(before_center-this_center)
//		this_direction = 1;
//	}
//	else if(this_center == before_center)
//	{
//		//offset = 0;
//		this_direction = 0;
//	}
//	else	// this_center > before_center
//	{
//		//offset = this_center-before_center;		// abs(before_center-this_center)
//		this_direction = -1;
//	}
	
}

//
//
// NOTE: Based on Red Extract 'process_segments'. Has more comments than that version.
//
//static void embedded_process_segments(Image_Vector_List& segs, Vec<line>& line_list, int width, int height, int center_step_offset)
void embedded_process_segments(Image_Vector_List_embedded& segs, line_list_custom_vector& line_list, int width, int height, int center_step_offset, int reject_line_length)
{
	//
	// First do top line (which we don't scan for previous, so are all line starts)
	//
	{
		int y = 0;
		Single_Line_embedded* p = &(segs.image[y]);
		int num_segments = p->number();
		
		for(int i=0; i<num_segments; i++)
		{
			int this_s = p->line[i].start;
			int this_e = p->line[i].end;
			int this_center = this_s + ((this_e-this_s)/2);  // does (s+e)/2 = s + ((e-s)/2) ?  Yes, since s/2 + e/2 = s + e/2 - s/2
			
			line temp(this_center, y);
			if(line_list.push_back2(temp)==false)		// impossible to fail. If it does then quit.
			{
				  write_line("pexit1");
				  return;
			}
			//cout<<"Create Line"  << line_list.size()-1 << " at x=" << this_center << " y=" << y << endl;
			
			p->line[i].bound_to_line = line_list.size()-1;
		}
	}
	
	// for each segment we will look for overlapping segments on the line above
	// if we find overlap then we will make these.
	// 
	// We can't do this for the stop and bottom lines so we ignore
	//
	// 
	for(int y=1; y < height; y++)
	{
		
		Single_Line_embedded* p = &(segs.image[y]);
		int num_segments = p->number();
		
		Single_Line_embedded* p_before = &(segs.image[y-1]);
		int num_segments_before = p_before->number();
		
		
		// just check the pointer has been filled in ... debugging purposes
		// for the entire line before
#if 0
		for(int seg=0; seg < num_segments_before; seg++)
		{
			if(p_before->line[seg].bound_to_line == -1)
			{
				PrintAlert("Missing bound to pointer for segment");
				exit(-1);
			}
		}
#endif

		//Single_Line_embedded* p_after = &(segs.image[y+1]);
		//int num_segments_after = p_after->number();
		
		for(int i=0; i<num_segments; i++)
		{
			int this_s = p->line[i].start;
			int this_e = p->line[i].end;
			
			int this_center = this_s + ((this_e-this_s)/2);
			
			//
			// Do we have to check both BEFORE and AFTER? (Since we scanning from vertically anyway)
			// Is it best if we scan top to bottom?
			//    
			
			// SUMMARY: check against all the segments on the line before
			//
			// DETAIL: The for loop literally just tries to find the nearest segment 
			// that matches from the line before.
			// We consider: (a) there must be overlap between the segment before and this one, and
			// (b) step of center of the two segments is within range.
			// It records the index of the previous, plus the number of pixels between the two centers (called 'offset').
			// We DON'T resolve conflicts between multiple competing segments on this line at this point
			for(int j=0; j<num_segments_before; j++)
			{
				int before_s = p_before->line[j].start;
				int before_e = p_before->line[j].end;
				int before_center = before_s + ((before_e-before_s)/2);
				int offset = abs(before_center-this_center);
				
				if(overlap(before_s, before_e, this_s, this_e) && offset <= center_step_offset)
				{
					// we also want to check that all these segments are going in the right direction
					// 1. Fetch previous segment direction (can be 0, -1, 1 ... no direction(or first one), -1 sometimes previous negative, +1 sometimes previous postive)
					// 2. Check this segment direction is the same or 0, if not cannot be part of this line
					//
					// POTENTIAL PROBLEMS:
					// Will tend to break lines into small pieces if wall edges meander about - however it's much faster
					// than some more complex average gradient routine.
					int previous_direction = p_before->line[j].line_gradient;
					int this_direction = signof(before_center-this_center);		// could also use this to calculate offset
					
					if(this_direction == 0 || previous_direction == 0 || previous_direction == this_direction)	// we can only consider it either we/previous are neutral or both are same direction					{
					{
						p->line[i].before_count++;
						
						if(p->line[i].before_count == 1)
						{
							// first one is always loaded
							p->line[i].minimum_center_abs_offset = offset;
							p->line[i].index_of_previous = j;
							// if previous_direction != 0, then we must take the direction of the previous segment comparison
							// if previous direction is zero then ... we are either zero or a specific direction from now on
							p->line[i].line_gradient = (previous_direction ? previous_direction : this_direction);
						}
						else
						{
							// otherwise we load if this one is smaller
							if(offset < p->line[i].minimum_center_abs_offset)
							{
								p->line[i].minimum_center_abs_offset = offset;
								p->line[i].index_of_previous = j;
								// if previous_direction != 0, then we must take the direction of the previous segment comparison
								// if previous direction is zero then ... we are either zero or a specific direction from now on
								p->line[i].line_gradient = (previous_direction ? previous_direction : this_direction);
							}
						}
					}
				}
			}
			
			// EARLY NOTES
			// then we find the segment centers and check there is not too large a difference
			// we might want to consider that further up the picture
			// what about cross sections (i.e. horizontal walls). These might break this algorithm
						
			// BLOCK DESCRIPTION:
			// a 45 degree line will have one step in x for one step in y)
			// if it's over 45 degrees we allow the other scan to pick it up
			// Actually center_step_offset would be 1 for this. For a specific case we 
			// allow 2 ... depending upon veritical or horizontal scanner.
			//
			// REVIEWING 5Apr2009: It looks like this is always true, but perhaps I misundertand 
			// the for loop above?
			if(p->line[i].before_count && p->line[i].minimum_center_abs_offset <= center_step_offset)	// is 1 a bit aggressive? should this be 2?
			{
				p->line[i].good_line_gradent_at_this_point = true;
			}
			
			// BLOCK DESCRIPTION:
			// If there is no matching previous segment, we need to create a new line.
			if(p->line[i].before_count == 0)
			{
				// is there is not before, this must be the start of a line
				line temp(this_center, y);
				int line_index = line_list.push_back_deleting(temp, reject_line_length, y-2);		// y-2 is ended on not last line but line before. Any line that ended there will never be extended
				if(line_index==-1)
				{
				  write_line("pexit2");		// exiting early ... probably ok
				  return;
				}
				//cout<<"Create Line "  << line_list.size()-1 << " at x=" << this_center << " y=" << y << endl;
				
				p->line[i].bound_to_line = line_index;
			}
			
			// BLOCK DESCRIPTION:
			// Resolve conflicts between segments that have identified the same
			// segment on the previous line.
			
			// DETAIL:
			// we link to the line and also put in the after segemnt here
			// This means multiple before candidates for this segment - which were resolved
			// by the search against all previous segments to the closest - are not a problem.
			// In this case we can line safely. This is the Y class of problems.
			//
			// However in there there might be multiple segment candidates for one before on this line.
			// We need to resolve these as we come to them. This is the inverted Y class of problems. 
			
			if(p->line[i].before_count)
			{
				// there was a previous match
				int j = p->line[i].index_of_previous;
				int i2 = p_before->line[j].index_of_next;		// next of previous
				
				if(i2 == -1)		// no previous forward patch - someone elses problem
				{
					p_before->line[j].index_of_next = i;			// make sure we can index both ways
					
					int this_line = p_before->line[j].bound_to_line;	// fetch line from the one we are bound to
					p->line[i].bound_to_line = this_line;
					if(y==150)
					{
						//cout << "Line 150: " << this_line << endl;
					}
					line_list[this_line].change_end(this_center, y);
				}
				else // this is being used for another segment for a previous (inverted Y) ... we need 
				{
					// inside variable i2 is the imposter that would steal our  thunder!
					// see who has more rights to this
					int current_before_next_s = p->line[i2].start;
					int current_before_next_e = p->line[i2].end;
					
					int before_next_center = current_before_next_s + ((current_before_next_e-current_before_next_s)/2);
					
					int before_next_offset = abs(before_next_center-this_center);
					
					// only if ours if smaller do we replace it...
					if(before_next_offset > p->line[i2].minimum_center_abs_offset)
					{
						// patch before next to replace previous as an entry
						
						
						// make new line start at the beginning of this one (can't be a line start since before_count for this one must originally be 1+)
						line temp(this_center, y);
						int line_index = line_list.push_back_deleting(temp, reject_line_length, y-2);		// y-2 is ended on not last line but line before. Any line that ended there will never be extended
						if(line_index==-1)
						{
							write_line("pexit3");		// exiting early ... probably ok since we are doing this before we've made any other modifications
							return;
						}
						//cout<<"Create Line " << line_list.size()-1 << " at x=" << this_center << " y=" << y << endl;
						p->line[i2].bound_to_line = line_index;

						// fix up the other elements as well
						p->line[i2].before_count = 0;							// might be other candidates but we know none of them are as good as this one
						p->line[i2].good_line_gradent_at_this_point = false;	// we need to blank this out ... probably best solution
						// p->line[i2].minimum_center_abs_offset ... can be left since before_count == 0
						// p->line[i2].index_of_previous ... can be left since before_count == 0


						// patch previous to make it point to this one
						p_before->line[j].index_of_next = i;			// make sure we can index both ways
						
						int this_line = p_before->line[j].bound_to_line;	// fetch line from the one we are bound to
						p->line[i].bound_to_line = this_line;
						line_list[this_line].change_end(this_center, y);
					}
					
				}
			}
			
			
		} // for i segments on this line
		
	} // for y = each line 
	
}

