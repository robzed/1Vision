/*
 *  embedded_image_processors.h
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
 */

#ifndef EMBEDDED_IMAGE_PROCESSORS_H
#define EMBEDDED_IMAGE_PROCESSORS_H

#include "line_list_custom_vector.h"

#ifdef RED_EXTRACT_BUILD
#include "red_extract_types.h"
#else
#include "stdint.h"
#include "robot_basic_types.h"
#endif

const int processed_image_width = 320;
const int processed_image_height = 240;
const int processed_row_bytes = processed_image_width/8;

void fast_bitwise_filter(byte* source, byte* destination);
//void embedded_scan_vertical(byte* source, uint16_t* segs, int minimum_seg, int max_number_segs);
//void embedded_scan_horizontal(byte* source, uint16_t* segs, int minimum_seg, int max_number_segs);

class Single_Line_embedded;
void embedded_scan_vertical(byte* source, Single_Line_embedded* line_of_segs, int minimum_seg);
void embedded_scan_horizontal(byte* source, Single_Line_embedded* line_of_segs, int minimum_seg);

void create_segment_pool(uint16_t number_in_pool);
void delete_segment_pool();
int get_number_left_in_pool();

class Image_Vector_List_embedded;
Image_Vector_List_embedded* create_vertical_line_buffer();		// delete with delete
Image_Vector_List_embedded* create_horizontal_line_buffer();		// delete with delete
void delete_line_buffer(Image_Vector_List_embedded* buffer);
Single_Line_embedded* get_single_line_array(Image_Vector_List_embedded* image_list);
Single_Line_embedded* get_single_line_array(Image_Vector_List_embedded* image_list, int index);

uint16_t number_on_line(Single_Line_embedded* ref);
uint16_t get_segment_start(Single_Line_embedded* ref, int segment_index);
uint16_t get_segment_end(Single_Line_embedded* ref, int segment_index);


void embedded_process_segments(Image_Vector_List_embedded& segs, line_list_custom_vector& line_list, int width, int height, int center_step_offset, int reject_line_length);



#endif // EMBEDDED_IMAGE_PROCESSORS_H
