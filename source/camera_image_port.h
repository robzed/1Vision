/*
 *  camera_image_port.h
 *  robot_core
 *
 *  Created by Rob Probin on 22/04/2007.
 *  Copyright (C) 2007 Rob Probin.
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

#ifndef CAMERA_IMAGE_PORT_H
#define CAMERA_IMAGE_PORT_H

#include "camera_serial.h"
#include <stdint.h>
#include "ipc_serial_driver.h"
#include "embedded_image_processors.h"
//
// Camera side class for read/write to camera
//
class camera_image_port {

public:
	camera_image_port();
	void test();
	void iotest();
	void measure();
	void dutycycles();
	void get_image(int type, byte level);
	void get_image_640x480();
	void get_image_320x240();
	void process_image(ipc_serial_driver& s, uint8_t RG_offset, bool blue_compare);
	void download_bitwise_image(bool enable_bitwise_filter, byte RG_level);
	
	bool setup_camera();

	void setparam(int clock_div, bool mode_8bit);
private:
	bool setup_camera_for_test(camera_serial2& comms);
	static bool mode_8bit_flag;
	static int clock_div_value;
	static bool camera_setup_already;
};


class receive_processed_image_lines {
	
public:
	receive_processed_image_lines(ipc_serial_driver& serial, line_list_custom_vector& lines1, line_list_custom_vector& lines2);
	
private:
	byte get_byte();
	int16_t get_word();
	void load_buffer();
	void load_first_buffer();
	
	// data
	ipc_serial_driver& s;
	byte block_count;			// what the number of this block is - starts at block_offset provided by client
	byte bytes_received_count;	// how many bytes received into data_store
	byte byte_read_index;
	byte data_store[14];
	uint16_t word_count;
};


#endif // CAMERA_IMAGE_PORT_H

