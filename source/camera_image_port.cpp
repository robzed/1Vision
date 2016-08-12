/*
 *  camera_image_port.cpp
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

#include "camera_image_port.h"
#include "pc_uart.h"
#include "robot_hardware_defs.h"
#include "timing.h"
#include "VIClowlevel.h"
#include "hardware_support.h"
#include "tick_timer.h"
#include "xmodem_send.h"
#include "camera_board_lowlevel.h"
#include "image_processors.h"
#include "language_support.h"
#include "ipc_serial_driver.h"


void camera_image_port::iotest()
{
//	camera_serial comms;
//	
//	if(!setup_camera_for_test(comms)) return;
//	
//	Timer t;
//	//
//	// Test the I/O lines are working
//	//
//	write_line("PCLK scan");
//	for(int i=0; i<2; i++)
//	{
//		t.set(2,0);
//		while(PCLK() && !t.expired()) {	/* spin */ }
//		if(t.expired()) { write_line("fail1"); return; }
//		write_line("PCLK=0");
//		
//		t.set(2,0);
//		while(!PCLK() && !t.expired()) { /* spin */ }
//		if(t.expired()) { write_line("fail2"); return; }
//		write_line("PCLK=1");
//	}
//	
//	write_line("HREF scan");
//	for(int i=0; i<2; i++)
//	{
//		t.set(2,0);
//		while(HREF() && !t.expired()) {	/* spin */ }
//		if(t.expired()) { write_line("fail3"); return; }
//		write_line("HREF=0");
//		
//		t.set(2,0);
//		while(!HREF() && !t.expired()) { /* spin */ }
//		if(t.expired()) { write_line("fail4"); return; }
//		write_line("HREF=1");
//	}
//	
//	write_line("VSYNC scan");
//	for(int i=0; i<2; i++)
//	{
//		t.set(2,0);
//		while(VSYNC() && !t.expired()) {	/* spin */ }
//		if(t.expired()) { write_line("fail5"); return; }
//		write_line("VSYNC=0");
//		
//		t.set(2,0);
//		while(!VSYNC() && !t.expired()) { /* spin */ }
//		if(t.expired()) { write_line("fail6"); return; }
//		write_line("VSYNC=1");
//	}
//	
//	write_line("FODD scan");
//	for(int i=0; i<2; i++)
//	{
//		t.set(2,0);
//		while(FODD() && !t.expired()) {	/* spin */ }
//		if(t.expired()) { write_line("fail7"); return; }
//		write_line("FODD=0");
//		
//		t.set(2,0);
//		while(!FODD() && !t.expired()) { /* spin */ }
//		if(t.expired()) { write_line("fail8"); return; }
//		write_line("FODD=1");
//	}
//	
}

camera_image_port::camera_image_port()
//: 
// pixel clock register = 63, this is nice and slow
//clock_div_value(63),
//mode_8bit_flag(true)
{
	
}


bool camera_image_port::mode_8bit_flag = true;
int camera_image_port::clock_div_value = 0;		// was 63 but now fifo is accepting the data!
bool camera_image_port::camera_setup_already = false;


void camera_image_port::setparam(int clock_div, bool mode_8bit)
{
	clock_div_value = clock_div & 63;
	mode_8bit_flag = mode_8bit;
	camera_setup_already = false;
}

bool camera_image_port::setup_camera_for_test(camera_serial2& comms)
{
	if(camera_setup_already) return true;
	
	cc3_camera_init();
	
	//comms.raw_write_blocked(0x12, 0x80);		// reset camera command
	
	//write_line("--");
	if(comms.raw_read_blocked(0x1c) != 0x7f)		// check manufacturer's ID are correct
	{
		write_line("Invalid ManuID1");
		return false;
	}
	if(comms.raw_read_blocked(0x1d) != 0xA2)
	{
		write_line("Invalid ManuID2");
		return false;
	}
	
	comms.raw_write_blocked(0x11, clock_div_value);			// probably want a function in camera_serial to do this at some point?
	
	if(comms.raw_read_blocked(0x11) != clock_div_value)
	{
		write_line("Write clock fail");
		return false;
	}
	
	// set up colour bar test pattern output and RGB
//	comms.raw_write_blocked(0x12, 0x24+0x02+0x08);	// 0x02 is colour bar, 0x24 is AGC+AWB, 0x08 is RGB Raw rather than YCrCb
	comms.raw_write_blocked(0x12, 0x24+0x08);	// 0x02 is colour bar, 0x24 is AGC+AWB, 0x08 is RGB Raw rather than YCrCb
//	comms.raw_write_blocked(0x12, 0x24);	// 0x02 is colour bar, 0x24 is AGC+AWB, 0x08 is RGB Raw rather than YCrCb
	
	// set up 8 bit bus
	if(mode_8bit_flag)
	{
		//
		// Suspected hardware bug ... 
		// When CCIR656 mode is off (CCIR601 mode) we get the last byte of the line (1280 wide line) as 0x10 (i.e. black level)
		// When CCIR656 mode is on we get the last byte of the line as 0xff
		//
		// CCIR656 mode puts a 4 byte start header and end header which is FF 00 00 xx
		// although these don't appear in the fifo data (normally) since they are 
		// outside of the HREF window.
		//
		// Therefore it looks like we are dropping the first pixel written to the FIFO.
		// We also see this "feature" in their code, where they read green first (GRGB) even 
		// though the data sheet clearly says BGRG ... (We have to do this as well)
		//
		// I *suspect* this has something to do with the HREF/NAND gate hack  (rather than, say, the pixel clock which appears to be set as rising both in the FIFO and the camera for data valid).
		// 
		// However, since it really only effects the last pixel it probably doesn't
		// effect us as long as we keep in mind that the pixel positions are moved for
		// blue pixels, and I'm not going to spend any more time on this.
		//
		// Also see email to Alan 29Feb2008
		//
		//const byte CCIR656_mode = 0x10;
		//comms.raw_write_blocked(0x13, 0x01+0x20+CCIR656_mode); // 0x01 is auto adjust, 0x20 is 8 bit data bus
		comms.raw_write_blocked(0x13, 0x01+0x20); // 0x01 is auto adjust, 0x20 is 8 bit data bus
	}
	else // 16 bit
	{
		comms.raw_write_blocked(0x13, 0x01); // 0x01 is auto adjust, 0x20 is 8 bit data bus
	}
	
	// now we want to wait for, say, 2 frames. This is maybe up to 2 seconds at the slow speed
	write_line("Wait 2s for stable");
	Timer t(2,0);
	while(!t.expired()) { /* spin */ }
	
	camera_setup_already = true;
	
	return true;
}

//
// Wait for the next frame
//
//static void get_next_frame()
//{
//	// There are two states here: 
//	//   1. We might be in a vsync (in which case we don't need to do anything)
//	//   2. we might be in a frame, in which case we need to wait for VSYNC
//	
//	// wait for frame to end
//	while(!VSYNC()) { /* spin */ }	
//}

//
// This routine wait for the vsync to end
//
//static void wait_for_vsync_end()
//{
//	// wait for VSYNC to end
//	while(VSYNC()) { /* spin */ }
//}


//
// This routine skips a variable amount of lines
//
//static void skip_lines(int lines)
//{
//	while(lines)
//	{
//		// wait for line to start
//		while(!HREF()) { /* spin */ }
//
//		// wait for line to end
//		while(HREF()) { /* spin */ }
//		
//		lines--;
//	}
//}

//
// This routine looks for the start of the next line
//
// this one needs to be quick
//static inline void get_next_line_start()
//{
//	while(!HREF()) { /* spin */ }
//}


void camera_image_port::get_image_640x480()
{
	get_image(0, 0);
}

void camera_image_port::get_image_320x240()
{
	get_image(1, 0);
}

static void put_uint32(unsigned char* p, unsigned int value)
{	
	*p = (value>>24) & 0xff; p++;
	*p = (value>>16) & 0xff; p++;
	*p = (value>>8) & 0xff; p++;
	*p = value & 0xff; p++;
}

#define imagekey "$RAW-IMAGE$"
const byte file_header[] = imagekey;
static void populate_file_header(byte* buffer, unsigned int width, unsigned int height, unsigned int bpp, unsigned int imagesize, unsigned int channels_per_pixel, unsigned int type)
{
	//memcpy(buffer, imagekey, sizeof(imagekey));
	for(unsigned int i=0; i < (sizeof(imagekey)-1); i++)
	{
		buffer[i] = file_header[i];
	}
	byte* data_address = buffer+sizeof(imagekey)-1;
	put_uint32(data_address, width);
	put_uint32(data_address+4, height);
	put_uint32(data_address+8, bpp);
	put_uint32(data_address+12, imagesize);
	put_uint32(data_address+16, channels_per_pixel);
	put_uint32(data_address+20, type);
	
	//for(int i=0; i<20; i++)
	//{
	//	write_int(buffer[i]);
	//	write_string(" ");
	//}
}

void camera_image_port::get_image(int type, byte level)
{
	if(type == 3) 
	{ 
		download_bitwise_image(false, level); 
		return; 
	}
	if(type == 4) 
	{ 
		download_bitwise_image(true, level); 
		return; 
	}
//	if(type == 5)
//	{
//		process_image(75);
//		return;
//	}
	
	const int width = 1280;
	const int lines_to_store = 40;
	const int total_lines = 240;
	camera_serial2 comms;
	XModem_Send tx;
	bool status;

	if(!setup_camera_for_test(comms)) return;


	byte* buffer = new byte[width*lines_to_store+1];	// +1 is for 10 terminator
	

	if(!load_frame_into_fifo())
	{
		write_line("Image Load failed");
		return;
	}
	
	write_line("Start Xmodem download **NOW**");
	Timer t(0,10); while(!t.expired()) { }
	write_line("$XMODEM$");
	
#define TRANSMIT_IMAGE_HEADER 1
#if TRANSMIT_IMAGE_HEADER
	if(type==0)
	{
		populate_file_header(buffer, 640,480,8,640*480, 4, 1);		// last 1 means not same coloured pixels
	}
	else
	{
		populate_file_header(buffer, 320,240,8,320*240, 4, 1);		// last 1 means not same coloured pixels
	}
	
	status = tx.send_data(buffer,1024);	// transmit header
#endif
	if(status ==  false)
	{
		t.set(1,0); while(!t.expired()) { }
		write_line("Xmodem send data failed");
	}
	else 
	{
		for(int i=0; i<total_lines; i+=lines_to_store)
		{
			byte* img = buffer;
			read_raw_pixels_from_camera(img, width*lines_to_store);

			if(type==0)
			{
				status = tx.send_data(buffer, lines_to_store*width);
			}
			else
			{
				img = buffer;
				byte* dest = buffer;
				for(int k=0; k<lines_to_store/2; k++)
				{
					for(int j=0; j<width/8; j++)
					{
						*dest = *img; dest++; img++;
						*dest = *img; dest++; img++;
						*dest = *img; dest++; img++;
						*dest = *img; dest++; img++;
						
						img+=4;
					}
					
					img+=width;
				}
				
				status = tx.send_data(buffer, lines_to_store*width/4);				
			}
			if(status ==  false)
			{
				t.set(1,0); while(!t.expired()) { }
				write_line("Xmodem send data failed");
				break;
			}

		} // for i, end of line block
	} // end else

	status = tx.finish_transfer();
	if(status == false)
	{
		t.set(1,0); while(!t.expired()) { }
		write_line("Xmodem finish transfer failed");
	}
	else
	{
		write_line("Image transfer success");
	}
	
	// clean up
	delete[] buffer;
}


void camera_image_port::download_bitwise_image(bool enable_bitwise_filter, byte RG_level)
{
	camera_serial2 comms;
	XModem_Send tx;
	bool status;
	
	const unsigned int image_size = processed_image_width*processed_image_height/8;
	
	if(!setup_camera_for_test(comms)) return;
	
	
	byte* buffer = new byte[image_size*2];
	byte* dest_buffer = buffer+image_size;
	
	
	if(!load_frame_into_fifo())
	{
		write_line("Image Load failed");
		return;
	}
	
	write_line("Start Xmodem download **NOW**");
	Timer t(0,10); while(!t.expired()) { }
	write_line("$XMODEM$");
	
	//populate_file_header(buffer, 320,240,1,image_size, 1, 0);
	//status = tx.send_data(buffer,1024);	// transmit header
	//status = true;
	//if(status ==  false)
	//{
	//	t.set(1,0); while(!t.expired()) { }
	//	write_line("Xmodem send data failed");
	//}
	//else 
	{
		get_processed_pixels_from_camera(buffer, RG_level);
		if(enable_bitwise_filter)
		{
			fast_bitwise_filter(buffer, dest_buffer);
			status = tx.send_data(buffer, image_size*2);
		}
		else
		{
			status = tx.send_data(buffer, image_size);
		}

		if(status ==  false)
		{
			t.set(1,0); while(!t.expired()) { }
			write_line("Xmodem send data failed");
		}
		
	} // end else
	
	status = tx.finish_transfer();
	if(status == false)
	{
		t.set(1,0); while(!t.expired()) { }
		write_line("Xmodem finish transfer failed");
	}
	else
	{
		write_line("Image transfer success");
	}
	
	// clean up
	delete[] buffer;
}


class transmit_lines_to_master {
public:
	transmit_lines_to_master(ipc_serial_driver& serial, line_list_custom_vector& line_list, byte block_offset);
private:
	void send_block();
	void finish_block();
	void send_byte(byte data);
	void send_word(int16_t data);
	
	// data
	ipc_serial_driver& s;
	line_list_custom_vector& lines;
	byte block_count;			// what the number of this block is - starts at block_offset provided by client
	byte byte_count;			// how many bytes recieved into data_store
	byte data_store[14];
	bool abort;
};

transmit_lines_to_master::transmit_lines_to_master(ipc_serial_driver& serial, line_list_custom_vector& line_list, byte block_offset)
: s(serial), lines(line_list), block_count(block_offset), byte_count(0), abort(false)
{
	// just send the data in a long stream ... lower levels will sort it out!
	uint16_t count = line_list.size();
	//write_line("#lines=", count);
	send_word(count);		// first send how many lines...
	
	for(uint16_t i=0; i<count; i++)
	{
		line& this_line = line_list[i];
		send_word(this_line.start.x);
		send_word(this_line.start.y);
		send_word(this_line.end.x);
		send_word(this_line.end.y);
		//send_word(line_list[i].start.x);
		//send_word(line_list[i].start.y);
		//send_word(line_list[i].end.x);
		//send_word(line_list[i].end.y);
		
		if(abort) return;
	}
	
	finish_block();
}

void transmit_lines_to_master::send_block()
{
	if(abort) return;
	if(byte_count == 0) return;		// nothing to send if not more bytes
	
	byte data = block_count;
	s.write_byte(data);
	//write_int(data);
	byte checksum = data;
	for(int i=0; i<byte_count; i++)
	{
		data = data_store[i];
		s.write_byte(data);
		//write_string(" ");
		//write_int(data);
		checksum += data;
	}
	s.write_byte(checksum);
	//write_line(" cs=",checksum);
}

enum { IP_OK_NEXT, IP_REPEAT, IP_RESET };

void transmit_lines_to_master::finish_block()
{
	if(abort) return;
	if(byte_count == 0) return;		// nothing to send if not more bytes

	// send block (place, data, checksum)
	// wait for reply
	// if reply is OK, next block
	// if reply is reset ... quit
	// if reply is anything else, repeat block
	
	while(1)
	{
		send_block();
		// now wait for reply
		byte rx = s.read_byte();
		//write_line("r=",rx);
		if(rx == IP_OK_NEXT)
		{
			// next block
			block_count++;
			break;
		}
		else if(rx == IP_RESET)
		{
			abort = true;
			break;
		}
	}
}

void transmit_lines_to_master::send_byte(byte data)
{
	if(abort) return;
	data_store[byte_count]=data;
	byte_count++;
	if(byte_count == 14)
	{
		finish_block();
		byte_count = 0;
	}
}

void transmit_lines_to_master::send_word(int16_t data)
{
	// we send in network byte order (big-endian first)
	send_byte(data>>8);
	send_byte(data & 0xff);	
}


void transmit_failed_to_master(ipc_serial_driver& s)
{
	s.write_byte(0xff);
	s.write_byte(0xff);
}



void receive_processed_image_lines::load_first_buffer()
{
	byte_read_index = 0;
	
	while(1) {
		byte block = s.read_byte();
		byte checksum = block;
		//write_string(" "); write_int(block);

		// next two bytes are word count
		byte data_hi = s.read_byte();
		checksum += data_hi;
		data_store[0] = data_hi;
		//write_string(" "); write_int(data_hi);

		byte data_lo = s.read_byte();
		checksum += data_lo;
		data_store[1] = data_lo;
		//write_string(" "); write_int(data_lo);
		
		uint16_t number_of_lines = (data_hi<<8) + data_lo;
		//write_line("words = ", number_of_lines);

		uint16_t number_of_data_bytes = (number_of_lines*8);
		if(number_of_data_bytes > 12)
		{
			number_of_data_bytes = 12;		// 14 data bytes but we have got 2 already
		}
		//write_line("data bytes = ", number_of_data_bytes);
		
		for(int i=2; i<number_of_data_bytes+2; i++)
		{
			byte data = s.read_byte();
			checksum += data;
			data_store[i] = data;
			//write_string(" "); write_int(data);
		}
		byte got_checksum = s.read_byte();
		bytes_received_count = number_of_data_bytes+2;	// +2 for word count uint16_t
		//write_string(" "); write_int(got_checksum);
		
		
		if(got_checksum == checksum &&
		   block == block_count)
		{
			block_count++;
			s.write_byte(IP_OK_NEXT);
			//write_line("ok");
			break;
		}
		
		s.write_byte(IP_REPEAT);
		//write_line("repeat");
		halt();
	};
	
}


void receive_processed_image_lines::load_buffer()
{
	uint16_t number_of_data_bytes = (word_count*2);
	if(number_of_data_bytes > 14)
	{
		number_of_data_bytes = 14;		// 14 data bytes in one packet
	}
	
	byte_read_index = 0;
	
	while(1) {
		byte block = s.read_byte();
		byte checksum = block;
		for(int i=0; i<number_of_data_bytes; i++)
		{
			byte data = s.read_byte();
			checksum += data;
			data_store[i] = data;
		}
		byte got_checksum = s.read_byte();
		bytes_received_count = number_of_data_bytes;
		
		if(got_checksum == checksum &&
		   block == block_count)
		{
			block_count++;
			s.write_byte(IP_OK_NEXT);
			break;
		}
		
		s.write_byte(IP_REPEAT);
	};
		
}

byte receive_processed_image_lines::get_byte()
{
	if(bytes_received_count == 0)
	{
		if(word_count == 0xffff)		// this is the first block to be received
		{
			//write_line("first buffer");
			load_first_buffer();
		}
		else
		{
			//write_line("next buffer, word count=", word_count);
			load_buffer();
		}
	}
	byte data = data_store[byte_read_index];
	//write_line("byte=",data);

	byte_read_index++;
	bytes_received_count--;
	
	return data;
}

int16_t receive_processed_image_lines::get_word()
{
	// we send in network byte order (big-endian first)
	int16_t data = get_byte() << 8;
	data += get_byte();
	
	word_count--;
	return data;
}
const int FIRST_BLOCK_START = 0;
const int SECOND_BLOCK_START = 100;

receive_processed_image_lines::receive_processed_image_lines(ipc_serial_driver& serial, line_list_custom_vector& lines1, line_list_custom_vector& lines2)
: s(serial), bytes_received_count(0)
{
	block_count = FIRST_BLOCK_START;
	word_count = 0xffff;
	int line_count = get_word();
	word_count = line_count*4;
	//write_line("word_count1 = ",word_count);
	//write_line("line_count1 = ",line_count);
	if(word_count == 0xffff)	// something went wrong with setup
	{
		return;
	}
	line this_line;
	for(uint16_t i=0; i<line_count; i++)
	{
		this_line.start.x = get_word();
		this_line.start.y = get_word();
		this_line.end.x = get_word();
		this_line.end.y = get_word();
		lines1.push_back(this_line);
		//if(abort) return;
	}
	
	// second batch of lines
	block_count = SECOND_BLOCK_START;
	word_count = 0xffff;
	line_count = get_word();
	word_count = line_count*4;
	//write_line("word_count2 = ",word_count);
	//write_line("line_count2 = ",line_count);
	if(word_count == 0xffff)	// something went wrong with setup
	{
		return;
	}
	for(uint16_t i=0; i<line_count; i++)
	{
		this_line.start.x = get_word();
		this_line.start.y = get_word();
		this_line.end.x = get_word();
		this_line.end.y = get_word();
		lines2.push_back(this_line);
		//if(abort) return;
	}
	
	// no timeout or missing byte correction
}


bool camera_image_port::setup_camera()
{	
	camera_serial2 comms;
	if(!setup_camera_for_test(comms)) return false;
	return true;
}

void swap_x_and_y(line_list_custom_vector& list)
{
	int length = list.size();
	for(int i=0; i<length; i++)
	{
		line& ln = list[i];
		uint16_t x = ln.start.x;
		uint16_t y = ln.start.y;
		ln.start.x = y;
		ln.start.y = x; 
		x = ln.end.x;
		y = ln.end.y;
		ln.end.x = y; 
		ln.end.y = x;
	}
}

void camera_image_port::process_image(ipc_serial_driver& s, uint8_t RG_offset, bool blue_compare)
{	
//	if(1)
//	{
//		line ln;
//		ln.start.x = 0; ln.start.y = 1; ln.end.x = 2; ln.end.y = 3;
//		line_list_custom_vector& line_list = *new line_list_custom_vector;
//		line_list.push_back(ln);
//		ln.start.x = 1; ln.start.y = 1; ln.end.x = 6; ln.end.y = 3;
//		line_list.push_back(ln);
//		ln.start.x = 2; ln.start.y = 1; ln.end.x = 7; ln.end.y = 3;
//		line_list.push_back(ln);
//		ln.start.x = 3; ln.start.y = 1; ln.end.x = 8; ln.end.y = 3;
//		line_list.push_back(ln);
//		ln.start.x = 4; ln.start.y = 7; ln.end.x = 9; ln.end.y = 3;
//		line_list.push_back(ln);
//
//		delete new transmit_lines_to_master(s, line_list, FIRST_BLOCK_START);	
//
//		delete &line_list;
//
//		line_list_custom_vector line_list2;				//Vec<line>line_list2;
//
//		ln.start.x = 8; ln.start.y = 7; ln.end.x = 6; ln.end.y = 5;
//		line_list2.push_back(ln);
//		ln.start.x = 0; ln.start.y = 1; ln.end.x = 2; ln.end.y = 3;
//		line_list2.push_back(ln);
//		ln.start.x = -1; ln.start.y = -1; ln.end.x = -1; ln.end.y = -1;
//		line_list2.push_back(ln);
//		ln.start.x = 18; ln.start.y = 7; ln.end.x = 6; ln.end.y = 5;
//		line_list2.push_back(ln);
//		ln.start.x = 128; ln.start.y = 7; ln.end.x = 6; ln.end.y = 5;
//		line_list2.push_back(ln);
//		ln.start.x = 228; ln.start.y = 7; ln.end.x = 6; ln.end.y = 5;
//		line_list2.push_back(ln);
//		ln.start.x = 218; ln.start.y = 7; ln.end.x = 6; ln.end.y = 5;
//		line_list2.push_back(ln);
//		
//		swap_x_and_y(line_list2);
//		delete new transmit_lines_to_master(s, line_list2, SECOND_BLOCK_START);
//		
//		return; 
//	}	
	
	camera_serial2 comms;
//	XModem_Send tx;

	if(!setup_camera_for_test(comms)) 
	{
		write_line("Setup fail");
		transmit_failed_to_master(s);
		return;
	}
	
	//write_line("process", RG_offset);
	
	const int horizontal_seg_size_minimum = 3;
	const int vertical_seg_size_minimum = 2;
	
	const int horizontal_center_step_offset = 2;
	const int vertical_center_step_offset = 1;

	const int horizontal_line_minimum = 3;
	const int vertical_line_minimum = 3;
//	const int horizontal_line_minimum = 0;
//	const int vertical_line_minimum = 0;

	const unsigned int src_image_size = processed_row_bytes * (processed_image_height+2);	
	const unsigned int dest_image_size = processed_row_bytes * processed_image_height; 

	//const int number_of_segments_per_line = 10;
	const int horizontal_segments_pool_size = 2400;		// was: number_of_segments_per_line*processed_image_height
	const int vertical_segments_pool_size = 2400;		// was: number_of_segments_per_line*processed_image_width

	byte* filtered_buffer = new byte[dest_image_size];
	byte* buffer = new byte[src_image_size];

	if(!load_frame_into_fifo())
	{
		write_line("Image Load failed");
		transmit_failed_to_master(s);
		return;
	}	

	//write_string(" ", RG_offset);
	// '75' needs to be a EEPROM setting send from main board or autodetect by camera or main board
	if(blue_compare == false)
	{
		get_processed_pixels_from_camera(buffer+processed_row_bytes, RG_offset);			// get pixels into buffer, offset by one line
	}
	else
	{
		get_processed_pixels_from_camera_BLUE(buffer+processed_row_bytes, RG_offset);
	}
	fast_bitwise_filter(buffer, filtered_buffer);

	delete[] buffer;			// get rid of this storage ... no longer needed

	//
	// First horizontal (going down each line, and processing that line for segments)
	//
	// produce scan lines in the horizontal direction
	Image_Vector_List_embedded* hsegs = create_horizontal_line_buffer();
	create_segment_pool(horizontal_segments_pool_size);
	embedded_scan_horizontal(filtered_buffer, get_single_line_array(hsegs), horizontal_seg_size_minimum);

	//write_line("Segl", get_number_left_in_pool());

	// now convert these into lines
	//line_list_custom_vector line_list;		//Vec<line>line_list;
	line_list_custom_vector& line_list = *new line_list_custom_vector;
	embedded_process_segments(*hsegs, line_list, processed_image_width, processed_image_height, horizontal_center_step_offset, horizontal_seg_size_minimum);
	//write_line("LnH=",line_list.size());
	
	delete_segment_pool();
	delete_line_buffer(hsegs);

	line_list_custom_vector& line_list_filtered = *new line_list_custom_vector;
	for(int i=0; i<line_list.size(); i++)
	{
		line& l = line_list[i];
		//write_line("y line abs = ",abs(l.end.y-l.start.y));
		if(abs(l.end.y-l.start.y) > horizontal_line_minimum)
		{
			line_list_filtered.push_back(l);
		}
	}
	// send line data back to main processor ...
	delete new transmit_lines_to_master(s, line_list_filtered, FIRST_BLOCK_START);

	delete &line_list_filtered;
	delete &line_list;

	//
	// Next Vertical (going across the width, one column at a time)
	//
	// produce scan lines in the horizontal direction
	Image_Vector_List_embedded* vsegs = create_vertical_line_buffer();
	create_segment_pool(vertical_segments_pool_size);
	embedded_scan_vertical(filtered_buffer, get_single_line_array(vsegs), vertical_seg_size_minimum);

	//write_line("Segl", get_number_left_in_pool());

	// shove this on the stack
	//line_list_custom_vector line_list2;				//Vec<line>line_list2;
	line_list_custom_vector& line_list2 = *new line_list_custom_vector;

	embedded_process_segments(*vsegs, line_list2, processed_image_height, processed_image_width, vertical_center_step_offset, vertical_seg_size_minimum);		// notice width/height are swapped!!!
	//write_line("LnV=",line_list2.size());
	delete_segment_pool();
	delete_line_buffer(vsegs);

	swap_x_and_y(line_list2);

	line_list_custom_vector& line_list2_filtered = *new line_list_custom_vector;
	for(int i=0; i<line_list2.size(); i++)
	{
		line& l = line_list2[i];
		//write_line("x line abs = ",abs(l.end.x-l.start.x));
		if(abs(l.end.x-l.start.x) > vertical_line_minimum)
		{
			line_list2_filtered.push_back(l);
		}
	}
	
	// set line data back to main processor ... remember to swap X and Y for vertical
	delete new transmit_lines_to_master(s, line_list2_filtered, SECOND_BLOCK_START);
	
	delete &line_list2_filtered;

	delete &line_list2;

	delete[] filtered_buffer;


	return;
	
#if 0
	const unsigned int image_size = processed_row_bytes * (processed_image_height+2);	

	uint16_t* master_buffer = new uint16_t[image_size];
	byte* buffer = reinterpret_cast<byte*>(master_buffer);
	byte* dest_buffer = buffer+image_size;
	
	// clear buffer
	for(unsigned int i=0; i<image_size; i++)
	{
		dest_buffer[i] = 0;
	}

	//disableIRQ();
	write_line("Started reading 1000 frames");

	for(int i=0; i<2000; i++)
	{
		//if(!load_frame_into_fifo())
		//{
		//	write_line("Image Load failed");
		//	return;
		//}	
		//get_processed_pixels_from_camera(buffer+processed_row_bytes, 10);			// get pixels into buffer, offset by one line
		//fast_bitwise_filter(buffer, dest_buffer);
		
		
		// words = 240 lines * ( 1 for segment length/line + (max_seg * 2 (start and end) ))
		// *2 for bytes
		// 
		// 9600 = 30 bytes/line. 28 bytes for segment data (remove count). 28/4 (start+end*2) = 7 segmeents per line.
		
		const int bytes_for_segment_data = 9600 - (320*2);
		const int max_segment_entries = bytes_for_segment_data / 4;		// 2 bytes for start, 2 bytes for end
		// this will be 2240, or 7 per line for 320 lines, or 9.3 for 240 lines
		
		uint16_t *vsegs = master_buffer;
		uint16_t *hsegs = vsegs;
		embedded_scan_vertical(dest_buffer, vsegs, vertical_seg_size_minimum, max_segment_entries);
		//embedded_scan_horizontal(dest_buffer, hsegs, horizontal_seg_size_minimum, max_segment_entries);

	}
	
	write_line("Finished");
	//enableIRQ();

	// clean up
	delete[] master_buffer;
#endif
}


/*

 >> work out buffer pin schematic and CPU pin out on Thursday 
 >> before - get picture off camera
 >>
 
#define _CC3_CAM_VSYNC		0x10000	
#define _CC3_BUF_WEE		0x400				... chip write allow
#define _CC3_CAM_RESET	        0x8000			... optional?
#define _CC3_CAM_HREF		0x4				... also to buffer
#define _CC3_CAM_POWER_DOWN     0x200000		... optional (not required)
#define _CC3_CAM_SCL		0x400000			... already defined
#define _CC3_CAM_SDA		0x800000			... already defined
 
 // Camera FIFO Constants
#define _CC3_BUF_WRST		0x2000
#define _CC3_BUF_RRST		0x1000
#define _CC3_BUF_RCK		0x800
#define _CC3_BUF_RESET		0x8				... don't need
 
 fodd ... no connection .. nice to have but no essential
 
 
 */

#define INTERRUPT_IMAGE_FETCH 1



void camera_image_port::test()
{
	// the plan:
	//   setup the pixel clock to as slow as possible (using the camera serial interface)
	//   wait for couple of frames (for the image to stablise)
	//   watch pclk, vsync, href (using hardware macros)
	//   buffer some data
	//   print the data
	
//	camera_serial comms;
//	
//	if(!setup_camera_for_test(comms)) return;
//
//	//
//	// Let's get some random image data from line 50
//	//
//
//	write_line("Image data fetch");
//
//	disableIRQ();
//	const int bytes_to_store = 2048;
//	byte *img = new byte[bytes_to_store];
//	byte *ip = img;
//
//	// fill buffer with dummy value
//	for(int i=0; i<bytes_to_store; i++)
//	{
//		*ip++ = 0xa6;
//	}
//	ip = img;
//
//#if INTERRUPT_IMAGE_FETCH
//
//	
//	setup_FIQ_registers(img, img+bytes_to_store-1);
//	
//	VICIntSelect = (1<<VIC_Channel_EINT0);	  // select EINT0 as FIQ
//	VICIntEnable = (1<<VIC_Channel_EINT0);    // Enable EINT0 interrupt
//	pin_function_select(16, 1);				 // allocate pin to EINT0
//
//	//byte* temp = get_final_pointer();
//	//write_string("initial Address = "); write_hex(reinterpret_cast<int>(temp)); write_cr();
//#endif
//
//	// (start of critical timing section)
//
//	// wait for frame start
//	while(!VSYNC()) { /* spin */ }
//	// wait for specific line
//	const int wanted_line = 50;
//
//	while(!HREF()){ /* spin */ }		// wait for a HREF whatever happens
//	for(int i=0; i < wanted_line; i++)
//	{
//		while(HREF()){ /* spin */ }		// we don't want this one
//		while(!HREF()){ /* spin */ }
//	}
//
//#if INTERRUPT_IMAGE_FETCH
//	// intially this is after href has already started - this is likely to be 
//	// late at least one pixle
//	enableFIQ();
//	// wait for next frame to start as the end marker
//	while(!VSYNC()) { /* spin */ }
//	while(VSYNC()) { /* spin */ }
//	while(!VSYNC()) { /* spin */ }
//	disableFIQ();
//
//	pin_function_select(16, 0);				 // allocate pin to GPIO
//
//#else
//	for(int i=0; i<bytes_to_store; i++)
//	{
//		// by default the data bus is stable at the rising edge of PCLK
//		while(PCLK()) { /* spin */ }	// wait for low
//		while(!PCLK()) { /* spin */ }	// wait for rising edge
//		*ip = CAMERA_DATA();
//		ip++;
//	}
//
//	enableIRQ();
//	// (end of critical timing section)
//	
//	ip = img;
//	for(int i=0; i<bytes_to_store; i++)
//	{
//		write_hex(*ip++); write_string(" ");
//	}
//	write_cr();
//	
//	//
//	// Let's analyse what happens to different flags
//	//
//	write_line("HREF fetch");
//	ip = img;
//
//	disableIRQ();
//
//	while(!VSYNC()) { /* spin */ }
//	// wait for specific line	
//	while(!HREF()){ /* spin */ }		// wait for a HREF whatever happens
//	for(int i=0; i < wanted_line; i++)
//	{
//		while(HREF()){ /* spin */ }		// we don't want this one
//		while(!HREF()){ /* spin */ }
//	}
//	
//	for(int i=0; i<bytes_to_store; i++)
//	{
//		// by default the data bus is stable at the rising edge of PCLK
//		while(PCLK()) { /* spin */ }	// wait for low
//		while(!PCLK()) { /* spin */ }	// wait for rising edge
//		*ip = (HREF()) ? 1 : 0;
//		ip++;
//	}
//	
//#endif
//	enableIRQ();
//	
//	// (end of critical timing section)
//	
//	ip = img;
//	for(int i=0; i<bytes_to_store; i++)
//	{
//		write_hex(*ip++); write_string(" ");
//	}
//	write_cr();
//	
//	// finally delete our buffer
//	delete[] img;
//	
}



void camera_image_port::measure()
{
//	camera_serial comms;
//	
//	if(!setup_camera_for_test(comms)) return;
//	
//	write_string("divider="); write_int(clock_div_value); 
//	if(mode_8bit_flag)
//	{
//		write_string(", 8");
//	}
//	else
//	{
//		write_string(", 16");
//	}
//	write_line(" bit mode");
//
//	
//	//
//	// Let's count the lines in a frame
//	//
//	while(VSYNC()) { /* spin */ }	// wait for VSYNC to finish if we are part way through
//	while(!VSYNC()) { /* spin */ }	// wait for VSYNC to start
//	
//	// are there any lines in the VSYNC itself?
//	bool lines_in_precount = false;
//	while(VSYNC())	// while in VSYNC
//	{ 
//		if(HREF()) lines_in_precount = true;		// doesn't do a good job of counting
//	}
//	
//	int lines = 0;
//	if(HREF()) lines++;				// count this line
//	while(!VSYNC())	// loop while frame start
//	{
//		while(!HREF() && !VSYNC()){ /* spin */ }	// wait for a HREF whatever happens
//		if(VSYNC()) break;
//		while(HREF() && !VSYNC()){ /* spin */ }		// wait for HREF to end
//		if(VSYNC()) break;
//		lines++;
//	}
//	
//	if(!lines_in_precount)
//	{
//		write_string("NO ");
//	}
//	write_line("Lines in VSYNC");
//	
//	write_string("Lines in a frame ="); write_int(lines); write_cr();
//	
//	//
//	// how many pclks is VSYNC?
//	//
//	int pclks_in_vsync = 0;
//	int pclks_not_in_vsync = 0;
//	bool in_pclk = false;
//	
//	while(VSYNC()) { /* spin */ }	// wait for VSYNC to finish if we are part way through
//	while(!VSYNC()) { /* spin */ }	// wait for VSYNC to start
//	
//	if(PCLK()) in_pclk = true;
//	
//	while(VSYNC())
//	{
//		if(PCLK() && !in_pclk)	// rising edge - has data
//		{
//			pclks_in_vsync++;
//			in_pclk = true;
//		}
//		if(!PCLK() && in_pclk)
//		{
//			in_pclk = false;
//		}
//	}
//	
//	while(!VSYNC())
//	{
//		if(PCLK() && !in_pclk)	// rising edge - has data
//		{
//			pclks_not_in_vsync++;
//			in_pclk = true;
//		}
//		if(!PCLK() && in_pclk)
//		{
//			in_pclk = false;
//		}
//	}
//	
//	write_string("PCLKS in a VSYNC ="); write_int(pclks_in_vsync); write_cr();
//	write_string("PCLKS outside VSYNC ="); write_int(pclks_not_in_vsync); write_cr();
//	
//	//
//	// This tests the duty cycle of pclk over 1 second
//	//
//	Timer t1(1,0);
//	int pclk1=0;
//	int pclk0=0;	
//	while(!t1.expired())
//	{
//		if(PCLK()) pclk1++;
//		else pclk0++;
//		if(!PCLK()) pclk0++;
//		else pclk1++;
//		if(PCLK()) pclk1++;
//		else pclk0++;
//		if(!PCLK()) pclk0++;
//		else pclk1++;
//		if(PCLK()) pclk1++;
//		else pclk0++;
//		if(!PCLK()) pclk0++;
//		else pclk1++;
//		if(PCLK()) pclk1++;
//		else pclk0++;
//		if(!PCLK()) pclk0++;
//		else pclk1++;
//		if(PCLK()) pclk1++;
//		else pclk0++;
//		if(!PCLK()) pclk0++;
//		else pclk1++;
//	}
//	write_int(pclk0); write_string("=pclk0, "); write_int(pclk1); write_line("=pclk1 in 1s");
//	
//	write_line("Reading VSYNC for 10s");
//	int old_vsync = VSYNC();
//	int count = 0;
//	Timer t(10,0);
//	while(!t.expired()) 
//	{
//		int new_vsync = VSYNC();
//		if(new_vsync && !old_vsync)
//		{
//			count++;
//		}
//		old_vsync = new_vsync;
//	}
//	
//	write_int(count); write_line(" VSYNCs in 10 seconds");
//	
//
//	unsigned int start = nanoseconds();
//	unsigned int end = nanoseconds();
//	unsigned int overhead = end - start;
//	write_string("Nanoseconds overhead = "); write_int(overhead); write_string(" ns"); write_cr();
//
//	// 
//	// how long is vsync in terms of time
//	//
//	// we might be in VSYNC here or !VSYNC .. so must wait for both
//	//
//	// wait initially for frame to end
//	while(!VSYNC()) { /* spin */ }
//	// then wait for frame to being
//	while(VSYNC()) { /* spin */ }
//	// wait initially for frame to end
//	while(!VSYNC()) { /* spin */ }
//
//	//start timer here
//	start = nanoseconds();
//
//	// then wait for frame to being
//	while(VSYNC()) { /* spin */ }
//
//	// measure here
//	end = nanoseconds();
//	
//	write_string("VSYNC time = "); write_int(end-start-overhead); write_string(" ns"); write_cr();
//
//	// get to next frame start
//	while(!VSYNC()) { /* spin */ }
//	while(VSYNC()) { /* spin */ }
//
//	start = nanoseconds();
//	while(!HREF()) { /* spin */ }
//	end = nanoseconds();
//	while(HREF()) { /* spin */ }
//	unsigned int end2 = nanoseconds();
//	while(!HREF()) { /* spin */ }
//	unsigned int end3 = nanoseconds();
//	while(HREF()) { /* spin */ }
//	while(!HREF()) { /* spin */ }
//	unsigned int end4 = nanoseconds();
//	write_string("VSYNC to HREF time = "); write_int(end-start-overhead); write_string(" ns"); write_cr();
//	write_string("HREF time = "); write_int(end2-end-overhead); write_string(" ns"); write_cr();
//	write_string("line blank time = "); write_int(end3-end2-overhead); write_string(" ns"); write_cr();
//	write_string("line time = "); write_int(end4-end3-overhead); write_string(" ns"); write_cr();
//	
//	start = nanoseconds();
//	HREF();
//	end = nanoseconds();	
//	write_string("port read time = "); write_int(end-start-overhead); write_string(" ns"); write_cr();
//	start = nanoseconds();
//	HREF();
//	end = nanoseconds();	
//	write_string("port read time = "); write_int(end-start-overhead); write_string(" ns"); write_cr();
//	
	// calculate overhead
}


void camera_image_port::dutycycles()
{
//	Timer t1;
//
//	camera_serial comms;
//	
//	mode_8bit_flag = true;
//	
//	for(int i=0; i<64; i++)
//	{
//		clock_div_value = i;
//		write_int(i); write_line(" = clock div");
//		if(!setup_camera_for_test(comms)) return;
//
//		t1.set(1,0);
//		int pclk1=0;
//		int pclk0=0;	
//		while(!t1.expired())
//		{
//			if(PCLK()) pclk1++;
//			else pclk0++;
//			if(!PCLK()) pclk0++;
//			else pclk1++;
//			if(PCLK()) pclk1++;
//			else pclk0++;
//			if(!PCLK()) pclk0++;
//			else pclk1++;
//			if(PCLK()) pclk1++;
//			else pclk0++;
//			if(!PCLK()) pclk0++;
//			else pclk1++;
//			if(PCLK()) pclk1++;
//			else pclk0++;
//			if(!PCLK()) pclk0++;
//			else pclk1++;
//			if(PCLK()) pclk1++;
//			else pclk0++;
//			if(!PCLK()) pclk0++;
//			else pclk1++;
//		}
//		write_int(pclk0); write_string("=pclk0, "); write_int(pclk1); write_string("=pclk1 in 1s");
//		if(pclk1 < pclk0) { write_string(" ***PCLK1 SMALLER***"); }
//		write_cr();
//	}
	
}




#if 0

unsigned int FIQ_port;
unsigned int BUFFER_address;
unsigned int BUFFER_address_end;


void setup_function()
{
	setup_FIQ_registers(0xE0028000, 0, 1);
	setup_FIQ_registers(0xE0028002, 1, 1);
	setup_FIQ_registers(0xE0028001, 0, 3);
}

//int fiq_r9;
byte* fiq_r10;
byte* fiq_r11;

const int num_bytes_for_image = 1024;

void set_up_image()
{
	fiq_r10 = new byte[num_bytes_for_image];
	fiq_r11 += num_bytes_for_image;
	fiq_r11 --;
}

void __attribute__ ((interrupt("FIQ"))) fiq_image_interrupt()
{
	//*fiq_r10++ = CAMERA_DATA();
	//if(fiq_r10 == fiq_r11)
	//{
	//	disableIRQ();
	//}
}
#endif	

/*

 The original CMUcam uses a divider of 2 (17fps) but the new CMUcam uses a divider of 0 (50fps) for very limited line count - BUT has a memory fifo.
 
 These use Scenix processors running at 50 MIPS or 75 MIPS. I'm guessing hand-optimised assembler - and maybe a faster I/O channel.
 
 They both have an 8-bit data channel.
 
 
 
 -----------
 Alan,
 
 Immediate problem is pclk duty cycle differences between 8 and 16 bit bus. It appears 8 bit mode is not symmetric, where as 16 bit mode is.
 
 Additionally, since we are running at absolutely slowest rate, we will need to speed up in order to get data, say 10 times per second. 
 
 ---------
 
 Calculations
 
 Divider value of 63
 Therefore freq = 27 MHz/(63+1) for 8bit or 27MHz/((63+1)*2)
 
 ---------
 
 8-bit bus
 
 NO Lines in VSYNC
 Lines in a frame =240
 PCLKS in a VSYNC =406
 PCLKS outside VSYNC =34762
 Reading VSYNC for 10s
 9 VSYNCs in 10 seconds
 
 Scope readings
 VSYNC period 1.07 s
 VSYNC high for 12ms
 FODD changes 0 ms before VSYNC goes high
 pclk period =  2.36 us (frequency of 423.7 KHz)
 pclk low for 60ns
 pclk high for 2.3us
 
 ---------
 
 16-bit bus
 
 NO Lines in VSYNC
 Lines in a frame =240
 PCLKS in a VSYNC =2574
 PCLKS outside VSYNC =222651
 Reading VSYNC for 10s
 9 VSYNCs in 10 seconds
 
 Scope readings
 VSYNC period 1.07 s
 VSYNC high for 12.40 ms
 FODD changes 2 ms before VSYNC goes high (reading in error?)
 pclk period = 4.7 us (frequency of 212.8 kHz)
 pclk low for 2.35 us
 pclk high for 2.35 us
 
 ---------
 
 8-bit mode, no divider (effectively 1): Pclk - 37ns approx (not square wave!). Symmetric
 8-bit mode, divider = 1 (effectively 2): Pclk = 74ns. Symmetric
 8-bit mode, divider = 2 (effectively 3): Pclk = 112 ns. 40 ns high (approx 1/3), 70 ns low (approx 2/3).
 8-bit mode, divider = 3 (effectively 4): Pclk = 148 ns. 76 ns high, 72 ns low.
 8-bit mode, divider = 4 (effectively 5): Pclk = 185 ns. 76 ns high, 116 ns low.
 8-bit mode, divider = 4 (pass 2) Pclk = 184ns. 115ns high, 72 ns low.
 8-bit mde, divider = 5 (eff. 6) Pclk = 222ns (4.5MHz), 148ns high, 72 ns low.
 
 
 16-bit mode, no divider. 73ns, 42ns high,32 ns low.
 16-bit mode, divider = 1, 148ns, 80ns high, 68ns low
 16-bit mode, divider = 2, pclk = 222ns, 116 ns high, 106ns low
 16-bit mode, divider = 3, pclk = 296ns, 154ns, 142ns low
 
 
 
 

CAMERA

 Pclk – high vs. low count measure
 -          Rising edge followed quickly by falling edge is bad
 -          Any specific reasons for one over another?
 
 FIQ routine on pclk
 -          load port (address stored in register)
 -          store port in pointer (register)
 -          increment pointer
 -          return from FIQ
 
 
 Compile C version, use output assembler
 
 Ram access, memory accelerator vs. register shifting?
 -          storing longwords
 -          storing bytes
 -          shifting and storing 1 in 4
 
 lsl 23
 and target
 if (end) store byte
 else lsr 8
 
 
 or 4 routines that are
 
 ;; this needs to be optimized…
 b r9                               ;; we might need a branch anyway? Are register offsets slower than immediate jumps? Check arm reference manual
 load port
 lsl 23
 and 0xff
 and to mix register
 ld r9, new routine
 return
 
 
 Background routine processing data as fast as possible … convert from longs to bytes to red match. In a loop until all bytes processed.
 Or maybe we should do conversion on the fly?
 
 Use 2 line buffers for FIQ and swap between them. This will allow an entire line time (100us) to process the previous line.
 
 
 AL422B
 Cost of buffer on digikey? Or from <http://www.averlogic.com/>www.averlogic.com
 <http://www.semiconductorstore.com/pages/asp/supplier.asp?pl=0034>http://www.semiconductorstore.com/pages/asp/supplier.asp?pl=0034
 
 $9.20 ea. No idea about international shipping costs.
 
 
 Samller FIFO available (from 16x9 (cheap1) upwards to 128Kx9 (expensive!)) but processor is unlikely to be able to store at 30fps or 60fps bandwidth – problem is not just interface but read rate over entire frame.
 
 Slower framerates we can read OK (don’t know what max is, maybe at 2Mbytes/second read speed, with 640x480 bytes then 6.5 frames per second).
 
 
 Do we need to AND pclk and href with a logic gate? See timing diagrams in manual for href vs. pclk.
 
  
 
*/

