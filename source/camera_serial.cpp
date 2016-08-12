/*
 *  camera_serial.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 24/03/2007.
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

#include "camera_serial.h"
#include "language_support.h"

#define DEBUGGING_ENABLED 0			// turn this on for read/write debugging
#include "debug_support.h"


// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   constructor
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 3 April 2007
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----

camera_serial::camera_serial()
:
mode(finished),
camera(),
error_count(0),
write_byte_count(0),
read_byte_count(0),
write_dataptr(0),
read_dataptr(0)
{
}



// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   destructor
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 3 April 2007
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----


camera_serial::~camera_serial()
{
	dwrite_line("delete camera serial");
	remove_old_local_buffer();
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   worker function
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 3 April 2007
// +
// | DESCRIPTION:  Called periodically in order to push forward the state machine
// +----------------------------------------------------------------ROUTINE HEADER----

void camera_serial::worker_function()
{
	const int max_errors = 3;
	
	// run the camera routine
	camera.service_i2c();
	
	// we have to decide what to do if there is an error...
	if( mode != finished && camera.completed())
	{
		dwrite_line("camera completed");

		if(camera.error())
		{
			dwrite_line("error!");

			if(error_count >= max_errors)
			{
				remove_old_local_buffer();		// remove buffer when we are done with it
				mode = finished;
			}
			else
			{
				error_count++;
				resetup_action();
			}
		}
		else // no error
		{
			if(mode==write_reg)		// special case
			{
				mode = read;
				resetup_action();
			}
			else		// something else...
			{
				remove_old_local_buffer();			// remove buffer when we are done with it
				mode = finished;		// we are all done
			}
		}
	}
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 10 April 2007
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void camera_serial::resetup_action()
{
	if(mode == write)
	{
		dwrite_line("camera send");
		camera.send_data(write_dataptr,  write_byte_count+1);
	}
	else if(mode == write_reg)
	{
		dwrite_line("camera send reg");
		camera.send_data(write_dataptr, 1);
	}
	else if(mode == read)
	{
		dwrite_line("camera read");
		camera.rx_data(read_dataptr, read_byte_count);		// get read_byte_count bytes
	}
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   completed
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 3 April 2007
// +
// | DESCRIPTION:  When we've completed, this will be true.
// +----------------------------------------------------------------ROUTINE HEADER----

bool camera_serial::completed()
{
	return mode == finished;
}


// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 6 April 2007
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----
	
void camera_serial::raw_write(byte camera_register, byte* data, int bytes_to_send)
{
	dwrite_line("raw write");
	if(mode != finished) halt();
	
	error_count = 0;
	write_byte_count = bytes_to_send;

	copy_to_local_buffer(camera_register, data, bytes_to_send);
	camera.send_data(write_dataptr, bytes_to_send+1);		// +1 for camera register

	mode = write;
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 6 April 2007
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void camera_serial::raw_read(byte camera_register, byte* data, int bytes_to_read)
{
	dwrite_line("raw read");
	if(mode != finished) halt();

	error_count = 0;
	read_byte_count = bytes_to_read;
	write_byte_count = 0;
	
	//
	// could theoretically avoid the allocation of heap space this involves
	// ... since we are always txing just one register byte. However, 
	// I have better things to do than to optimise a byte or two of RAM
	// at the moment.
	//
	copy_to_local_buffer(camera_register, data, 0);		// no bytes to send (except register), data is dummy
	read_dataptr = data;		// this is where the data is
	camera.send_data(write_dataptr, 1);		// select register first
	
	mode = write_reg;
}



// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 10 April 2007
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void camera_serial::copy_to_local_buffer(byte camera_register, byte* data_in, int bytes_to_send)
{
	remove_old_local_buffer();
	
	write_dataptr = new byte[bytes_to_send+1];				// plus 1 for camera register
	
	write_dataptr[0] = camera_register;
	
	byte* data_target = write_dataptr+1;
	for(int i=0; i<bytes_to_send; i++)
	{
		*data_target = *data_in;
		data_target++; data_in++;
	}
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 10 April 2007
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void camera_serial::remove_old_local_buffer()
{
	delete[] write_dataptr;		// don't actually need to check this!
	write_dataptr = 0;
}


// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 22 April 2007
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

byte camera_serial::raw_read_blocked(byte camera_register)
{
	byte return_val;
	
	raw_read(camera_register, &return_val, 1);
	while(!completed())
	{
		worker_function();
	}
	
	return return_val;
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 22 April 2007
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void camera_serial::raw_write_blocked(byte camera_register, byte data)
{	
	raw_write(camera_register, &data, 1);
	while(!completed())
	{
		worker_function();
	}
}


//
//
// Software i2c interface
//
//

#include "camera_board_i2c.h"

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 4th Feb 2008
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

byte camera_serial2::raw_read_blocked(byte camera_register)
{
	byte return_data;
	
	bool status = cc3_camera_get_raw_register(camera_register, &return_data);
	
	if(status == false)
	{
		halt("camera i2c read failed");
	}
	
	return return_data;
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 4th Feb 2008
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void camera_serial2::raw_write_blocked(byte camera_register, byte data)
{
	bool status = cc3_camera_set_raw_register(camera_register, data);
	
	if(status == false)
	{
		halt("camera i2c write failed");
	}
}

