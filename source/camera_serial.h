/*
 *  camera_serial.h
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

#ifndef CAMERA_SERIAL_H
#define CAMERA_SERIAL_H

#include "i2c_driver.h"


class camera_serial  {
	
public:
	camera_serial();
	~camera_serial();
	
	void worker_function();
	bool completed();
	
	// this cannot do more than one thing at a time
	//
	// for these two functions data is copied ... so you can dispose of it after the call
	void raw_write(byte camera_register, byte* data, int bytes_to_send);
	void raw_read(byte camera_register, byte* data, int bytes_to_read);
	
	// these functions block until complete
	byte raw_read_blocked(byte camera_register);
	void raw_write_blocked(byte camera_register, byte data);

	
private:
	enum mode_t { write_reg, read, write, finished };
	mode_t mode;
	i2c_driver camera;
	int error_count;
	
	int write_byte_count;
	int read_byte_count;
	byte* write_dataptr;		// pointer to data to send
	byte* read_dataptr;	// pointer to data to read

	// private functions
	void remove_old_local_buffer();
	void copy_to_local_buffer(byte camera_register, byte* data_in, int bytes_to_send);
	void resetup_action();
};



// emulates the above class with the software camera read and write
class camera_serial2 {

public:
	// these functions block until complete
	byte raw_read_blocked(byte camera_register);
	void raw_write_blocked(byte camera_register, byte data);

private:
};

#endif // CAMERA_SERIAL_H
