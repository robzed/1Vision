/*
 *  xmodem_send.h
 *  robot_core
 *
 *  Created by Rob Probin on 12/05/2007.
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
 *
 */

#ifndef XMODEM_SEND_H
#define XMODEM_SEND_H

#include "robot_basic_types.h"			// required for byte



// Xmodem-1k Send
//
// This is the basic version of Xmodem-1k protocol. It transfers a file 
// using 1024/128 byte encoded binary blocks with a Checksum/CRC. It does not 
// have an idea of file length so if the file is not a modulo 128 then 
// it will be padded with Ctrl-Z characters.
//
// It has several known weaknesses but is useful for relatively 
// reliable serial links transfering small amounts of data files.
// 
// 
// Notice: Xmodem 
//
class XModem_Send {
	
public:
	// General
	// 1. All these calls block until the requested operation is completed.
	// 2. Timers must be running.
	//
	// send_data() will negotiate if the transfer is not open yet
	//
	// It allows the sender to construct the data in multiple passes
	// Note: this call is optional. You can use start_transfer once and then finish 
	// transfer if you have the data ready in one part only.
	//
	bool send_data(byte* data_ptr, unsigned int data_length);

	// finish_transfer() ends the transfer of data - i.e. when the file is
	// sent, call this to cause the receiver to save it
	//
	bool finish_transfer();
	
	// if you need to cancel the transfer call this
	// can be called even if not started (will have no effect in that case)
	void cancel_transfer();
	
	XModem_Send();
	~XModem_Send();
	
private:
	bool open;
	byte block_num;
	bool crc;
	void do_cancel();
	bool internal_send_data(byte* data_ptr, unsigned int data_length);

};



#endif // XMODEM_SEND_H

