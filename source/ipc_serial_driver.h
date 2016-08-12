/*
 *  interprocessor_comms.h
 *  robot_core
 *
 *  Created by Rob Probin on 21/07/2007.
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

#ifndef IPC_SERIAL_DRIVER_H
#define IPC_SERIAL_DRIVER_H

#include "robot_basic_types.h"
#include "robot_hardware_defs.h"

//
// Very basic UART commnications for IPC
//
class ipc_serial_driver {
public:
	ipc_serial_driver();
	//bool tx_empty();
	//void write(const unsigned char* data, unsigned int number_of_bytes);
//(obsolete)	void read(unsigned char* data, unsigned int& number_of_bytes);
	
	//void read(unsigned char* data, unsigned int max_bytes);
	//unsigned int bytes_available();
	
	//
	// low level byte routines
	//
	void write_byte(byte data);
	bool byte_incoming_ready();
	byte read_byte();

	static const int MAX_BYTES_IN_UART_FIFO = 16;
	
//	void print_registers();
private:
	byte bytes_transmitted;

	enum {
		receiver_data_ready_bit = (1<<0),
		transmitter_holding_register_empty_bit = (1<<5)
	};
	
#ifdef TERMINAL_BUILD_TEST_FRAMEWORK
public:
	byte emulation_outbuffer[MAX_BYTES_IN_UART_FIFO];
	int emulation_outbytes;
	byte emulation_inbuffer[MAX_BYTES_IN_UART_FIFO];
	int emulation_inbytes;
#endif
};


const int IPC_BAUD_RATE = 230400*4;
//const int IPC_BAUD_RATE = 115200;
const int IPC_BIT_TIME_IN_cpu_ticks = pclk/IPC_BAUD_RATE;

#endif // IPC_SERIAL_DRIVER_H



