/*
 *  i2c_driver.h
 *  robot_core
 *
 *  Created by Rob Probin on 01/03/2007.
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

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "robot_basic_types.h"

class i2c_driver {
	
public:
	// normal 
	i2c_driver();
	~i2c_driver();
	
	//
	// these are the main public interface
	//
	// note: user routines shouldn't remove data from pointer location until finished
	// i.e. data is not copied but used directly
	void send_data(unsigned char* data, int bytes);	// send data command
	void rx_data(unsigned char* data, int bytes);	// receive data command
						// NOTE: wait for compleition before starting a new command
	
	void service_i2c();		// call frequently to advance progress
	bool completed();		// true when finished
	bool error();			// true when something went wrong - cleared on new command, valid on completed
	bool device_busy();		// true when start address return NOT ACK
	//
	// the folowing are very low level control - don't use normally except 
	// for class unit testing and debug
	//
	void print_reg_status();
	void set_control_reg(bool AA_in, bool SI_in, bool STO_in, bool STA_in);
	void set_data_reg(byte data);
	byte get_data_reg();
	bool get_serial_interrupt_flag();

private:
	// these are the protocol-state-machine service member functions
	void service_inactive();
	void service_write();
	void service_read();

	// these are utility protocol state machines
	void service_wait_for_SI();
	void service_wait_for_slave_address_tx_finish();
	void service_stop();
	void service_wait_for_read_byte();
	void insert_wait_for_SI(int status_required);
	void insert_stop();
	void insert_wait_for_read_byte();

	// these are used to start protocol service function state machines
	typedef void(i2c_driver::*funcptr_t)();
	void call_FSM(funcptr_t state);
	void finish_FSM();		// finish a call
	void abort_FSM(bool in_error);		// abort all FSM back to idle
	
	void print_FSM_status();


	// used in basic i2c functions
	unsigned char *data_ptr;
	int byte_count;
	bool error_flag;
	bool device_busy_flag;
	
	unsigned int wait_for_status_code;
	//
	// The following functions are related to the FSM (finite state
	// machine) management and the ability for limited nesting
	
	// this is to track the call return thingy :-)
	unsigned int protocol_level;
		
	// current protocol states
	funcptr_t update_funcptr;
	int protocol_state;
	
	// stacked protocol state
	funcptr_t pushed_update_funcptr;
	int pushed_protocol_state;	
	
	// address
	unsigned char i2c_write_address;
	unsigned char i2c_read_address;
};



#endif // I2C_DRIVER_H

