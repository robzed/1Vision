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

#ifndef INTERPROCESSOR_COMMS_H
#define INTERPROCESSOR_COMMS_H

#include "robot_basic_types.h"
#include "ipc_serial_driver.h"
#include "language_support.h"
#include "line_list_custom_vector.h"	// for line list custom vector
//
//
//
class IPC_protocol_engine {
public:
	IPC_protocol_engine(ipc_serial_driver *serial_driver);
	void service();
	byte get_reply();
	void handle_reply();
	bool reply_recieved_correctly(byte this_commands_length);
	void set_new_command(byte* data, byte length);
	void send_old_command();
	bool retry_failed() { return retry_failed_flag; };
	byte* get_current_data();
	int number_of_data_bytes();
private:
	byte rx_length();
	bool checksum_ok(byte this_commands_length);
	void retry_code();
	byte incoming[16];
	byte incoming_length;
	byte outgoing_copy[15];
	byte outgoing_length;
	ipc_serial_driver *driver;
	byte retry_count;
	unsigned int packet_start_time;
	unsigned int reply_timeout_time;
	bool retry_failed_flag;
};


class IPC_packet_base {
	
};


//
//
//
class IPC_led_command {
public:
	void service();
	//void set_led(int which_led, bool status);
	bool completed() { return complete || command_failed(); }
	IPC_led_command(ipc_serial_driver *serial_driver, byte which_led, byte state);
	bool command_failed();
private:
	bool complete;
	IPC_protocol_engine base;
};


//
//
//
class IPC_device_data_command {
public:
	void service();
	//void set_led(int which_led, bool status);
	bool completed() { return complete || command_failed(); }
	IPC_device_data_command(ipc_serial_driver *serial_driver);
	byte get_version() { return protocol_version; }
	bool command_failed();
private:
	bool complete;
	IPC_protocol_engine base;
	byte protocol_version;
};


//
//
//
class IPC_button_command {
public:
	void service();
	//void set_led(int which_led, bool status);
	bool completed() { return complete || command_failed(); }
	IPC_button_command(ipc_serial_driver *serial_driver);
	byte button_status() { return button_data; }
	bool command_failed();
private:
	bool complete;
	IPC_protocol_engine base;
	byte button_data;
};


//
//
//
class IPC_i2c_read {
public:
	void service();
	//void set_led(int which_led, bool status);
	bool completed() { return complete || command_failed(); }
	IPC_i2c_read(ipc_serial_driver *serial_driver, byte address);
	byte data() { return data_store; }
	bool command_failed();
private:
	bool complete;
	IPC_protocol_engine base;
	byte data_store;
};


//
//
//
class IPC_i2c_write {
public:
	void service();
	//void set_led(int which_led, bool status);
	bool completed() { return complete || command_failed(); }
	IPC_i2c_write(ipc_serial_driver *serial_driver, byte address, byte data);
	bool write_success() { return write_success_flag; }
	bool command_failed();
private:
	bool complete;
	IPC_protocol_engine base;
	bool write_success_flag;
};



//
//
//
class IPC_spool_image_out {
public:
	void service();
	//void set_led(int which_led, bool status);
	bool completed() { return complete || command_failed(); }
	IPC_spool_image_out(ipc_serial_driver *serial_driver, byte which_type, byte which_level);
	bool command_failed();
private:
	bool complete;
	IPC_protocol_engine base;
};


//
//
//
class IPC_init_camera {
public:
	void service();
	//void set_led(int which_led, bool status);
	bool completed() { return complete || command_failed(); }
	IPC_init_camera(ipc_serial_driver *serial_driver);
	bool command_failed();
private:
	bool complete;
	IPC_protocol_engine base;
};


//
//
//
class IPC_get_lines {
public:
	void service();
	//void set_led(int which_led, bool status);
	bool completed() { return complete || command_failed(); }
	IPC_get_lines(ipc_serial_driver *serial_driver, byte RG_level, byte blue_compare, line_list_custom_vector& lines1_out, line_list_custom_vector& lines2_out);
	bool command_failed();
private:
	bool complete;
	IPC_protocol_engine base;
	ipc_serial_driver *driver;
	line_list_custom_vector& lines1;
	line_list_custom_vector& lines2;
};



//
// General functions
//
void ipc_slave_service();
byte get_ipc_protocol_version();

// TO DO
//timeout = 100th of a second ... why so long?
//base to make programming simpler


#endif // INTERPROCESSOR_COMMS_H
