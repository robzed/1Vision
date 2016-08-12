/*
 *  interprocessor_comms.cpp
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

#include "interprocessor_comms.h"
#include "robot_hardware_defs.h"
#include "hardware_support.h"
#include "tick_timer.h"
#include "camera_board_i2c.h"
#include "camera_image_port.h"
#include "pc_uart.h"

//#define DEBUGGING_ENABLED 1
#include "debug_support.h"

const byte IPC_PROTOCOL_VERSION = 8;			// both ends must agree on this, checked at master end.

//
//
//
byte get_ipc_protocol_version()
{
	return IPC_PROTOCOL_VERSION;
}


//
// This is the packet first byte header
//
enum {
	//
	// Commands go here
	//
	IPC_COMMAND_NONE=10,	// this is a dummy and usually never appears on the bus but is return if length = 0
	IPC_COMMAND_DEVICE_DATA,
	IPC_COMMAND_LED,
	IPC_COMMAND_READ_BUTTON,
	IPC_COMMAND_READ_I2C,
	IPC_COMMAND_WRITE_I2C,
	IPC_COMMAND_SPOOL_IMAGE_OUT,
	//IPC_COMMAND_GET_LINES_RESET,
	//IPC_COMMAND_GET_LINES_NEXT,
	//IPC_COMMAND_GET_LINES_REPEAT_BLOCK, 
	IPC_COMMAND_INIT_CAMERA,
	
	IPC_COMMAND_PROCESS_IMAGE_BASE=30,
	IPC_COMMMAND_PROCESS_IMAGE_TOP=88,
	
	
	//
	// replies go here ... generic first
	//
	//IPC_REPLY_RETRY_PLEASE,			// single reply for all those fails
	IPC_REPLY_BAD_CHECKSUM=100,		// slave got a bad checksum
	IPC_REPLY_INVALID_COMMAND,	// slave got a command it didn't understand
	IPC_REPLY_COMMAND_TIMEOUT,	// slave waited more than 1ms for a command to complete
	
	// now replies to specific commands
	IPC_REPLY_DEVICE_DATA=110,
	IPC_REPLY_LED,
	IPC_REPLY_READ_BUTTON,
	IPC_REPLY_READ_I2C,
	IPC_REPLY_WRITE_I2C,
	IPC_REPLY_SPOOL_IMAGE_OUT,
	
	//IPC_REPLY_GET_LINE_STARTED,
	//IPC_REPLY_FIRST_BLOCK,
	
	
	
};

bool print_flush = false;

void flush_incoming_buffer(ipc_serial_driver *s)
{
	while(s->byte_incoming_ready())	// any incoming bytes
	{
		// have to be careful about this line on the slave
		byte temp = s->read_byte();		// throw them away
		(void)temp;
		if(print_flush)
		{
			dwrite_string("flush>"); dwrite_int(temp); dwrite_cr();
		}
	}
}


void flush_incoming_packet(ipc_serial_driver *s)
{
	//
	// Wait for any trailing characters to finish coming in... i.e. flush the buffer
	// 
	if(print_flush) { dwrite_line("Flush Start"); }
	unsigned int timer_start = cpu_ticks();
	// 16 characters is 160 bits. We allow 200 bits to allows some overhead for gaps, etc.
	// 200 bits (20 characters) = 217us
	// 1000 bits = 1ms (ish)
	// can't make this too long ... sitting here we are not controlling the robot!
	while((cpu_ticks()-timer_start) < 200*IPC_BIT_TIME_IN_cpu_ticks)
	{
		//write_int(cpu_ticks()-timer_start); write_string("<");	
		flush_incoming_buffer(s);
	}
	flush_incoming_buffer(s);

	if(print_flush) { dwrite_line("Flush End"); }
}


IPC_protocol_engine::IPC_protocol_engine(ipc_serial_driver *serial_driver)
: incoming_length(0), driver(serial_driver), reply_timeout_time(pclk/100), retry_failed_flag(false)
{
	flush_incoming_buffer(serial_driver);			// always flush the rx buffer
	print_flush = true;
}

void IPC_protocol_engine::service()
{
	if(driver->byte_incoming_ready())
	{
		incoming[incoming_length] = driver->read_byte();
		dwrite_string("base-got-byte>>>"); dwrite_int(incoming[incoming_length]); dwrite_cr();
		incoming_length++;
	}
	
	unsigned int time = cpu_ticks()-packet_start_time;
	if(time > reply_timeout_time)
	{
		// 1ms has elapsed... retry
		dwrite_line("Reply timeout, clocks=",time);
		retry_code();
	}
	// timeout could also be handled here...
	// 1ms after first byte (since it has the answer)
	// 50ms before first byte (since it miht be processing the answer)
	// Maybe we should have a interrim packet from the camera that says "processing" for long replies?
}

byte IPC_protocol_engine::get_reply()
{
	if(incoming_length==0)
	{
		return IPC_COMMAND_NONE;
	}
	else
	{
		return incoming[0];
	}
}

void IPC_protocol_engine::handle_reply()
{
	//byte command = get_reply();
	//if(command == IPC_REPLY_BAD_CHECKSUM || command == IPC_REPLY_COMMAND_TIMEOUT || command == IPC_REPLY_INVALID_COMMAND)
	//if(command == IPC_REPLY_RETRY_PLEASE)
	//{
	
	/* doesn't matter what it is - retry since that's the only option here */
	//const byte lpc_reply_fault_length = 2;
	//if(reply_recieved_correctly(lpc_reply_fault_length))
	//{
	byte command = get_reply();
	if(command == IPC_COMMAND_NONE)
	{
		return;				// nothing to do here
	}
	//else if(command == IPC_REPLY_RETRY_PLEASE)
	//{
	//	dwrite_line("MASTER: Got retry request");
	//}
	else if(command == IPC_REPLY_BAD_CHECKSUM)
	{
		dwrite_line("REPLY: Bad Checksum");
	}
	else if(command == IPC_REPLY_INVALID_COMMAND)
	{
		dwrite_line("REPLY: Invalid Command");
	}
	else if(command == IPC_REPLY_COMMAND_TIMEOUT)
	{
		dwrite_line("REPLY: command timeout");
	}
	else
	{
		dwrite_string("MASTER: Got funny reply - "); dwrite_int(command); dwrite_cr();
	}
	
	retry_code();
	//} // else we wait
	//}
	//else // don't know how to handle this, let lower level handle
	//{
	//	...errr...
	//}
	
}

void IPC_protocol_engine::retry_code()
{
	if(retry_count == 10)
	{ 
		dwrite_line("IPC MASTER RETRYING FAILURE"); 
		retry_failed_flag = true;
	}
	else
	{
		retry_count++;
		dwrite_line("IPC MASTER: Flush and Retrying");
		flush_incoming_packet(driver);
		incoming_length = 0;
		send_old_command();
	}
}

byte IPC_protocol_engine::rx_length()
{
	return incoming_length;
}

bool IPC_protocol_engine::checksum_ok(byte this_commands_length)
{
	byte checksum = 0;
	for(int i=0; i<this_commands_length-1; i++)
	{
		checksum += incoming[i];
	}
	
	bool success = checksum == incoming[this_commands_length-1];
	
	if(!success)		// checksum automatically does tries
	{
		retry_code();
	}
	
	return success;
}

//
//
//
bool IPC_protocol_engine::reply_recieved_correctly(byte this_commands_length)
{
	return rx_length() == this_commands_length && checksum_ok(this_commands_length);
}

//
//
//
void IPC_protocol_engine::set_new_command(byte* data, byte length)
{
	// first thing - write out bytes as soon as possible...
	byte checksum = 0;
	byte* out = outgoing_copy;
	//byte* ptrcopy = data;
	for(int i=0; i<length; i++)
	{
		byte thisbyte = *data;
		
		driver->write_byte(thisbyte);
		*out = thisbyte;
		data++;
		out++;
		
		checksum+=thisbyte;
	}
	driver->write_byte(checksum);

	// we could do this as we go along, but to avoid delays in the above loop
	//byte* out = outgoing_copy;
	//for(int i=0; i<length; i++)
	//{
	//	*out = *ptrcopy;
	//	out++; ptrcopy++;
	//}
	
	outgoing_length = length;
	retry_count = 0;

	packet_start_time = cpu_ticks();
}

void IPC_protocol_engine::send_old_command()
{
	// first thing - write out bytes as soon as possible...
	byte checksum = 0;
	byte* out = outgoing_copy;
	for(int i=0; i<outgoing_length; i++)
	{
		byte thisbyte = *out;
		driver->write_byte(thisbyte);		
		out++;
		checksum+=thisbyte;
	}
	driver->write_byte(checksum);
	
	packet_start_time = cpu_ticks();
}


byte*  IPC_protocol_engine::get_current_data()
{
	return incoming+1;
}


int IPC_protocol_engine::number_of_data_bytes()
{
	if(incoming_length <= 2)
	{
		return 0;
	}
	return incoming_length-2;
}

// ------------------------------------------------------------------------------------


IPC_led_command::IPC_led_command(ipc_serial_driver *serial_driver, byte which_led, byte state)
: complete(false), base(serial_driver)
{
	byte led_command[] = { IPC_COMMAND_LED, 0, 0};
	led_command[1] = which_led;
	led_command[2] = state;
	base.set_new_command(led_command, sizeof(led_command));
}

void IPC_led_command::service()
{
	if(completed()) return;
	
	// ensure service routine is called
	base.service();
	
	byte command = base.get_reply();
	if(command == IPC_REPLY_LED)
	{
		const byte lpc_reply_led_length = 2;
		if(base.reply_recieved_correctly(lpc_reply_led_length))
		{
			dwrite_line("MASTER: Got good LED reply");
			complete = true;
		} // else we wait
	}
	else // don't know how to handle this, let lower level handle
	{
		base.handle_reply();
	}
}


bool IPC_led_command::command_failed()
{
	return base.retry_failed();
}

// ------------------------------------------------------------------------------------


IPC_device_data_command::IPC_device_data_command(ipc_serial_driver *serial_driver)
: complete(false), base(serial_driver)
{
	byte command[] = { IPC_COMMAND_DEVICE_DATA };
	base.set_new_command(command, sizeof(command));
}

void IPC_device_data_command::service()
{
	if(completed()) return;
	
	// ensure service routine is called
	base.service();
	
	byte command = base.get_reply();
	if(command == IPC_REPLY_DEVICE_DATA)
	{
		const byte lpc_reply_led_length = 3;
		if(base.reply_recieved_correctly(lpc_reply_led_length))
		{
			dwrite_line("MASTER: Got good device data reply");
			complete = true;
			
			protocol_version = *base.get_current_data();

		} // else we wait
	}
	else // don't know how to handle this, let lower level handle
	{
		base.handle_reply();
	}
}


bool IPC_device_data_command::command_failed()
{
	return base.retry_failed();
}

// ------------------------------------------------------------------------------------


IPC_button_command::IPC_button_command(ipc_serial_driver *serial_driver)
: complete(false), base(serial_driver)
{
	byte command[] = { IPC_COMMAND_READ_BUTTON };
	base.set_new_command(command, sizeof(command));
}

void IPC_button_command::service()
{
	if(completed()) return;
	
	// ensure service routine is called
	base.service();
	
	byte command = base.get_reply();
	if(command == IPC_REPLY_READ_BUTTON)
	{
		const byte lpc_reply_led_length = 3;
		if(base.reply_recieved_correctly(lpc_reply_led_length))
		{
			dwrite_line("MASTER: Got good device data reply");
			complete = true;
			
			button_data = *base.get_current_data();
			
		} // else we wait
	}
	else // don't know how to handle this, let lower level handle
	{
		base.handle_reply();
	}
}


bool IPC_button_command::command_failed()
{
	return base.retry_failed();
}

// ------------------------------------------------------------------------------------


IPC_i2c_write::IPC_i2c_write(ipc_serial_driver *serial_driver, byte address, byte data)
: complete(false), base(serial_driver)
{
	byte command[] = { IPC_COMMAND_WRITE_I2C, address, data };
	base.set_new_command(command, sizeof(command));
}

void IPC_i2c_write::service()
{
	if(completed()) return;
	
	// ensure service routine is called
	base.service();
	
	byte command = base.get_reply();
	if(command == IPC_REPLY_WRITE_I2C)
	{
		const byte lpc_reply_led_length = 3;
		if(base.reply_recieved_correctly(lpc_reply_led_length))
		{
			dwrite_line("MASTER: Got good i2c write reply");
			complete = true;
			
			if(*base.get_current_data())
			{
				write_success_flag = true;
			}
			else
			{
				write_success_flag = false;
			}
			
		} // else we wait
	}
	else // don't know how to handle this, let lower level handle
	{
		base.handle_reply();
	}
}


bool IPC_i2c_write::command_failed()
{
	return base.retry_failed();
}

// ------------------------------------------------------------------------------------


IPC_i2c_read::IPC_i2c_read(ipc_serial_driver *serial_driver, byte address)
: complete(false), base(serial_driver)
{
	byte command[] = { IPC_COMMAND_READ_I2C, address };
	base.set_new_command(command, sizeof(command));
}

void IPC_i2c_read::service()
{
	if(completed()) return;
	
	// ensure service routine is called
	base.service();
	
	byte command = base.get_reply();
	if(command == IPC_REPLY_READ_I2C)
	{
		const byte lpc_reply_led_length = 3;
		if(base.reply_recieved_correctly(lpc_reply_led_length))
		{
			dwrite_line("MASTER: Got good i2c read reply");
			complete = true;
			
			data_store = *base.get_current_data();
			
		} // else we wait
	}
	else // don't know how to handle this, let lower level handle
	{
		base.handle_reply();
	}
}


bool IPC_i2c_read::command_failed()
{
	return base.retry_failed();
}

// ------------------------------------------------------------------------------------


IPC_spool_image_out::IPC_spool_image_out(ipc_serial_driver *serial_driver, byte which_type, byte which_level)
: complete(false), base(serial_driver)
{
	byte spool_command[] = { IPC_COMMAND_SPOOL_IMAGE_OUT, 0, 0};
	spool_command[1] = which_type;
	spool_command[2] = which_level;
	base.set_new_command(spool_command, sizeof(spool_command));
}

void IPC_spool_image_out::service()
{
	if(completed()) return;
	
	// ensure service routine is called
	base.service();
	
	byte command = base.get_reply();
	if(command == IPC_REPLY_SPOOL_IMAGE_OUT)
	{
		const byte lpc_reply_led_length = 2;
		if(base.reply_recieved_correctly(lpc_reply_led_length))
		{
			dwrite_line("MASTER: Got good spool reply");
			complete = true;
		} // else we wait
	}
	else // don't know how to handle this, let lower level handle
	{
		base.handle_reply();
	}
}


bool IPC_spool_image_out::command_failed()
{
	return base.retry_failed();
}

// ------------------------------------------------------------------------------------


IPC_init_camera::IPC_init_camera(ipc_serial_driver *serial_driver)
: complete(false), base(serial_driver)
{
	byte spool_command[] = { IPC_COMMAND_INIT_CAMERA};
	base.set_new_command(spool_command, sizeof(spool_command));
}

void IPC_init_camera::service()
{
	if(completed()) return;
	
	// ensure service routine is called
	base.service();
	
	byte command = base.get_reply();
	if(command == IPC_COMMAND_INIT_CAMERA)
	{
		const byte lpc_reply_led_length = 2;
		if(base.reply_recieved_correctly(lpc_reply_led_length))
		{
			dwrite_line("MASTER: Got good cam reply");
			complete = true;
		} // else we wait
	}
	else // don't know how to handle this, let lower level handle
	{
		base.handle_reply();
	}
}


bool IPC_init_camera::command_failed()
{
	return base.retry_failed();
}

// ------------------------------------------------------------------------------------


IPC_get_lines::IPC_get_lines(ipc_serial_driver *serial_driver, byte RG_level, byte blue_compare, line_list_custom_vector& lines1_out, line_list_custom_vector& lines2_out)
: complete(false), base(serial_driver), driver(serial_driver), lines1(lines1_out), lines2(lines2_out)
{
	byte spool_command[] = { IPC_COMMAND_PROCESS_IMAGE_BASE, 0, 0};
	spool_command[1] = RG_level;
	spool_command[2] = blue_compare;
	base.set_new_command(spool_command, sizeof(spool_command));
}

void IPC_get_lines::service()
{
	if(completed()) return;
	
	// ensure service routine is called
	base.service();
	
	byte command = base.get_reply();
	if(command == IPC_COMMAND_PROCESS_IMAGE_BASE)
	{
		const byte lpc_reply_led_length = 2;
		if(base.reply_recieved_correctly(lpc_reply_led_length))
		{
			dwrite_line("MASTER: Got good get lines reply");
			complete = true;

			receive_processed_image_lines(*driver, lines1, lines2);			
		} // else we wait
	}
	else // don't know how to handle this, let lower level handle
	{
		base.handle_reply();
	}
}


bool IPC_get_lines::command_failed()
{
	return base.retry_failed();
}


// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// SLAVE CODE
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------

void ipc_slave_send_error(ipc_serial_driver *s, byte error_type)
{
	//dwrite_line("E");

	// wait for the rest of the packet to come back
	flush_incoming_packet(s);

	// Now send back retry request
	s->write_byte(error_type);			// command
	s->write_byte(error_type);			// checksum
}

bool read_bytes(ipc_serial_driver *s, byte* incoming, byte number)
{
	unsigned int timer_start = cpu_ticks();
	
	while(number)
	{
		while(!s->byte_incoming_ready())
		{
			// OLD: if we don't get any bytes in 1ms, bomb out (80 character worth, approx, at 800kbps)
			// NEW: if we don't get any bytes in 5 full packets worth, bomb out.
			// 5 packet worth = 5*16*10 bits
			// 
			if((cpu_ticks()-timer_start) > IPC_BIT_TIME_IN_cpu_ticks*5*16*10)
			{
				dwrite_line("TO");
				return false;
			}
			// timer
		}
		*incoming = s->read_byte();
		//dwrite_string(";"); dwrite_int(*incoming); dwrite_cr();
		incoming++;
		number--;
	}
	
	return true;
}


#include "hardware_manager.h"



static ipc_serial_driver s;
void ipc_slave_service()
{
	byte buffer[16];
	bool success;
	
	if(s.byte_incoming_ready())
	{
		byte command = s.read_byte();
		//dwrite_string(":"); dwrite_int(command); dwrite_cr();
		if(command == IPC_COMMAND_DEVICE_DATA)
		{
			//dwrite_line("DC");
			success = read_bytes(&s, buffer, 1);
			if(!success)
			{
				ipc_slave_send_error(&s,IPC_REPLY_COMMAND_TIMEOUT);
			}
			else
			{
				byte checksum = buffer[0];
				if(checksum==IPC_COMMAND_DEVICE_DATA)
				{
					s.write_byte(IPC_REPLY_DEVICE_DATA);
					s.write_byte(IPC_PROTOCOL_VERSION);
					s.write_byte(IPC_PROTOCOL_VERSION+IPC_REPLY_DEVICE_DATA);
				}
				else
				{
					ipc_slave_send_error(&s, IPC_REPLY_BAD_CHECKSUM);
				}
			}
		}
		if(command == IPC_COMMAND_READ_BUTTON)
		{
			//dwrite_line("DC");
			success = read_bytes(&s, buffer, 1);
			if(!success)
			{
				ipc_slave_send_error(&s,IPC_REPLY_COMMAND_TIMEOUT);
			}
			else
			{
				byte checksum = buffer[0];
				if(checksum==IPC_COMMAND_READ_BUTTON)
				{
					byte button_data = Hardware_Manager::Instance()->button1()->pressed();
					
					s.write_byte(IPC_REPLY_READ_BUTTON);
					s.write_byte(button_data);
					s.write_byte(button_data+IPC_REPLY_READ_BUTTON);
				}
				else
				{
					ipc_slave_send_error(&s, IPC_REPLY_BAD_CHECKSUM);
				}
			}
		}
		else if(command == IPC_COMMAND_READ_I2C)
		{
			//dwrite_line("DC");
			success = read_bytes(&s, buffer, 2);
			if(!success)
			{
				ipc_slave_send_error(&s,IPC_REPLY_COMMAND_TIMEOUT);
			}
			else
			{
				byte address = buffer[0];
				byte checksum = buffer[1];
				if(checksum==IPC_COMMAND_READ_I2C+address)
				{
					byte i2c_data = 0;
					bool i2c_status = cc3_camera_get_raw_register(address, &i2c_data);
					if(i2c_status==false)
					{
						i2c_data = 0;
					}
					s.write_byte(IPC_REPLY_READ_I2C);
					s.write_byte(i2c_data);
					s.write_byte(i2c_data+IPC_REPLY_READ_I2C);
				}
				else
				{
					ipc_slave_send_error(&s, IPC_REPLY_BAD_CHECKSUM);
				}
			}
		}
		else if(command == IPC_COMMAND_WRITE_I2C)
		{
			//dwrite_line("DC");
			success = read_bytes(&s, buffer, 3);
			if(!success)
			{
				ipc_slave_send_error(&s,IPC_REPLY_COMMAND_TIMEOUT);
			}
			else
			{
				byte address = buffer[0];
				byte data = buffer[1];
				byte checksum = buffer[2];
				if(checksum==IPC_COMMAND_WRITE_I2C+address+data)
				{
					byte i2c_status = cc3_camera_set_raw_register(address, data);
					
					s.write_byte(IPC_REPLY_WRITE_I2C);
					s.write_byte(i2c_status);
					s.write_byte(i2c_status+IPC_REPLY_WRITE_I2C);
				}
				else
				{
					ipc_slave_send_error(&s, IPC_REPLY_BAD_CHECKSUM);
				}
			}
		}
		else if(command == IPC_COMMAND_LED)
		{
			//dwrite_line("LC");
			success = read_bytes(&s, buffer, 3);
			if(!success)
			{
				ipc_slave_send_error(&s, IPC_REPLY_COMMAND_TIMEOUT);
			}
			else
			{
				byte which_led = buffer[0];		// which led?
				byte led_state = buffer[1];		// on or off?
				byte checksum = buffer[2];		// checksum
				if(checksum==IPC_COMMAND_LED+which_led+led_state)
				{
					//dwrite_string("Led"); dwrite_int(which_led); dwrite_string(","); dwrite_int(led_state); dwrite_cr();
					s.write_byte(IPC_REPLY_LED);
					s.write_byte(IPC_REPLY_LED);
#ifndef TERMINAL_BUILD_TEST_FRAMEWORK
					Hardware_Manager* hw = Hardware_Manager::Instance();
#endif
					if(which_led == 0)
					{
						dwrite_line("sLED", led_state);
#ifndef TERMINAL_BUILD_TEST_FRAMEWORK
						hw->status_LED().set(led_state);
#endif
					}
					else if(which_led == 1)
					{
						dwrite_line("aLED", led_state);
#ifndef TERMINAL_BUILD_TEST_FRAMEWORK
						hw->activity_LED().set(led_state);
#endif
					}
					else
					{
						dwrite_line("eLED", led_state);
#ifndef TERMINAL_BUILD_TEST_FRAMEWORK
						hw->error_LED().set(led_state);
#endif
					}
				}
				else
				{
					ipc_slave_send_error(&s, IPC_REPLY_BAD_CHECKSUM);
				}
			}
		}
		else if(command == IPC_COMMAND_SPOOL_IMAGE_OUT)
		{
			success = read_bytes(&s, buffer, 3);
			if(!success)
			{
				ipc_slave_send_error(&s, IPC_REPLY_COMMAND_TIMEOUT);
			}
			else
			{
				byte which_type = buffer[0];	// which type of image
				byte which_level = buffer[1];		// RG level
				byte checksum = buffer[2];		// checksum
				if(checksum==IPC_COMMAND_SPOOL_IMAGE_OUT+which_type+which_level)
				{
					s.write_byte(IPC_REPLY_SPOOL_IMAGE_OUT);
					s.write_byte(IPC_REPLY_SPOOL_IMAGE_OUT);
					
					// we do it after the reply because we don't want to hit the time out
					camera_image_port cam;
					cam.get_image(which_type, which_level);
				}
				else
				{
					ipc_slave_send_error(&s, IPC_REPLY_BAD_CHECKSUM);
				}
			}
		}
		else if(command == IPC_COMMAND_INIT_CAMERA)
		{
			success = read_bytes(&s, buffer, 1);
			if(!success)
			{
				ipc_slave_send_error(&s, IPC_REPLY_COMMAND_TIMEOUT);
			}
			else
			{
				byte checksum = buffer[0];		// checksum
				if(checksum==IPC_COMMAND_INIT_CAMERA)
				{
					s.write_byte(IPC_COMMAND_INIT_CAMERA);
					s.write_byte(IPC_COMMAND_INIT_CAMERA);
					
					// we do it after the reply because we don't want to hit the time out
					
					camera_image_port cam;
					if(cam.setup_camera() == false)
					{
						halt("Setup camera Fail");
					}
				}
				else
				{
					ipc_slave_send_error(&s, IPC_REPLY_BAD_CHECKSUM);
				}
			}
		}
		else if(command == IPC_COMMAND_PROCESS_IMAGE_BASE)
		{
			success = read_bytes(&s, buffer, 3);
			if(!success)
			{
				ipc_slave_send_error(&s, IPC_REPLY_COMMAND_TIMEOUT);
			}
			else
			{
				byte which_level = buffer[0];	// which level of R>G setting
				byte blue_compare = buffer[1];	// whether it does a blue compare or the normal green compare
				byte checksum = buffer[2];		// checksum
				if(checksum==IPC_COMMAND_PROCESS_IMAGE_BASE+which_level+blue_compare)
				{
					s.write_byte(IPC_COMMAND_PROCESS_IMAGE_BASE);
					s.write_byte(IPC_COMMAND_PROCESS_IMAGE_BASE);
					
					// we do it after the reply because we don't want to hit the time out
					camera_image_port cam;
					cam.process_image(s, which_level, blue_compare);
				}
				else
				{
					ipc_slave_send_error(&s, IPC_REPLY_BAD_CHECKSUM);
				}
			}
		}
		else	// we have a problem
		{
			//dwrite_string("CE"); dwrite_int(command); dwrite_cr();
			ipc_slave_send_error(&s, IPC_REPLY_INVALID_COMMAND);
		}
	}
}

// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------
//
// ------------------------------------------------------------------------------------
//#if 0
///*
//struct command_entry {
//	byte message_id;
//	byte command_length; 
//	byte reply_length;
//};
//
//
//command_entry commands {
//	// 
//};
//*/
//
//enum { 
//	ipc_version_check,
//	ipc_led1,
//	ipc_number_of_commands
//};
//
// includes data, but not commands or checksums.
//byte command_data_lengths[] = {
//	1,
//	1
//};
//
//struct {
//	byte command;
//	byte data_length;
//	byte number_of_commands;
//};
//
//enum {
//	ipc_checksum_error,
//	ipc_command_out_of_range,
//	ipc_done,
//};
//
//ipc_command_t command_array[ipc_number_of_commands];
//
//const int number_of_replies = 
//reply_array[ipc_number_of_replies];
//
//const int block_size = 16;
//
//send_block()
//{
//	// wait until the fast time has expired (= transfer time + 8 characters)
//	// this is only 260us
//	while(current_time > time_before_send_again)
//	{
//		// sit idle!
//	}
//	// send all the bytes as fast as possible...
//	for(i=0; i<16; i++)
//	{
//		write_byte(data[i]);
//	}
//	
//	
//	time_before_send_again = current_time + 24 bytes.
//	
//	
//}
//
// once we have bytes to receive, we try to get them all at once...
//receive_block()
//{
//}
//
//void ipc_command_processor::decode_command()
//{
//	byte command = read_byte();
//	if(command > ipc_number_of_commands)
//	{
//		ipc_reply(ipc_command_out_of_range);
//		return;
//	}
//	byte buffer[15];
//	byte length = command_data_lengths[command]+1;	// +1 for checksum
//	byte checksum = 0;
//	for(int i=0; i<length; i++)
//	{
//		byte data = read_byte();
//		buffer[i] = data;
//		checksum += data;
//	}
//	if(checksum != 0)
//	{
//		
//	}
//	
//	switch(command)
//	{
//		case ipc_version_check:
//			byte checksum = 
//			break;
//		case ipc_led1:
//			break;
//		default:
//			ipc_reply(ipc_command_out_of_range);
//			break;
//	}
//}
//
//
//
//class ipc_transport {
//public:
//	ipc_protocol();
//	void service();
//
// calling send and receive whilst it's busy is illegal so don't do it.
//	void send_packet();
//	bool data_waiting();
//	void receive_packet();
//	bool idle();
//private:
//	enum ipcp_t {
//		ipcp_waiting,
//		ipcp_sending,
//		ipcp_send_error_retry,
//		ipcp_receive,
//		ipcp_receive_error_wait 
//	};
//	ipcp_t state;
//	int time;
//	byte input_data[16];		// store for fetch
//	byte output_data[16];		// stored for retry
//	byte num_output_chars;
//	byte num_input_chars;
//};
//
//
//
//
//
//ipc_transport::ipc_transport()
//: state(ipcp_waiting),
//time(0)
//{
//	// nothing to do here
//}
//
//
//
//
//void ipc_transport::send_packet(byte* data, byte num_chars)
//{
//	if(state!=ipcp_waiting)
//	{
//		write_line("IPC Send before idle");
//		halt();
//	}
//	
//	for(int i=0; i<num_chars; i++)
//	{
//		output_data[i] = data[i];
//	}
//
//	// seperately send to serial port as fast as possible - i.e. minimal gaps
//	for(int i=0; i<num_chars; i++)
//	{
//		output_data[i] = *data;
//		data++;
//	}
//}
//
//
// Return true if idle
//
//bool ipc_transport::idle()
//{
//	return state==ipcp_waiting;
//}
//
//
//
//
//void ipc_transport::service()
//{
//	switch(state)
//	{
//		case ipcp_waiting:
//			// nothing to do
//			break;
//			
//		case ipcp_sending:
//			
//			break;
//
//		case ipcp_send_error_retry:
//			break;
//
//		case ipcp_receive:
//			break;
//
//		case ipcp_receive_error_wait:
//			// recieve is responsible for waiting beofre
//			break;
//				
//		default:
//			// halt here? Maybe just reset to waiting
//			state=ipcp_waiting;
//			break;
//	}
//	
//}
//
//
//class ipc_slave_protocol {
//public:
//	ipc_slave_protocol();
//	void service();
//private:
//	ipc_transport ipc;
//	enum slave_states {
//		slave_waiting,
//		slave_send
//	};
//	slave_states state;
//};
//
//
//
//void ipc_slave_protocol::ipc_slave_protocol()
//{
//}
//
//
//
// implement the slave protocol
//
// It's only at this layer we differentiate between messages.
//
// Once this gets a message in, it actions this message straight away.
// It doesn't return to the main loop until this is done.
// Service should still be called multiple times, but be aware that 
// for intensive tasks this might be away for many milliseconds.
//
// This is ok for the slave, because it shouldn't be doing anything else.
//
//void ipc_slave_protocol::service()
//{
//	if(state == slave_waiting)
//	{
//		if(ipc.receive_message())
//		{
//			decode_message();
//			action_message();
//			send_reply();
//			state = slave_send;
//		}
//		return;
//	}
//
//	if(state == slave_send)
//	{
//		if(!ipc.sending_message())
//		{
//			state = slave_waiting;
//		}
//		
//		return;
//	}
//
//}
//
//
// implement the master protocol
//
// It's only at this layer we differentiate between messages.
//
//class ipc_master_protocol {
//public:
//	ipc_master_protocol();
//	void service();
//	//void send_identify(identify_result* my_result);
//	void send_led(int led_number, bool state);
//	bool busy();
//
//private:
//	ipc_transport ipc;
//	enum master_states {
//		master_waiting,
//		master_awaiting_reply,
//	};
//	master_states state;
//	void busy_check();
//	//ipc_result_class *current_result;
//	//byte *current_result;
//};
//
//
// Master constructor ... mainly for variable initialisation
//
//void ipc_master_protocol::ipc_master_protocol()
//: state(master_waiting)
//{
//	
//}
//
//
//const int ipc_max_packet_length = 16;
//
//void ipc_master_protocol::send_led(int led_number, bool state)
//{
//	busy_check();
//
//	byte data[ipc_max_packet_length];
//	data[0] = LED_COMMAND;
//	data[1] = led_number;
//	data[2] = state;
//	data[3] = ipc_checksum(LED_COMMAND, data);
//
//	ipc.send_packet(data);
//}
//
//
//void ipc_master_protocol::busy_check()
//{
//	if(busy()) { write_line("IPC BUSY VIOLATION"); halt(); }
//}
//
//void ipc_master_protocol::busy()
//{
//	// busy when we are not waiting
//	return state!=master_waiting;
//}
//
//
//void ipc_master_protocol::service()
//{
//	// waiting does nothing
//	if(state == master_waiting)
//	{
//		return;
//	}
//
//	if(state == master_awaiting_reply)
//	{
//		if(ipc.receive_message())
//		{
//			// do some post processing here, like fill in reply packet
//			
//			// *current_result = input_data[0];
//			// *current_result = input_data[1];
//			// etc...
//			state = master_waiting;
//		}
//	}
//}
//
//#endif
//
//
