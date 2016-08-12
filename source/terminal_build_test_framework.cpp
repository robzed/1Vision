/*
 *  terminal_build_test_framework.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 20/11/2006.
 *  Copyright 2006 Rob Probin.
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

#include "terminal_build_test_framework.h"

#include <stdio.h>
#include <math.h>	// for fmod

//#include "command_line.h"
#include "ipc_serial_driver.h"
//#include "DriverServices/DriverServices.h"
#include <ApplicationServices/ApplicationServices.h>
#include "interprocessor_comms.h"

ipc_serial_driver* slave_serial;
ipc_serial_driver* master_serial;

//char help_command[] = "help help";

void emulate_hardware_action();
AbsoluteTime base_time;


int main()
{
	printf("Robot Test Framework\n");

#ifndef TERMINAL_BUILD_TEST_FRAMEWORK
	printf("You need to define TERMINAL_BUILD_TEST_FRAMEWORK in your project for a Mac/PC build\n");
#endif
	base_time = UpTime();
	
	slave_serial = new ipc_serial_driver;
	master_serial = new ipc_serial_driver;
	
#define IPC_PROTOCOL_TEST 1
#if IPC_PROTOCOL_TEST


	IPC_led_command *cmd_ipc_led_ptr;
	cmd_ipc_led_ptr = new IPC_led_command(master_serial, 1, 1);

	while(1)
	{
		// run the slave
		ipc_slave_service(slave_serial);
		
		// run the master
		if(cmd_ipc_led_ptr)
		{		
			cmd_ipc_led_ptr->service();
			if(cmd_ipc_led_ptr->completed())
			{
				delete cmd_ipc_led_ptr;
				cmd_ipc_led_ptr = 0;
			}
		}
		
		emulate_hardware_action();

	}

#endif // IPC_PROTOCOL_TEST

	
#define IPC_SERIAL_DRIVER_TEST 0 
#if IPC_SERIAL_DRIVER_TEST	
	//
	// put test code here
	//
	//Command_line cmd;
	
	for(int i='A'; i<'G'; i++)
	{
		slave_serial->write_byte(i);
		if(master_serial->byte_incoming_ready())
		{
			printf("Next byte=%i\n",(int)master_serial->read_byte());
		}
	}
	
	//cmd.execute(help_command);
	while(1)
	{
		emulate_hardware_action();
		//cmd.run();
		if(master_serial->byte_incoming_ready())
		{
			printf("Next byte=%i\n",(int)master_serial->read_byte());
		}
		
	}
#endif // IPC_SERIAL_DRIVER_TEST
	
}


// SUPPORT FUNCTIONS
// maybe print needs to be emulated in pc_uart.h?
void write_line(const char *string)
{
	printf("%s\n", string);
}

void write_line(const char *string, int value)
{
	printf("%s%i\n", string, value);
}


void write_cr()
{
	printf("\n");
}

void write_string(const char *string)
{
	printf("%s", string);
}


void write_int(int value)
{
	printf("%i", value);
}
			   
int read_char() 
{ 
	return getchar(); 
}

void halt()
{
	write_cr();
	write_line("*HALT*");
	
    while(1)
		;
}

//
// Halt with a message (for debug)
//
void halt(const char* error_message)
{
	write_line(error_message);
	halt();
}

void emulate_copy_buffer(ipc_serial_driver* from, ipc_serial_driver* to)
{
	// if there are bytes available in the source
	while(from->emulation_outbytes)
	{
		// we can only copy if there are spaces available in input buffer of the destination
		if(to->emulation_inbytes >= ipc_serial_driver::MAX_BYTES_IN_UART_FIFO)
		{
			break;
		}
		
		// copy bytes across from the source out to the destination in
		to->emulation_inbuffer[to->emulation_inbytes] = from->emulation_outbuffer[0];
		to->emulation_inbytes++;
		// now remove character from 'from' buffer
		for(int i=0; i < (ipc_serial_driver::MAX_BYTES_IN_UART_FIFO-1); i++)		// theoretically we only need to copy emulation_outbytes here
		{
			from->emulation_outbuffer[i] = from->emulation_outbuffer[i+1];
		}
		from->emulation_outbytes--;	
	}
}

//
// This does various hardware things, including copying between the two serial ports
//
void emulate_hardware_action()
{
	if(slave_serial==0) halt("No slave serial");
	if(master_serial==0) halt("No master serial");
	
	//
	// Deal with the slave output bytes ... copy across to master input buffer
	//
	emulate_copy_buffer(slave_serial, master_serial);
	
	emulate_copy_buffer(master_serial, slave_serial);
}


unsigned int nanoseconds()
{
	// do your timed stuff here!

	AbsoluteTime new_time = UpTime();

	Nanoseconds nano = AbsoluteDeltaToNanoseconds( new_time, base_time );
		
	// if you want that in (floating point) seconds:
	//double seconds = ((double) UnsignedWideToUInt64( nano )) * 1e-9;
	double nanosecs = ((double) UnsignedWideToUInt64( nano ));
	
//	return static_cast<unsigned int>(nanoseconds);
	unsigned int nano_return = fmod(nanosecs, static_cast<double>(0xffffffff)+1);
	return nano_return;
}


/*
 *  interprocessor_comms.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 21/07/2007.
 *  Copyright 2007 Lightsoft. All rights reserved.
 *
 */

//#include "ipc_serial_driver.h"
//#include "robot_hardware_defs.h"
//#include "hardware_support.h"

ipc_serial_driver::ipc_serial_driver()
: bytes_transmitted(0), emulation_outbytes(0), emulation_inbytes(0)
{
	printf("<<Initialised ipc_serial_driver>>\n");
}



//
//
//
void ipc_serial_driver::write_byte(byte data)
{
	if(bytes_transmitted >= MAX_BYTES_IN_UART_FIFO)
	{
		// FIFO might be full ... so let's wait for it to be empty
		// 
		// Actually slower than an interrupt because we have to 
		// wait for it to fully empty ... however we should 
		// achieve close to the maximum bandwidth, except
		// for the last few characters before a wait.
		while(emulation_outbytes!=0)
		{
			// wait for it to empty
			emulate_hardware_action();
		}
		
		bytes_transmitted = 0;		// we know the FIFO is empty
	}
	
	//UART1_THR = data;
	printf("<<%i>>",data);
	if(emulation_outbytes>=MAX_BYTES_IN_UART_FIFO) 
	{ halt("Fifo overrun"); }
	emulation_outbuffer[emulation_outbytes] = data;
	emulation_outbytes++;

	
	bytes_transmitted ++;
}

// +---------------------------------------------------------------------------------

bool ipc_serial_driver::byte_incoming_ready()
{
	return emulation_inbytes!=0;
}

// +---------------------------------------------------------------------------------

byte ipc_serial_driver::read_byte()
{
	while(emulation_inbytes==0)
	{
		// wait for a byte to arrive
		emulate_hardware_action();
	}
	
	byte rx_byte = emulation_inbuffer[0];
	for(int i=0; i<(MAX_BYTES_IN_UART_FIFO-1); i++)
	{
		emulation_inbuffer[i] = emulation_inbuffer[i+1];
	}
	emulation_inbytes--;
	
	return rx_byte;
}


