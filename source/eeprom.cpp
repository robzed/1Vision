/*
 *  eeprom.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 27/10/2008.
 *  Copyright (C) 2008 Rob Probin.
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

#include "eeprom.h"
#include "language_support.h"
#include "timing.h"

#define DEBUGGING_ENABLED 0			// turn this on for read/write debugging
#include "debug_support.h"

const unsigned int EEPROM_SIZE_IN_BYTE = 16384;

const unsigned int EEPROM_ROUTINE_WORD_SIZE = 4;

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   eeprom::read
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 28 October 2008
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----

unsigned int eeprom::read(unsigned short address)
{
	//
	// First write the address
	//
	dwrite_line("Write address");
	address = address*EEPROM_ROUTINE_WORD_SIZE;		// we only read and write words, so adjust address based on multiples of that
	unsigned char packet[2];
	
	packet[0] = address >> 8;
	packet[1] = address & 0xff;
	
	checked_send_data(packet, 2);

	dwrite_line("Read data");
	//
	// Now read the data
	//
	unsigned char data[EEPROM_ROUTINE_WORD_SIZE];
	i2c.rx_data(data, EEPROM_ROUTINE_WORD_SIZE);

	wait_for_end();
	if(i2c.error())		// should never be busy at this point - so don't check for busy
	{
		halt("EE fail:read");
	}
	
	
	unsigned int return_val;
	
	// big endian
	return_val = (data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3];
	
	return return_val;
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   eeprom::write
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 28 October 2008
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----

void eeprom::write(unsigned short address, unsigned int data32)
{
	//
	// Write address and data in one go
	//
	dwrite_line("Write address+data");
	unsigned char packet[2+EEPROM_ROUTINE_WORD_SIZE];

	address = address*EEPROM_ROUTINE_WORD_SIZE;		// we only read and write words, so adjust address based on multiples of that

	// write the address, high byte first
	packet[0] = address >> 8;
	packet[1] = address & 0xff;
	
	// write the data, big endian
	packet[2] = (data32 >> 24) & 0xff;
	packet[3] = (data32 >> 16) & 0xff;
	packet[4] = (data32 >> 8) & 0xff;
	packet[5] = data32 & 0xff;
	
	checked_send_data(packet, 6);
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   eeprom::checked_send_data
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 1 November 2008
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----

void eeprom::checked_send_data(unsigned char* data, int bytes)
{
	// write time should be 5ms - so we allow 10ms (old chips used to be 10. Also 1 more for timer inaccuracies)
	msTimer timer(10);

	do
	{
		i2c.send_data(data, bytes);
		wait_for_end();
		if(!i2c.error())
		{
			break;
		}
		if((!i2c.device_busy()))
		{
			halt("EE fail:send1");			
		}
		
	} while(!timer.expired());
	if(i2c.error())
	{
		halt("EE fail:send2");
	}
	
}


// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   eeprom::wait_for_end
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 29 October 2008
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----

void eeprom::wait_for_end()
{
	while(! i2c.completed())
	{
		i2c.service_i2c();
	}
}



// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   eeprom::instance
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 28 October 2008
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----
//static eeprom* eeprom::instance()
//{
//	// local static instance is good for single threaded applications
//	static eeprom eeprom_instance;
//
//	return &eeprom_instance;
//}


#if 0
void run_unit_test()
{
	write_line("EEPROM write test");
	eeprom ee;
	write_hex(ee.read(2));
	ee.write(2, 0x12345678);
	write_hex(ee.read(2));
	ee.write(2, 0x11111111);
	write_hex(ee.read(2));
	ee.write(2, 0x22222222);
	write_hex(ee.read(2));
	write_line("End");
}
#endif
