/*
 *  cc3_camera_i2c.cpp
 *  robot_core
 *
 * Modifications Copyright (C) 2008 Rob Probin.
 * Copyright 2006-2007  Anthony Rowe and Adam Goode
 *
 * This file, specifically, was derived from the cc3.c file 
 * which is Apache License 2.0. Therefore this file is also 
 * under that license.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "camera_board_i2c.h"
#include "robot_hardware_defs.h"
#include "hardware_support.h"
#include "camera_board_util.h"

#define DEBUGGING_ENABLED 1
#include "debug_support.h"


//#define _CC3_DEFAULT_PORT_DIR		0x003EFD99
#define _CC3_DEFAULT_PORT_DIR		0x003EBDC9
//#define DEFAULT_PORT_DIR	0x0 | BUF_WEE | CAM_RESET | BUF_WRST | BUF_RRST | BUF_RCK | BUF_RESET

// SCCB Config Constants
#define _CC3_SCCB_PORT_DDR_IDLE		0x003EBDC9
#define _CC3_SCCB_PORT_DDR_READ_SDA	0x007EBDC9
#define _CC3_SCCB_PORT_DDR_READ_SCL	0x00BEBDC9
#define _CC3_SCCB_PORT_DDR_WRITE		0x00FEBDC9



//void _cc3_delay_sccb ()
//{
//	volatile int x;
//	//for (x = 0; x < 1000; x++);
//	for (x = 0; x < 25; x++);
//	
//}
//
//
///**
// * _cc3_delay_us_4()
// *
// * This function delays in intervals of 4us
// * without using the timer.
// */
//void _cc3_delay_us_4 (int cnt)
//{
//	volatile int i, x;
//	for (i = 0; i < cnt; i++)
//		for (x = 0; x < 10; x++);
//}


static void _cc3_set_cam_ddr_sccb_idle (void)
{
	GPIO_IODIR = _CC3_SCCB_PORT_DDR_IDLE;
	_cc3_delay_sccb ();
}

static void _cc3_set_cam_ddr_sccb_write (void)
{
	GPIO_IODIR = _CC3_SCCB_PORT_DDR_WRITE;
	_cc3_delay_sccb ();
}


static void _cc3_set_cam_ddr (volatile unsigned long val)
{
	//DDR(SCCB_PORT,val);
	GPIO_IODIR = val;
	_cc3_delay_sccb ();
}


static bool _cc3_sccb_send (unsigned int num, unsigned int *buffer)
{
	bool ack;
	unsigned int i, k;
	unsigned int data;
	
	// Send Start Bit
	//SCCB_SDA=0;  // needed because values can be reset by read-modify cycle
	GPIO_IOCLR = 0x00800000;
	_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);        // SDA=0 SCL=1
	//SCCB_SCL=0;  // needed because values can be reset by read-modify cycle
	GPIO_IOCLR = 0x00400000;
	_cc3_set_cam_ddr_sccb_write ();        // SDA=0 SCL=0
	
	// Send the Byte
	for (k = 0; k != num; k++) {
		data = buffer[k];           // To avoid shifting array problems
		for (i = 0; !(i & 8); i++)  // Write data
		{
			if (data & 0x80) {
				_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);  // SDA=1 SCL=0
				_cc3_set_cam_ddr_sccb_idle ();   // SDA=1 SCL=1
			}
			else {
				_cc3_set_cam_ddr_sccb_write ();  // SDA=0 SCL=0
				_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);  // SDA=0 SCL=1
				
			}
			while (!(GPIO_IOPIN & 0x00400000));
			//while(!SCCB_SCL);
			
			
			if (data & 0x08) {
				_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);  // SDA=1 SCL=0
				
			}
			else {
				_cc3_set_cam_ddr_sccb_write ();  // SDA=0 SCL=0
			}
			
			data <<= 1;
		}                           // END OF 8 BIT FOR LOOP
		
		// Check ACK  <*************************************
		_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);      // SDA=1 SCL=0
		
		_cc3_set_cam_ddr_sccb_idle ();       // SDA=1 SCL=1
		ack = false;
		
		//if(SCCB_SDA)                     // sample SDA
		if (GPIO_IOPIN & 0x00800000) {
			ack = true;
			break;
			// I think there is a bug here, it won't see a raised SDA when a bad ack occurs
		}
		
		_cc3_set_cam_ddr_sccb_write ();      // SDA=0 SCL=0
		
	}
	
	// Send Stop Bit
	_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);        // SDA=0 SCL=1
	_cc3_set_cam_ddr_sccb_idle (); // SDA=1 SCL=1
	
	return ack;
	
}

// I2C rules:
//  SDA falling whilst SCL high = start (or repeated start)
//  Data allowed to change while SCL low. (write/change data)
//  Data stable whilst SCL high. (read/keep data stable)
//  Data can be delayed by the slave by delaying clock rise (holding clock low whilst it is low)
//  SDA rising whilst SCL high = stop


static bool _cc3_sccb_recv (byte slave_address, unsigned int num_in, byte* return_data)
{
	bool ack;
	unsigned int i, k;
	byte data;
	
	// Send Start Bit
	//SCCB_SDA=0;  // needed because values can be reset by read-modify cycle
	GPIO_IOCLR = 0x00800000;
	_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);        // SDA=0 SCL=1
	//SCCB_SCL=0;  // needed because values can be reset by read-modify cycle
	GPIO_IOCLR = 0x00400000;
	_cc3_set_cam_ddr_sccb_write ();        // SDA=0 SCL=0
	
	// Send the Byte
	data = slave_address;           // To avoid shifting array problems
	for (i = 0; !(i & 8); i++)  // Write data
	{
		if (data & 0x80) {
			_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);  // SDA=1 SCL=0
			_cc3_set_cam_ddr_sccb_idle ();   // SDA=1 SCL=1
		}
		else {
			_cc3_set_cam_ddr_sccb_write ();  // SDA=0 SCL=0
			_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);  // SDA=0 SCL=1
			
		}
		while (!(GPIO_IOPIN & 0x00400000));		// wait for clock extending to stop
		//while(!SCCB_SCL);
		
		// toggle the clock low
		if (data & 0x08) {
			_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);  // SDA=1 SCL=0
			
		}
		else {
			_cc3_set_cam_ddr_sccb_write ();  // SDA=0 SCL=0
		}
		
		data <<= 1;
	}                           // END OF 8 BIT FOR LOOP
	
	// Check ACK  <*************************************
	_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);      // SDA=1 SCL=0
	
	_cc3_set_cam_ddr_sccb_idle ();       // SDA=1 SCL=1
	ack = false;
	
	//if(SCCB_SDA)                     // sample SDA
	if (GPIO_IOPIN & 0x00800000) {
		ack = true;
		// break;
		// I think there is a bug here, it won't see a raised SDA when a bad ack occurs
	}
	else
	{
		
		//_cc3_set_cam_ddr_sccb_write ();      // SDA=0 SCL=0

		// allow slave to write
		_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);      // SDA=1 SCL=0
		//
		// Read the bytes
		//
		for (k = 0; k != num_in; k++) {
			data = 0;
			for (i = 0; !(i & 8); i++)  // read data
			{
				// stop the slave altering the data
				_cc3_set_cam_ddr_sccb_idle ();       // SDA=1 SCL=1
				while (!(GPIO_IOPIN & 0x00400000));		// wait for clock extension to stop

				// data stable whilst SCL high, so read
				data <<= 1;
				if(GPIO_IOPIN & 0x00800000)
				{
					data |= 1;
				}

				// allow slave to write
				_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);      // SDA=1 SCL=0
			}                           // END OF 8 BIT FOR LOOP
			
			return_data[k] = data;           // To avoid shifting array problems
			
			// decide what ack to write...
			if(k == (num_in-1)) // no acknowledge for last byte to signal end of transfer
			{
				_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);      // SDA=1 SCL=0
				_cc3_set_cam_ddr_sccb_idle ();       // SDA=1 SCL=1
			}
			else 		// write good ACK if not last one
			{
				_cc3_set_cam_ddr_sccb_write ();      // SDA=0 SCL=0
				_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);  // SDA=0 SCL=1
			}
			
			// check clock has gone high (no suspend from slave)
			while (!(GPIO_IOPIN & 0x00400000));
			//while(!SCCB_SCL);

			// the device will read here
			
			// now we need to raise the clock to stop that ... it doesn't matter 
			// really what the data is at this point...
			// except perhaps if this is the last byte
			_cc3_set_cam_ddr_sccb_write ();      // SDA=0 SCL=0		
		}
	}
	
	// Send Stop Bit
	_cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);        // SDA=0 SCL=1
	_cc3_set_cam_ddr_sccb_idle (); // SDA=1 SCL=1
	
	return ack;
	
}

enum _cc3_camera_type_t {
    _CC3_OV6620_WRITE = 0xC0,
    _CC3_OV7620_WRITE = 0x42,
    _CC3_OV6620_READ = 0xC1,
    _CC3_OV7620_READ = 0x43
} ;


/**
 * cc3_set_raw_register():
 * This will take an address and a value from the OmniVision manual
 * and set it on the camera.  This should be used for advanced low level
 * manipulation of the camera modes.  Currently, this will not set the
 * corresponding cc3 internal data structure that keeps record of the camera
 * mode.  Use with CAUTION.
 *
 * For basic manipulation of camera parameters see other cc3_set_xxxx functions.
 */

bool cc3_camera_set_raw_register (byte address, byte value)
{
	unsigned int data[3];
	int to;
	data[0] = _CC3_OV7620_WRITE;
	data[1] = address;
	data[2] = value;
	to = 0;
	while (_cc3_sccb_send (3, data)) {
		to++;
		if (to > 3)
			return false;
	}
	_cc3_delay_us_4 (1);
	return true;
}

bool cc3_camera_select_register_write (byte address)
{
	unsigned int data[2];
	int to;
	data[0] = _CC3_OV7620_WRITE;
	data[1] = address;
	to = 0;
	while (_cc3_sccb_send (2, data)) {
		to++;
		if (to > 3)
			return false;
	}
	_cc3_delay_us_4 (1);
	return true;
}

bool cc3_camera_get_raw_register (byte address, byte* return_data)
{
	int to;	
	//byte return_data;
	to = 0;
	while(1)
	{		
		// I think we need to set what register we need to access before every read retry (as 
		// well as the first read attempt) ... otherwise things might not go as we expect, 
		// i.e. might we write over random registers?
		if(cc3_camera_select_register_write(address) == false)
		{
			return false;
		}
		if(_cc3_sccb_recv(_CC3_OV7620_READ, 1, return_data)) {
			to++;
			if (to > 3)
				return false;
		}
		else
		{
			break;
		}
	}
	_cc3_delay_us_4 (1);
	return true;
}

