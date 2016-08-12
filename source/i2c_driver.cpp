/*
 *  i2c_driver.cpp
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

#include "i2c_driver.h"
#include "pc_uart.h"
#include "lpc210x_gnuarm.h"
#include "hardware_support.h"

#define DEBUGGING_ENABLED 0			// turn this on for read/write debugging
#include "debug_support.h"

//
// i2c control register bit definitions
//
const int AA = (1<<2);		// assert acknowledge
const int SI = (1<<3);		// serial interrupt
const int STO = (1<<4);		// stop
const int STA = (1<<5);		// start
const int I2EN = (1<<6);    // i2c interface enable

i2c_driver::i2c_driver()
: 
error_flag(false),
device_busy_flag(false),
protocol_level(0),
update_funcptr(&i2c_driver::service_inactive),	// do nothing to start
protocol_state(0)
{
	// enable port usage
	pin_function_select(2,1);
	pin_function_select(3,1);
	
	// set up i2c clock
	I2C_I2SCLH = 100;	// H+L = 200, 60M/200 = 300 kHz
	I2C_I2SCLL = 100;
	
	// enable i2c driver
	I2C_I2CONCLR = AA + SI + STA; // ensure other bits are clear
	I2C_I2CONSET = I2EN;
	
	
	// addresses for EEPROM
	i2c_write_address = 0xA0;
	i2c_read_address = 0xA1;
	
}
//const int camera_write_address = 0x42;
//const int camera_read_address = 0x43;


i2c_driver::~i2c_driver()
{
	// disable i2c driver
	I2C_I2CONCLR = I2EN;
	
	// disable port usage
	pin_function_select(2,0);
	pin_function_select(3,0);	
}

void i2c_driver::send_data(unsigned char* data, int bytes)
{
	dwrite_line("send_data request");

	if(bytes > 0)
	{
		data_ptr = data;
		byte_count = bytes;
		error_flag = false;
		device_busy_flag = false;
		call_FSM(&i2c_driver::service_write);
	}
}

void i2c_driver::rx_data(unsigned char* data, int bytes)
{
	dwrite_line("rx data request");

	if(bytes > 0)
	{
		data_ptr = data;
		byte_count = bytes;
		error_flag = false;
		device_busy_flag = false;
		call_FSM(&i2c_driver::service_read);
	}
}


void i2c_driver::print_reg_status()
{
	int control = I2C_I2CONSET;
	write_string("AA="); write_bool(control & AA);
	write_string(" SI="); write_bool(control & SI);
	write_string(" STO="); write_bool(control & STO);
	write_string(" STA="); write_bool(control & STA);
	write_string(" I2EN="); write_bool(control & I2EN);

	write_string(" STATUS = ");
	write_hex(I2C_I2STAT); 
	write_cr();
}

void i2c_driver::set_control_reg(bool AA_in, bool SI_in, bool STO_in, bool STA_in)
{
	int set_control = 0;
	int clr_control = 0;
	if(AA_in) set_control |= AA; else clr_control |= AA;
	if(SI_in) set_control |= SI; else clr_control |= SI;
	if(STO_in) set_control |= STO;	// stop is auto clearing
	if(STA_in) set_control |= STA; else clr_control |= STA;
	set_control |= I2EN;	// always set due to constructor

	// we want to set bits that weren't set before
	int control = I2C_I2CONSET;

	set_control = set_control & ~control; // clear out bits that are not set, i.e. only keep bits to set that are currently clear
	clr_control = clr_control & control; // clear any bits that are clear already, i.e. control needs to be set and clear bit needs to be set

	write_hex(set_control); write_string(" clr="); write_hex(clr_control); write_cr();

	if(set_control) I2C_I2CONSET = set_control;
	if(clr_control) I2C_I2CONCLR = clr_control;
}

void i2c_driver::set_data_reg(byte data)
{
	I2C_I2DAT = data;
}

byte i2c_driver::get_data_reg()
{
	return I2C_I2DAT;
}

bool i2c_driver::get_serial_interrupt_flag()
{
	return I2C_I2CONSET & SI;
}

// Cooperative Threading
// Options:
// (a) Switch statement: monolythic (for the most part), not really overly OO
// (b) Member function pointers: must have same return type and arguments
//
// google for c++ member function pointer
// http://www.google.com/search?hl=en&q=c%2B%2B+member+function+pointer&btnG=Google+Search
// 
// Good Hits:
// http://www.codeproject.com/cpp/FastDelegate.asp
// http://www.parashift.com/c++-faq-lite/pointers-to-members.html#faq-33.1
//


//state_table_t state_table[] = 
//{
//	{ write_start, 0 },
//	{ wait_for_SI, 0 },
//	{ check_for_status, 8 },
//	{ write_data, 
//	{ clear_start, 0 },
//	{ 
//}
// state param
// check for status = f8


void i2c_driver::service_i2c()
{
	(this->*update_funcptr)();		// redirect to service routine
}

bool i2c_driver::completed()
{
	return (update_funcptr == &i2c_driver::service_inactive);
}

bool i2c_driver::error()
{
	return error_flag;
}

bool i2c_driver::device_busy()
{
	return device_busy_flag;
}

void i2c_driver::service_inactive()
{
	// nothing to do here
}

//void i2c_driver::state_write_start()
//{
//	call(STATE_check_ok);
//	call(STATE_start_i2c);
//	call(STATE_address_read);
//	call(STATE_write_data);
//	call(STATE_stop);
//}

//void i2c_driver::state_read_start()
//{
//	call(STATE_check_ok);
//	call(STATE_start_i2c);
//	call(STATE_address_read);
//	call(STATE_read_data);
//	`call(STATE_stop);
//}

	


void i2c_driver::service_read()
{
	
	switch(protocol_state)
	{
		case 0:		// check for status = f8
			if(I2C_I2STAT != 0xf8)
			{
				dwrite_line("<not f8 - stopping>");
				insert_stop();
			}
			else	// if it is OK
			{
				dwrite_line("<status f8>");
				protocol_state++;
			}
			break;
			
		case 1:		// start the i2c
			dwrite_line("<wait for start>");
			I2C_I2CONSET = STA;
			protocol_state++;
			insert_wait_for_SI(0x08);	// check status = 0x08
			break;
					
		case 2:		// now write address out
			dwrite_line("<address + r>");
			I2C_I2DAT = i2c_read_address;
			I2C_I2CONCLR = STA + SI;
			protocol_state++;
			device_busy_flag = true;
			insert_wait_for_SI(0x40);	// check status = 0x40
			//call_FSM(&i2c_driver::service_wait_for_slave_address_tx_finish);	// check status for slave address transmit
			break;
			
		case 3:		// read the data
			device_busy_flag = false;
			dwrite_line("<reading>");
			if(byte_count > 1)
			{
				dwrite_line("<first is last>");
				I2C_I2CONSET = AA;			// answer the byte if not the last one
			}
			I2C_I2CONCLR = SI;			// clear the serial interrupt
			protocol_state++;
			insert_wait_for_read_byte();	// check status = 0x50 or 0x58
			break;
			
		case 4:
			*data_ptr = I2C_I2DAT;
			dwrite_string("<read "); dwrite_hex(*data_ptr); dwrite_line(" >");
			//print_reg_status();
			
			data_ptr++; byte_count--;
			if(byte_count==0)
			{
				dwrite_line("<stop>");
				// write the stop
				protocol_state++; 
				insert_stop();
			}
			else		// get next byte
			{
				dwrite_line("<read next>");
				if(byte_count == 1)
				{
					dwrite_line("<last>");
					I2C_I2CONCLR = AA;			// answer the byte if not the last one
				}
				I2C_I2CONCLR = SI;			// clear the serial interrupt
				insert_wait_for_read_byte();	// wait for next byte
			}
			break;
			
		case 5:
		default:			// no idea what we do in default case...
			dwrite_line("<finished read>");
			finish_FSM();
			break;
	}


}

void i2c_driver::service_write()
{

	switch(protocol_state)
	{
		case 0:		// check for status = f8
			if(I2C_I2STAT != 0xf8)
			{
				dwrite_line("<not f8 - stopping>");
				insert_stop();
			}
			else	// if it is OK
			{
				dwrite_line("<status f8>");
				protocol_state++;
			}
			break;
			
		case 1:		// start the i2c
			dwrite_line("<wait for start>");
			I2C_I2CONCLR = AA;			// ensure not slave mode
			I2C_I2CONSET = STA;
			protocol_state++;
			insert_wait_for_SI(0x08);	// check status = 0x08
			break;
			
		case 2:		// now write address out
			dwrite_line("<address + w>");
			I2C_I2DAT = i2c_write_address;
			I2C_I2CONCLR = STA + SI;
			protocol_state++;
			device_busy_flag = true;
			insert_wait_for_SI(0x18);	// check status = 0x18
			//call_FSM(&i2c_driver::service_wait_for_slave_address_tx_finish);	// check status for slave address transmit
			break;
			
		case 3:		// write the data out
			device_busy_flag = false;
			dwrite_string("<writing "); dwrite_hex(*data_ptr); dwrite_line(" >");
			I2C_I2DAT = *data_ptr;
			data_ptr++; byte_count--;
			I2C_I2CONCLR = SI;
			if(byte_count==0) 
			{
				protocol_state++; 
			}
			insert_wait_for_SI(0x28);	// check status = 0x28  (... could be a 30 for a not ack byte...)
			break;
			
		case 4:		// write the stop
			dwrite_line("<stop>");
			protocol_state++;
			insert_stop();
			break;
						
		case 5:
			dwrite_line("<finished write>");
			finish_FSM();
			break;

		default:			// no idea what we do in default case...
			dwrite_line("WEIRD VALUE");
			finish_FSM();
			break;
	}	
}


void i2c_driver::insert_stop()
{
	dwrite_line("insert stop request");

	I2C_I2CONSET = STO;			// create a stop
	I2C_I2CONCLR = AA + STA + SI;	// clear auto-answer and ensure not start
	call_FSM(&i2c_driver::service_stop);
}


void i2c_driver::insert_wait_for_SI(int status_required)
{
	dwrite_string("insert wait for SI request = "); dwrite_hex(status_required); dwrite_cr();

	wait_for_status_code = status_required;
	call_FSM(&i2c_driver::service_wait_for_SI);
}

void i2c_driver::insert_wait_for_read_byte()
{
	dwrite_line("insert read byte request");

	call_FSM(&i2c_driver::service_wait_for_read_byte);
}

void i2c_driver::service_stop()
{	
#warning "need timing if stop doesn't work"
	if( (I2C_I2CONSET & STO) == 0)
	{
		// stop has cleared ... so we are ok to quit?
		if(I2C_I2STAT == 0xf8)
		{
			dwrite_line("finished stop");
			finish_FSM();
		}
		else	// not sure what to do if we fail this test?
		{
			dwrite_line("abort from stop");
			abort_FSM(true);
		}
	}

#if 0
	//
	// Now write stop
	//
	write_line("STOPPING I2C");
	i2c_test_obj->set_control_reg(0, 0, 1, 0);	// AA SI STO STA
	i2c_test_obj->print_reg_status();
	// check for stop then status = f8
#endif
}


void i2c_driver::service_wait_for_SI()
{
#warning "need timing if SI doesn't work"
	if(I2C_I2CONSET & SI)
	{
		if(I2C_I2STAT == wait_for_status_code)		// byte and there is another byte afterwards
		{
			dwrite_string("finished wait for SI - got correct value ("); dwrite_hex(wait_for_status_code); dwrite_line(")");
			finish_FSM();
		}
		else
		{
			dwrite_string("abort from wait for SI - value ="); dwrite_hex(I2C_I2STAT); dwrite_cr();
			abort_FSM(true);
		}
	}	
}


//void i2c_driver::service_wait_for_slave_address_tx_finish()
//{
//#warning "need timing if SI doesn't work"
//	if(I2C_I2CONSET & SI)
//	{
//		// see '8XC552_562OVERVIEW_2.pdf' table 4 and 5.
//		unsigned int status = I2C_I2STAT;
//		if(status == 0x18 || status == 0x40) // SLA+W has been transmitted; ACK has been received
//		{
//			finish_FSM();
//		}
//		else if(status == 0x20 || status == 0x48) // SLA+W has been transmitted; NOT ACK has been received
//		{
//			no_ack_from_slave_address_flag = true;
//			abort_FSM(true);
//		}
//		else
//		{
//			dwrite_string("abort from wait for SI - value ="); dwrite_hex(I2C_I2STAT); dwrite_cr();
//			abort_FSM(true);
//		}
//	}	
//}


void i2c_driver::service_wait_for_read_byte()
{
#warning "need timing if SI doesn't work"
		if(I2C_I2CONSET & SI)
		{
			if(I2C_I2STAT == 0x50)		// byte and there is another byte afterwards
			{
				dwrite_line("finished read byte 50 (more)");
				finish_FSM();
			}
			else if(I2C_I2STAT == 0x58)		// byte and there is not any more data
			{
				dwrite_line("finished read byte 58 (no more)");
				finish_FSM();
			}
			else
			{
				dwrite_line("abort from read byte");
				abort_FSM(true);
			}
		}
}

#if 0
void i2c_driver::state_check_i2stat()
{
	if(I2C_I2STAT != 0xf8)
	{
		// clear faults by stopping
		I2C_I2CONSET = STO;
		next_state = &i2c_driver::check_state;
		update_funcptr = &i2c_driver::state_check_for_cleared_stop;
	}
	// wait for stop
}


void i2c_driver::state_check_for_cleared_stop()
{
	if(!(I2C_I2CONSET&STO))
	{
		// stop has cleared
		update_funcptr = next_state;
	}
	
	if(timer.expired())
	{
		abort_FSM(true);
	}
}
#endif


void i2c_driver::print_FSM_status()
{
#if 0
	dwrite_string("Protocol level=");
	dwrite_int(protocol_level);
	
	dwrite_string(" Protocol state=");
	dwrite_int(protocol_state);

	dwrite_string(" error=");
	dwrite_int(error_flag);
	dwrite_string(" busy=");
	dwrite_int(device_busy_flag);
	dwrite_cr();	
#endif
}

void i2c_driver::call_FSM(funcptr_t state)
{
	//dwrite_string("call_FSM          "); print_reg_status();
	print_FSM_status();
	
	if(protocol_level <= 1)		// only one deep push
	{
		if(protocol_level != 0)		// special case at top level
		{
		pushed_protocol_state = protocol_state;
		pushed_update_funcptr = update_funcptr;
		}
		
		protocol_level++;
		
		//
		// prepare for a call to the next state
		//
		protocol_state = 0;
		update_funcptr = state;
		
		service_i2c();		// make the initial call straight away
	}
	else
	{
		dwrite_line("Call level error");
		abort_FSM(true);
	}

	//dwrite_string("end call_FSM          "); print_reg_status();
	//print_FSM_status();
}

void i2c_driver::finish_FSM()
{
	//dwrite_string("finish_FSM          "); print_reg_status();
	//print_FSM_status();
	
	if(protocol_level == 2)
	{
		protocol_state = pushed_protocol_state;
		update_funcptr = pushed_update_funcptr;
		protocol_level--;
	}
	else
	{
		if(protocol_level != 1)		// at protocol level == 1 we are returning to inactive
		{
			dwrite_line("Protocol level error");
			abort_FSM(true);
		}
		else
		{
			abort_FSM(false);
		}
	}

	//dwrite_line("end finish_FSM");
	//print_FSM_status();
}


void i2c_driver::abort_FSM(bool in_error)
{
#if DEBUGGING_ENABLED
	dwrite_string("abort_FSM - ");
	if(in_error)
	{
		dwrite_line("*** ERROR ***");
	}
	else
	{
		dwrite_line("no error");
	}
	//print_FSM_status();
#endif
	
	if(in_error) error_flag = true;		// never clear error flag
	protocol_level = 0;
	protocol_state = 0;

	// if we are in error then try to recover with a stop
	// ... except if we just aborted from a stop!
	if(in_error && update_funcptr != &i2c_driver::service_stop)
	{
		insert_stop();
	}
	else	// not in error, or we are in error but from a stop (so go inactive)
	{
		update_funcptr = &i2c_driver::service_inactive;
	}
	
	//dwrite_line(" end abort_FSM");
	//print_FSM_status();
}


// for writing
// ===========
//
// clear AA, STA, STO, SI
// write STA
// wait for SI and 08h
// write address and write bit
// clear SI
// write bytes

// for reading
// ===========
//
// clear AA, STA, STO, SI
// write address and write bit
// write bytes

