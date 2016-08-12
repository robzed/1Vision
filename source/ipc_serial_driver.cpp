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

#include "ipc_serial_driver.h"
#include "robot_hardware_defs.h"
#include "hardware_support.h"

//#define DEBUGGING_ENABLED 0
//include "debug_support.h"
//#include "pc_uart.h"

enum { 
	serial5bit = 0,
	serial6bit = 1,
	serial7bit = 2,
	serial8bit = 3
};

enum { 
	serial1stop = 0,
	serial2stops = 1<<2
};

enum {
	serial_disable_parity = 0,
	serial_enable_parity = 1<<3
};

enum {
	serial_odd_parity = 0,
	serial_even_parity = 1 << 4,
	serial_force_1_parity = 2 << 4,
	serial_force_0_parity = 3 << 4
};

enum {
	serial_disable_break = 0,
	serial_enable_break = 1<<6
};

enum {
	serial_divisor_access_disable = 0,
	serial_divisor_access_enable = 1 << 7
};


ipc_serial_driver::ipc_serial_driver()
: bytes_transmitted(0)
{
	pin_function_select(MAIN_ipc_tx_port,1);		// select TxD0
	pin_function_select(MAIN_ipc_rx_port,1);		// select RxD0
	
	int baud_rate = IPC_BAUD_RATE;		// transfers are nice and fast

	int  baud_rate_clock = 16 * baud_rate;
	int  clock_divisor = pclk / baud_rate_clock;
	
	UART1_LCR = serial_divisor_access_enable;
	
	UART1_DLL = 0xff & clock_divisor;
	UART1_DLM = (0xff00 & clock_divisor) >> 8;
	
	UART1_LCR = serial_divisor_access_disable + serial_disable_break + serial_odd_parity + serial_disable_parity + serial1stop + serial8bit;
	
	UART1_FCR = 0x7;		// bit 0 = enable fifo, bit 1 = reset rx, bit 2 = reset tx, bit 6/7 = rx interrupt trigger level (0=1 char)	
}


//void ipc_serial_driver::print_registers()
//{
//	write_hex(UART1_IER); write_cr();
//	write_hex(UART1_IIR); write_cr();
//	write_line("FCR=WO");
//	write_hex(UART1_LCR); write_cr();
//	write_hex(UART1_MCR); write_cr();
//	write_hex(UART1_LSR); write_cr();
//	write_hex(UART1_MSR); write_cr();
//	UART1_LCR = serial_divisor_access_enable;
//	write_int(UART1_DLL); write_cr();
//	write_int(UART1_DLM); write_cr();
//	UART1_LCR = serial_divisor_access_disable + serial_disable_break + serial_odd_parity + serial_disable_parity + serial1stop + serial8bit;
//
//}

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
		while(!(UART1_LSR & transmitter_holding_register_empty_bit))
		{
			// wait for it to empty
		}
		
		bytes_transmitted = 0;		// we know the FIFO is empty
	}
	
	UART1_THR = data;
	bytes_transmitted ++;

	//dwrite_string("<"); dwrite_int(data); dwrite_string(">");
}

// +---------------------------------------------------------------------------------

bool ipc_serial_driver::byte_incoming_ready()
{
	return UART1_LSR & receiver_data_ready_bit;
}

// +---------------------------------------------------------------------------------

byte ipc_serial_driver::read_byte()
{
	while(!(UART1_LSR & receiver_data_ready_bit))
	{
		// wait for a byte to arrive
	}
	
	byte data = UART1_RBR;
	//dwrite_string("["); dwrite_int(data); dwrite_string("]");
	return data;
}
