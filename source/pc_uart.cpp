/*
 *  pc_uart.cpp
 *  
 *
 *  Created by Rob Probin on 09/09/2006.
 *  Copyright (C) 2006 Rob Probin.
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

#include "lpc210x_gnuarm.h"
#include "hardware_support.h"
#include "robot_hardware_defs.h"

#include "pc_uart.h"

// +---------------------------------------------------------------------------------

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

// +---------------------------------------------------------------------------------


void set_up_uart()
{
	
	pin_function_select(0,1);		// select TxD0
	pin_function_select(1,1);		// select RxD0
	
//	int baud_rate = 115200;			// original baud rate
	int baud_rate = 230400;			// improved baud rate
	
	int baud_rate_clock = 16 * baud_rate;
	int clock_divisor = pclk / baud_rate_clock;
	
	UART0_LCR = serial_divisor_access_enable;
	
	UART0_DLL = 0xff & clock_divisor;
	UART0_DLM = (0xff00 & clock_divisor) >> 8;
	
	UART0_LCR = serial_divisor_access_disable + serial_disable_break + serial_odd_parity + serial_disable_parity + serial1stop + serial8bit;
	
	UART0_FCR = 0x7;		// bit 0 = enable fifo, bit 1 = reset rx, bit 2 = reset tx, bit 6/7 = rx interrupt trigger level (0=1 char)
}


// +---------------------------------------------------------------------------------

static unsigned char bytes_transmitted = 0;

const int MAX_BYTES_IN_UART_FIFO = 16;

enum {
	receiver_data_ready_bit = (1<<0),
	transmitter_holding_register_empty_bit = (1<<5)
};


static void tx_char(const char char_out)
{
	if(bytes_transmitted >= MAX_BYTES_IN_UART_FIFO)
	{
		// FIFO might be full ... so let's wait for it to be empty
		// 
		// Actually slower than an interrupt because we have to 
		// wait for it to fully empty ... however we should 
		// achieve close to the maximum bandwidth, except
		// for the last few characters before a wait.
		while(!(UART0_LSR & transmitter_holding_register_empty_bit))
		{
			// wait for it to empty
		}
		
		bytes_transmitted = 0;		// we know the FIFO is empty
	}
	
	UART0_THR = char_out;
	bytes_transmitted ++;
}

//
CharBuffer::CharBuffer()
: index(0)
{
	for(int i=0; i<max_length; i++)
	{
		buffer[i]=0;
	}
}
void CharBuffer::write(const char char_out)
{
	buffer[index] = char_out;
	index++;
	if(index==max_length) { index=0; }
}
void CharBuffer::replay()
{
	// write oldest stuff first
	for(int i=index; i<max_length; i++)
	{
		char c=buffer[i];
		if(c) { tx_char(c); }
	}
	// then write new stuff
	for(int i=0; i<index; i++)
	{
		char c=buffer[i];
		if(c) { tx_char(c); }
	}
}

// make a buffer
#define BUFFER_WRITE_CHARACTERS
#ifdef BUFFER_WRITE_CHARACTERS
static CharBuffer* buffer;
#endif

void set_replay_buffer(CharBuffer* write_buffer)
{
#ifdef BUFFER_WRITE_CHARACTERS
	buffer = write_buffer;
#endif
}

void write_char(const char char_out)
{
#ifdef BUFFER_WRITE_CHARACTERS
	if(buffer) { buffer->write(char_out); }
#endif
	tx_char(char_out);
}

void replay_write()
{
#ifdef BUFFER_WRITE_CHARACTERS
	if(buffer) { buffer->replay(); }
#endif
}

// +---------------------------------------------------------------------------------

bool key_ready()
{
	return UART0_LSR & receiver_data_ready_bit;
}

// +---------------------------------------------------------------------------------

int read_char()
{
	while(!(UART0_LSR & receiver_data_ready_bit))
	{
		// wait for a byte to arrive
	}
	
	return UART0_RBR;
}

// +---------------------------------------------------------------------------------

void write_string(const char *string)
{
	char c;
	
	// for the moment write only 16 bytes
	while(1)
	{
		if(!(c = *string)) break;
		
		write_char(c);
		string++;
	}
}

// +---------------------------------------------------------------------------------
void write_data(const unsigned char* data, unsigned int number_of_bytes)
{
	while(number_of_bytes)
	{
		write_char(*data);
		data++;
		number_of_bytes--;
	}
}

// +---------------------------------------------------------------------------------

void write_cr()
{
#define ZTERM 0
#if ZTERM
	write_char(13);		// cr ... for mac programs
#else
	write_char(10);		// lf ... for unix programs
#endif
}

// +---------------------------------------------------------------------------------

void write_line(const char *string)
{
	write_string(string);
	write_cr();
}

// +---------------------------------------------------------------------------------

void write_line(const char *string, int value)
{
	write_string(string);
	write_int(value);
	write_cr();
}

// +---------------------------------------------------------------------------------

void write_string(const char *string, int value)
{
	write_string(string);
	write_int(value);
}

// +---------------------------------------------------------------------------------

void write_int(int value)
{
	char intstring[30];
	int i=0;
	
	if(value<0) 
	{ 
		intstring[i++] = '-'; 
		value = -value; 
	}
	
	if(value==0)
	{
		intstring[i++] = '0';
		intstring[i] = 0;
	}
	else
	{
		int position = i;
		while(value)
		{
			intstring[i++] = (value%10)+'0';
			value=value/10;
		}
		intstring[i] = 0;		
		
		// now reverse the digits
		char temp;
		int reverse_length = (i-position)/2;
		for(int x=0; x < reverse_length; x++)
		{
			i--;
			temp = intstring[i];
			intstring[i] = intstring[position];
			intstring[position] = temp;
			position++;
		}
	}
	
	write_string(intstring);
}

// +---------------------------------------------------------------------------------
void write64_16(int16_t value)
{
	// 1 = 6 bits = 4 bits data + 2  bits size 
	// 2 = 12 bits = 10 bits + 2 bits size
	// 3 = 18 bits = 16 bits + 2 bits size
	// 4 = 24
	// 5 = 30
	// 6 = 36 = 32 bits of data + 2 bits size + 2 space
	uint16_t v = static_cast<uint16_t>(value);
	
	char s3 = '0' + (v & 0x3f);		// char '0' = 30 hex = 48
	v = v >> 6;
	uint8_t s2 = '0' + (v & 0x3f);  // 48+64 = 112
	v = v >> 6;
	uint8_t s1 = '0' + (v & 0x3f);
	
	write_char(s1);
	write_char(s2);
	write_char(s3);
}
// +---------------------------------------------------------------------------------

void write_uint(unsigned int value)
{
	char intstring[30];
	int i=0;
	
	if(value==0)
	{
		intstring[i++] = '0';
		intstring[i] = 0;
	}
	else
	{
		int position = i;
		while(value)
		{
			intstring[i++] = (value%10)+'0';
			value=value/10;
		}
		intstring[i] = 0;		
	
		// now reverse the digits
		char temp;
		int reverse_length = (i-position)/2;
		for(int x=0; x < reverse_length; x++)
		{
			i--;
			temp = intstring[i];
			intstring[i] = intstring[position];
			intstring[position] = temp;
			position++;
		}
	}
	
	write_string(intstring);
}

// +---------------------------------------------------------------------------------

void write_hex(unsigned int value)
{
	char intstring[30];
	int i=0;

	if(value==0)
	{
		intstring[i++] = '0';
		intstring[i] = 0;
	}
	else
	{
		int position = i;
		while(value)
		{
			int digit = value%16;
			if(digit<=9)
			{
				intstring[i++] = digit+'0';
			}
			else
			{
				intstring[i++] = digit - 10 + 'a';
			}
			value=value/16;
		}
		intstring[i] = 0;		
		
		// now reverse the digits
		char temp;
		int reverse_length = (i-position)/2;
		for(int x=0; x < reverse_length; x++)
		{
			i--;
			temp = intstring[i];
			intstring[i] = intstring[position];
			intstring[position] = temp;
			position++;
		}
	}
	
	write_string(intstring);
}


//
//
//
void write_bool(bool b)
{
	if(b)
	{
		write_string("1");
	}
	else
	{
		write_string("0");
	}
}

// +---------------------------------------------------------------------------------

//void interrupt recieve_chars()
//{
//}
	
// +---------------------------------------------------------------------------------

//void interrupt transmit_empty()
//{
//}
	
// +---------------------------------------------------------------------------------
