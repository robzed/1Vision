/*
 *  pc_uart.h
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

#ifndef PC_UART_H
#define PC_UART_H

#include <stdint.h>

void set_up_uart();
void write_string(const char *string);
void write_cr();
void write_line(const char *string);
void write_int(int value);
void write_hex(unsigned int value);
void write_bool(bool b);
void write_data(const unsigned char* data, unsigned int number_of_bytes);
void write_uint(unsigned int value);

// more complex ones ... this happens all the time
void write_line(const char *string, int value);
void write_string(const char *string, int value);


// basic ones based on write64
void write64_16(int16_t value);


//
// basic i/o
//
int read_char();
void write_char(const char c);
bool key_ready();

// can we use << operators?


// replay features
class CharBuffer {
	static const int max_length = 200;
	char buffer[max_length];
	short index;
public:
	void write(const char char_out);
	void replay();
	CharBuffer();
};

void replay_write();
void set_replay_buffer(CharBuffer* write_buffer);



#endif // PC_UART_H
