/*
 *  xmodem_send.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 12/05/2007.
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
 *
 */


#include "xmodem_send.h"
#include "pc_uart.h"
#include "timing.h"
#include "xmodem_crc.h"

#define ENFORCE_128_BYTE_TRANSFERS_ONLY 1


enum {
	NUL,		// 0
	SOH = 1,	// 1 ctrl-a
	STX,		// 2 ctrl-b
	ETX,		// 3 ctrl-c
	EOT,		// 4 ctrl-d
	ENQ,		// 5 ctrl-e
	ACK = 6,	// 6 ctrl-f
	BEL,		// 7 ctrl-g
	BS,			// 8 ctrl-h
	HT,			// 9 ctrl-i
	LF,			// 10, 0x0a ctrl-j
	VT,			// 11 ctrl-k
	FF,			// 12 ctrl-l
	CR,			// 13, 0x0d ctrl-m
	SO,			// 14 ctrl-n
	SI,			// 15 ctrl-o
	DLR,		// 16 ctrl-p
	DC1,		// 17 ctrl-q
	DC2,		// 18 ctrl-r
	DC3,		// 19 ctrl-s 
	DC4,		// 20 ctrl-t
	NAK = 21,	// 21 ctrl-u
	SYN,		// 22 ctrl-v
	ETB,		// 23 ctrl-w
	CAN = 24,	// 24 ctrl-x
	EM,			// 25 ctrl-y
	SUB,		// 26 ctrl-z
	ESC,		// 27, 0x1b ctrl-[
	FS,			// 28 ctrl-\              (ignore me - avoids backslash at end of line!)
	GS,			// 29 ctrl-]
	RS,			// 30 ctrl-^
	US,			// 31 ctrl-_
	DEL = 127,
};

const unsigned int xmodem_min_block_size = 128;			// needs to be a power of 2
const unsigned int xmodem_large_block_size = 1024;			// needs to be a power of 2
const int cancel_count_number = 2;

#if !ENFORCE_128_BYTE_TRANSFERS_ONLY
#warning "padding code for non-128 byte transfers not done yet"
#endif

//
//
//
bool XModem_Send::send_data(byte* data_ptr, unsigned int data_length)
{
	if(data_length & (xmodem_min_block_size-1)) { return false; }		// data must be %128=0
	if(data_length == 0) { return false; }
	
	if(!open)
	{
		open = true;
		block_num = 1;
		//
		// First we wait for the transmission to start
		//
		Timer t(110,0);			// wait for 110s for a start message
		int cancel_count = 0;
		while(1)
		{
			// we might have data ready
			if(key_ready())
			{
				char c = read_char();
				if(c==NAK)		// NAK causes the data to be sent
				{
					// Checksum requested
					crc = false;
					break;
				}
				if(c=='C')
				{
					// CRC requested
					crc = true;
					break;
				}
				if(c==CAN)		// receiver selected cancel the transfer?
				{
					cancel_count++;
					if(cancel_count == cancel_count_number)
					{
						open = false;
						return false;
					}
				}
				else	// all other characters reset this count
				{
					cancel_count = 0;
				}
			}

			// or the timer might be up
			if(t.expired())
			{
				do_cancel();
				// tell the sender something is wrong
				return false;
			}
		}
	}

	// now we send each block
	return internal_send_data(data_ptr, data_length);
}

//
//
//
bool XModem_Send::internal_send_data(byte* data_ptr, unsigned int data_length)
{
	if(!open) { return false; }
	if(data_length & (xmodem_min_block_size-1)) { do_cancel(); return false; }		// data must be %128=0
	if(data_length == 0) { do_cancel(); return false; }
	
	unsigned int xmodem_block_size;
	while(data_length)
	{
		if(data_length >= xmodem_large_block_size)
		{
			xmodem_block_size = xmodem_large_block_size;
		}
		else
		{
			xmodem_block_size = xmodem_min_block_size;
		}
		
		unsigned short check = 0;
		if(crc)
		{
			check = xmodem_crc(data_ptr, xmodem_block_size);
		}
		else // simple additive checksum
		{
			// create checksum for this block
			byte* p = data_ptr;
			for(unsigned int i=0; i < xmodem_block_size; i++)
			{
				check += *p;
				p++;
			}
		}
		
		// send the block
		// block <SOH> <blockno> <~blockno> <data-128-bytes> <checksum>
		// block <STX> <blockno> <~blockno> <data-1024-bytes> <checksum>
		if(xmodem_block_size == xmodem_large_block_size)
		{
			write_char(STX);
		}
		else
		{
			write_char(SOH);
		}
		write_char(block_num);
		write_char(~block_num);
		write_data(data_ptr, xmodem_block_size);
		if(crc)
		{
			write_char(check>>8);				// first CRC byte
		}
		write_char(check & 0xff);			// checksum or second CRC byte
		
		// wait for reply
		Timer t(60,0);			// wait for 60s for a reply
		int cancel_count = 0;
		while(1)
		{
			// we might have data ready
			if(key_ready())
			{
				char c = read_char();
				if(c==NAK)		// resend this block
				{
					break;
				}
				if(c==ACK)		// send next block
				{
					// next block
					block_num ++;
					data_length -= xmodem_block_size;
					data_ptr += xmodem_block_size;
					break;
				}
				if(c==CAN)		// receiver selected cancel the transfer?
				{
					cancel_count++;
					if(cancel_count == cancel_count_number)
					{
						open = false;
						return false;
					}
				}
				else	// all other characters reset this count
				{
					cancel_count = 0;
				}
			}
			
			// or the timer might be up
			if(t.expired())
			{
				do_cancel();
				// tell the sender something is wrong
				return false;
			}
		} // while waiting reply		
		
	} // while data_length

	//
	// we get here when we run out of data for the current transfer run...
	//
	return true;
}


//
//
//
bool XModem_Send::finish_transfer()
{
	if(!open) { return false; }
	
	while(1)
	{
		write_char(EOT);
		
		// wait for reply
		Timer t(60,0);			// wait for 60s for a reply
		int cancel_count = 0;
		while(1)
		{
			// we might have data ready
			if(key_ready())
			{
				char c = read_char();
				if(c==ACK)		// finished!
				{
					open = false;
					return true;
				}
				
				if(c==CAN)		// receiver selected cancel the transfer?
				{
					cancel_count++;
					if(cancel_count == cancel_count_number)
					{
						open = false;
						return false;
					}
				}
				else	// all other characters reset this count
				{
					cancel_count = 0;
					break;				// and cause the EOT to be transmitted
				}
			}
			
			// or the timer might be up
			if(t.expired())
			{
				do_cancel();
				// tell the sender something is wrong
				return false;
			}
		} // while waiting reply		
		
	}
	
}


//
//
//
XModem_Send::XModem_Send()
:
open(false),
block_num(1),
crc(false)
{
}

//
//
//
XModem_Send::~XModem_Send()
{
	if(open)
	{
		// finish the transfer
		finish_transfer();
	}

}


void XModem_Send::do_cancel()
{
	// eight cancels (ctrl-X) to cancel the transmission
	for(int i=0; i<8; i++)
	{
		write_char(CAN);
	}
	// write backspace to delete the characters from the input bufffer
	//for(int i=0; i<8; i++)
	//{
	//	write_char(BS);
	//}
	
	open = false;
}


void XModem_Send::cancel_transfer()
{
	if(open)
	{
		do_cancel();
	}
}

