// xmodem-crc implementation
//
// It's a CRC-CCITT implementation except:
//	(a) with a start value of 0 (I think)
//  (b) require to shift in two bytes after calculation (much argument about whether CCITT specifies this
//  but certainly xmodem requires it!
//
// This file can be used without restriction but no guarantees are made as to fitness for purpose.
// Converted 16-5-2007 by Rob Probin
// Copyright (C) 2007 Rob Probin.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
#ifndef xmodem_crc_h
#define xmodem_crc_h



unsigned short xmodem_crc(unsigned char const *message, int number_bytes);

//
// Do we want to test the routines?
//
#define CRC_TEST_MAIN 0

#if CRC_TEST_MAIN
void crc_test_main(void);
#endif



#endif // xmodem_crc_h

