/*
 *  eeprom.h
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

#ifndef EEPROM_H
#define EEPROM_H

#include "i2c_driver.h"


class eeprom  {
	
public:
	unsigned int read(unsigned short address);
	void write(unsigned short address, unsigned int data32);

//	static eeprom* instance();
protected:
//	eeprom();
//	eeprom(const eeprom&);
//	eeprom& operator= (const eeprom&);

private:
	i2c_driver i2c;
	
	void wait_for_end();
	void checked_send_data(unsigned char* data, int bytes);
};


#endif // EEPROM_H
