/*
 *  logger.h
 *  robot_core
 *
 *  Created by Rob Probin on 19/11/2009.
 *  Copyright (C) 2009 Rob Probin.
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

#ifndef LOGGER_H
#define LOGGER_H

//typedef unsigned short int Logger_data_t;
typedef int Logger_data_t;

class Logger {
public:
	Logger();
	void log(Logger_data_t value);
	void print();
private:
	static const int size = 15500;		// was 31000 for short
	Logger_data_t data[size];
	Logger_data_t* dest;
	int length;
};


#endif

