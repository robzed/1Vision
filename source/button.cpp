/*
 *  button.cpp
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

#include "button.h"

Button::Button(byte port_in)
: button_port(port_in, false)
{
}

bool Button::pressed()
{
	return !button_port.get();
}

//bool Button::held()
//{
//	return false;
//}
