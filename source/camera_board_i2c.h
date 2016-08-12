/*
 *  cc3_camera_i2c.h
 *  robot_core
 *
 *  Created by Rob Probin on 10/01/2008.
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

#ifndef CC3_CAMERA_I2C_H
#define CC3_CAMERA_I2C_H

#include "robot_basic_types.h"
#include "language_support.h"

// return true for ok, false for fail
bool cc3_camera_get_raw_register (byte address, byte* return_data);
bool cc3_camera_set_raw_register (byte address, byte value);


#endif // CC3_CAMERA_I2C_H
