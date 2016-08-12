/*
 *  basic_motor_primitives.h
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
#include "robot_hardware_defs.h"


void init_motor_direction_pins();

//
// Functions select by motor parameter
//
//void motor_full_forward(int motor);
void motor_remove_power(int motor);

void motor_full(int motor, bool forward);



//
// Used by current limit
//
void restore_motor1();
void restore_motor2();
void override_and_turn_motor1_off();
void override_and_turn_motor2_off();

