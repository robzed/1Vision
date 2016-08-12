/*
 *  motor_pwm.h
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

const int max_pwm = 1000;

// use either the joint PWM commands or the single commands - not both
//
void enable_pwm(int motor_1_pwm_setting, int motor_2_pwm_setting);
void disable_pwm();
void adjust_pwm(int motor_1_pwm_setting, int motor_2_pwm_setting);

// motor specific PWM
void set_single_pwm(int motor, int motor_pwm_setting);
void disable_single_pwm(int motor);
