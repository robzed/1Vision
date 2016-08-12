// sine_cosine.h
// Created by Rob Probin 17th March 2009
/*
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
#ifndef SINE_COSINE_H
#define SINE_COSINE_H

const int trig_scale_factor = 1000;
const int trig_number_of_entries = 2048;
const int trig_max_angle = trig_number_of_entries-1;

int convert_to_trig_angle(int angle_in_degrees);
int convert_to_trig_angle_from_tenths(int angle_in_tenths_of_degrees);
int fast_sine(int trig_angle);
int fast_cosine(int trig_angle);

const int angle90degrees = trig_number_of_entries/(360/90);
const int angle180degrees = trig_number_of_entries/(360/180);
const int angle45degrees = trig_number_of_entries/(360/45);
const int angle_minus90degrees = -trig_number_of_entries/(360/90);
const int angle_minus45degrees = -trig_number_of_entries/(360/45);
const int angle10degrees = trig_number_of_entries/(360/10);
const int min_adjust_angle = trig_number_of_entries/(360/3);

#endif
