// arctan.h
// Created by Rob Probin 28th March 2009
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
#ifndef ARCTAN_H
#define ARCTAN_H

const int arctan_full_circle_angle = 2048;			// this angle represents 360 degrees
int fast_arctan(int numerator, int demoninator);

void arctan_tests();			// test this arctan works

inline int trig_angle_to_degrees(int trig_angle) { return (trig_angle*360)/arctan_full_circle_angle; }

#endif
