/*
 *  led_control.h
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


#ifndef LED_CONTROL_H
#define LED_CONTROL_H

//void ledInit();
//void flash_led(int number_of_times);
//void led(bool state);



// - Factory class
// - Use constructor to set up RAM variables for port.

// <<more complex than I want>> - Template class (pass in port as template)
// <<stupid>> - Write three sets of code
// <<doesn't work>> - static const LEDs objects in flash

#include "robot_hardware_defs.h"


class LED {
public: 
	LED(byte port_in);
	void on() { LEDport.set(); }
	void off() { LEDport.clear(); }
	void set(bool state);
	void flash(int number_of_times);
private: 
	IOPort LEDport;
};

#endif // LED_CONTROL_H


