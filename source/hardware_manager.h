/*
 *  hardware_manager.h
 *  robot_core
 *
 *  Created by Rob Probin on 20/07/2007.
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
 *
 */

#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include "led_control.h"
#include "button.h"
#include "buzzer.h"
#include "ipc_serial_driver.h"
#include "stored_parameters.h"

//
// We define the Hardware Manager as a singleton - there
// can only be one. But this is ok since there is only one 
// piece of hardware (on a single board)!
//
// Internally, however, this could be one of two actual boards.
//
class Hardware_Manager {
public:
	static Hardware_Manager* Instance();
	
	static bool is_main_processor();			// these are static because we only ever need one and the init uses it
	static bool is_camera_processor();			// these are static because we only ever need one and the init uses it
	
	// LEDs or proxies for them, must exist ... although they might 
	// not be independant...
	virtual LED& status_LED() = 0;
	virtual LED& activity_LED() = 0;
	virtual LED& error_LED() = 0;

	// buttons might (Main board) or might not exist (camera)
	virtual Button* button1();
	virtual Button* button2();
	virtual Button* button3();
	
	virtual Buzzer* buzzer();
	
	//virtual ipc_serial_driver* get_ipc_serial_driver();
	virtual stored_parameters* get_params();
	virtual void set_params(stored_parameters* params);
	
	virtual IOPort* motor_1_current_sense();
	virtual IOPort* motor_2_current_sense();
	
	virtual ~Hardware_Manager() { }
	
protected:
	Hardware_Manager();		// don't allow creation this way
private:
	static Hardware_Manager* instance;
};


#endif // HARDWARE_MANAGER_H


