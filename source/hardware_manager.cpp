/*
 *  hardware_manager.cpp
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

#include "hardware_manager.h"
#include "led_control.h"
#include "robot_hardware_defs.h"
#include "pc_uart.h"
#include "language_support.h"

// ****************************************************************
//
// This class defines the main controller
//
class Main_Controller_Board : public Hardware_Manager {
	
public:
	Main_Controller_Board();
	
	virtual LED& status_LED() { return blueLED; }
	virtual LED& activity_LED() { return yellowLED; }
	virtual LED& error_LED() { return redLED; }

	virtual Button* button1() { return &mbutton1; }
	virtual Button* button2() { return &mbutton2; }
	virtual Button* button3() { return &mbutton3; }
	
	virtual Buzzer* buzzer() { return &mbuzzer; }
	
	virtual stored_parameters* get_params() { return nvm_param; }
	virtual void set_params(stored_parameters* params) { nvm_param = params; }

	virtual IOPort* motor_1_current_sense() { return &m1_current; }
	virtual IOPort* motor_2_current_sense() { return &m2_current; }

private:
	LED blueLED;
	LED yellowLED;
	LED redLED;
	
	Button mbutton1;
	Button mbutton2;
	Button mbutton3;
	
	Buzzer mbuzzer;
	stored_parameters* nvm_param;
	
	IOPort m1_current;
	IOPort m2_current;
};


//
// Constructor must ensure all the LED's are instantiated with the right port
//
Main_Controller_Board::Main_Controller_Board()
: 
blueLED(MAIN_blue_led_port), 
yellowLED(MAIN_yellow_led_port), 
redLED(MAIN_red_led_port),
mbutton1(MAIN_red_switch_1_port),
mbutton2(MAIN_yellow_switch_2_port),
mbutton3(MAIN_blue_switch_3_port),
nvm_param(0),
m1_current(MAIN_motor1_current_limit_sense, false),	// input not output
m2_current(MAIN_motor2_current_limit_sense, false)	// input not output
{
}


// ****************************************************************
//
// This class defines the main controller
//
class Camera_Controller_Board : public Hardware_Manager {

public:
	Camera_Controller_Board();
	
	virtual LED& status_LED() { return yellow1LED; }
	virtual LED& activity_LED() { return yellow2LED; }
	virtual LED& error_LED() { return redLED; }

	// there are no buttons on the camera board
	virtual Button* button1() { return &myButton; }
	
private:
	LED yellow1LED;
	LED yellow2LED;
	LED redLED;
	Button myButton;
	
};

//
// Constructor must ensure all the LED's are instantiated
//
Camera_Controller_Board::Camera_Controller_Board()
: 
yellow1LED(CC3_led3_port), 
yellow2LED(CC3_led2_port), 
redLED(CC3_sdcard_slot_spi_mosi_plus_led_port),
myButton(CC3_bootloader_select_sdcard_slot_spi_cs_port)
{
}


// ****************************************************************
//
// These are definitions for the base class
//

//
// This is the variable that tells us what instance we are using
//
Hardware_Manager* Hardware_Manager::instance = 0;

//
// Constructor for base ... does nothing except
//
Hardware_Manager::Hardware_Manager()
{
}

//
// Class based variable that instantiates correct board
//
Hardware_Manager* Hardware_Manager::Instance()
{
	if(instance == 0)
	{
		if(is_main_processor())
		{
			instance = new Main_Controller_Board;
		}
		else
		{
			instance = new Camera_Controller_Board;
		}
	}
	
	return instance;
}


//
// Are we on the main processor?
//
bool Hardware_Manager::is_main_processor()
{
	return !is_camera_processor();
}


//
// Are we on the camera processor?
//
bool Hardware_Manager::is_camera_processor()
{
	int port_data = GPIO_IOPIN & (1<<MAIN_board_identifier_port);
	return port_data==0;
}

//
// Unless overridden we assume none...
//
Button* Hardware_Manager::button1()
{
	write_line("No button");
	halt();
	return 0;
}

Button* Hardware_Manager::button2()
{
	return button1();
}

Button* Hardware_Manager::button3()
{
	return button1();
}

Buzzer* Hardware_Manager::buzzer()
{
	write_line("No buzzer");
	halt();
	return 0;
}

stored_parameters* Hardware_Manager::get_params()
{
	write_line("No params");
	halt();
	return 0;
}
void Hardware_Manager::set_params(stored_parameters* params) { }

IOPort* Hardware_Manager::motor_1_current_sense()
{
	halt();
	return 0;
}
IOPort* Hardware_Manager::motor_2_current_sense()
{
	halt();
	return 0;
}

