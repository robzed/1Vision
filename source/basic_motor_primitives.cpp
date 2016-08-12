/*
 *  basic_motor_primitives.cpp
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

#include "basic_motor_primitives.h"

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        motor_1_full_forward
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 17th June 2006
// +
// | DESCRIPTION:  C version of Macro to control motor
// +----------------------------------------------------------------ROUTINE HEADER----
inline void motor_1_full_forward()
{
	DISABLE_MOTOR_1(); 
	//	very_short_delay(); // need delay here?
	MOTOR_1_SELECT_FORWARD(); 
	ENABLE_MOTOR_1();
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        motor_1_full_reverse
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 17th June 2006
// +
// | DESCRIPTION:  C version of Macro to control motor
// +----------------------------------------------------------------ROUTINE HEADER----
inline void motor_1_full_reverse()
{
	DISABLE_MOTOR_1(); 
	MOTOR_1_SELECT_BACKWARD(); 
	ENABLE_MOTOR_1();
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        motor_1_fast_stop
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 17th June 2006
// +
// | DESCRIPTION:  C version of Macro to control motor
// +----------------------------------------------------------------ROUTINE HEADER----
inline void motor_1_fast_stop()
{	
	DISABLE_MOTOR_1(); 
	MOTOR_1_SELECT_FAST_STOP(); 
	ENABLE_MOTOR_1();
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        motor_1_remove_power
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 17th June 2006
// +
// | DESCRIPTION:  C version of Macro to control motor
// +----------------------------------------------------------------ROUTINE HEADER----
inline void motor_1_remove_power()
{		
	DISABLE_MOTOR_1();			// free run the motor
	//MOTOR_1_SELECT_FAST_STOP(); // this has no effect except if enable is turned on again
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        motor_2_full_forward
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 17th June 2006
// +
// | DESCRIPTION:  C version of Macro to control motor
// +----------------------------------------------------------------ROUTINE HEADER----
inline void motor_2_full_forward()
{		
	DISABLE_MOTOR_2(); 
	MOTOR_2_SELECT_FORWARD(); 
	ENABLE_MOTOR_2();
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        motor_2_full_reverse
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 17th June 2006
// +
// | DESCRIPTION:  C version of Macro to control motor
// +----------------------------------------------------------------ROUTINE HEADER----
inline void motor_2_full_reverse()
{		
	DISABLE_MOTOR_2(); 
	MOTOR_2_SELECT_BACKWARD(); 
	ENABLE_MOTOR_2();
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        motor_2_fast_stop
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 17th June 2006
// +
// | DESCRIPTION:  C version of Macro to control motor
// +----------------------------------------------------------------ROUTINE HEADER----
inline void motor_2_fast_stop()
{
	DISABLE_MOTOR_2(); 
	MOTOR_2_SELECT_FAST_STOP(); 
	ENABLE_MOTOR_2();
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        motor_2_remove_power
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 17th June 2006
// +
// | DESCRIPTION:  C version of Macro to control motor
// +----------------------------------------------------------------ROUTINE HEADER----
inline void motor_2_remove_power()
{		
	DISABLE_MOTOR_2();			// free run the motor
	//MOTOR_2_SELECT_FAST_STOP(); // this has no effect except if enable is turned on again
}

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        init_motor_direction_pins
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 10th June 2006
// +
// | DESCRIPTION:
// +----------------------------------------------------------------ROUTINE HEADER----

void init_motor_direction_pins()
{
	GPIO_IODIR |= (1<<MOTOR_1_ENABLE_PIN);
	DISABLE_MOTOR_1();
	GPIO_IODIR |= (1<<MOTOR_1_IN_1_PIN);
	GPIO_IODIR |= (1<<MOTOR_1_IN_2_PIN);
	
	GPIO_IODIR |= (1<<MOTOR_2_ENABLE_PIN);
	DISABLE_MOTOR_2();
	GPIO_IODIR |= (1<<MOTOR_2_IN_1_PIN);
	GPIO_IODIR |= (1<<MOTOR_2_IN_2_PIN);
}


//
//
//
//#include "pc_uart.h"
#if 0		// this function is not used currently
void motor_full_forward(int motor)
{
	//write_string("f"); write_int(motor);

	if(motor==1)
	{
		motor_1_full_forward();
	}
	else
	{
		motor_2_full_forward();
	}	
}
#endif


typedef enum { 
	motor_backward = -1, 
	motor_off = 0, 
	motor_forward = 1
} motor_direction_t;

static motor_direction_t motor_1_non_overriden_state = motor_off;
static motor_direction_t motor_2_non_overriden_state = motor_off;
static bool motor_1_override = false;
static bool motor_2_override = false;

void restore_motor1()
{
	if(motor_1_non_overriden_state == motor_forward)
	{
		motor_1_full_forward();
	}
	else if(motor_1_non_overriden_state == motor_backward)
	{
		motor_1_full_reverse();
	}
	motor_1_override = false;
}

void restore_motor2()
{
	if(motor_2_non_overriden_state == motor_forward)
	{
		motor_2_full_forward();
	}
	else if(motor_2_non_overriden_state == motor_backward)
	{
		motor_2_full_reverse();
	}
	motor_2_override = false;
}

void override_and_turn_motor1_off()
{
	motor_1_remove_power();
	motor_1_override = true;
}

void override_and_turn_motor2_off()
{
	motor_2_remove_power();
	motor_2_override = true;
}


//
//
//
void motor_remove_power(int motor)
{
	//write_string("r"); write_int(motor);

	if(motor==1)
	{
		motor_1_remove_power();
		motor_1_non_overriden_state = motor_off;
	}
	else
	{
		motor_2_remove_power();
		motor_2_non_overriden_state = motor_off;
	}	
}

void motor_full(int motor, bool forward)
{
	if(forward)
	{
		if(motor==1)
		{
			if(!motor_1_override)
			{
				motor_1_full_forward();
			}
			motor_1_non_overriden_state = motor_forward;
		}
		else
		{
			if(!motor_2_override)
			{
				motor_2_full_forward();
			}
			motor_2_non_overriden_state = motor_forward;
		}
	}
	else
	{
		if(motor==1)
		{
			if(!motor_1_override)
			{
				motor_1_full_reverse();
			}
			motor_1_non_overriden_state = motor_backward;
		}
		else
		{
			if(!motor_2_override)
			{
				motor_2_full_reverse();
			}
			motor_2_non_overriden_state = motor_backward;
		}
	}
}


