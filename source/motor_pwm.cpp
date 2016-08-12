/*
 *  motor_pwm.cpp
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

#include "motor_pwm.h"

#include "lpc210x_gnuarm.h"
#include "hardware_support.h"
#include "stdint.h"
#include "robot_hardware_defs.h"

// +-------------------------------+-------------------------+-----------------------
// | TITLE:        
// | AUTHOR(s):    Rob & Alan
// | DATE STARTED: 
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void enable_pwm(int motor_1_pwm_setting, int motor_2_pwm_setting)
{
	if(motor_1_pwm_setting < 0) motor_1_pwm_setting = 0;
	if(motor_2_pwm_setting < 0) motor_2_pwm_setting = 0;
	if(motor_1_pwm_setting > 1000) motor_1_pwm_setting = 1000;
	if(motor_2_pwm_setting > 1000) motor_2_pwm_setting = 1000;
	
	// motor 1 = pwm 2 (was: pwm 6)
	// motor 2 = pwm 5
	pin_function_select(MAIN_motor1_enable_port,2);
	pin_function_select(MAIN_motor2_enable_port,1);
	PWM_TC = 0;				// set timer counter to zero initially
	PWM_PC = 0;				// set the prescaler counter to zero
	PWM_PR = 0;				// set the prescale register to zero
	
#if 0
	// currently cclk = FOsc = 14.7456MHz
	// currently pclk = cclk / 4
	// currently pclk = 3.6864 MHz
	// To make period 10 second:
	// prescaler = 10 * 3,686,400 / 1000 = 36864
	// match 0 = 1000
	// match 5/6 = variable
	PWM_PR = 36864;			// prescale to get 10s
							//	PWM_PR = 0x0;			// 0=increment timer on every pclk, 1=increment timer on every 2 pclk
	PWM_MR0 = 1000;			// this is the overall period
#else
	// *** OLD ***
	// currently cclk = FOsc = 14.7456MHz
	// currently pclk = FOsc * 4
	// currently pclk = 58.9824 MHz
	// To make minimum period:
	// prescaler = 0 
	// period of wrap = 58982400 / x*1000 	// match 0 = 1000
	// match 5/6 = variable
	// period of wrap = 58982400 / 3000 = 19660.8 Hz = 50.86us
	// 
	// ** NEW **
	// 58982400 / 100 (count) = 589254
	// 589824 / 8000 (loop freq) = 73.728
	// prescaler value = 73 (slightly above wanted value)
	// wrap frequency = 8079.7 Hz
	// ** NEW **
	// 58982400 / 1000 (count) = 58925.4
	// 58982.4 / 8000 (loop freq) = 7.3728
	// prescaler value = 7 (slightly above wanted value)
	// wrap frequency = 8426 Hz
	
//	PWM_PR = 0;				// prescale to divide by 1 from pclk
	PWM_PR = 6;			// prescale to divide by 7 from pclk
							//	PWM_PR = 0x0;			// 0=increment timer on every pclk, 1=increment timer on every 2 pclk
	
// the pulses come in every 0.256mm, which at top speed of approx 1m/s = 4KHz.
// If we want to do the entire PWM sweep (1 cycle of the PWM) twice in this period
// we will want 29.491KHz / 8KHz = 3 approx.
//#define PWM_ALTERNATE_PRESCALER	3		// previously 2
	
	PWM_MR0 = 1000;										// this is the overall period, i.e. 100 steps
//	PWM_MR0 = (PWM_ALTERNATE_PRESCALER*1000);			// this is the overall period, i.e. 100 steps
#endif
//	PWM_MR5 = motor_2_pwm_setting * PWM_ALTERNATE_PRESCALER;
	PWM_MR5 = motor_2_pwm_setting;
	//PWM_MR6 = motor_1_pwm_setting * PWM_ALTERNATE_PRESCALER;
//	PWM_MR2 = motor_1_pwm_setting * PWM_ALTERNATE_PRESCALER;
	PWM_MR2 = motor_1_pwm_setting;

	PWM_MCR = 0x02;			// reset main timer on Match 0
	
	//PWM_PCR = 0x6000;		// enable PWM 5 and 6 output
	PWM_PCR = 0x2400;		// enable PWM 5 and 2 output
	//PWM_LER = 0x61;			// load match 0, match 5 and match 6 into shadow registers
	PWM_LER = 0x25;			// load match 0, match 5 and match 2 into shadow registers
	
	//PWM_IR = ?;		// we don't use interrupts
	
	// reset all the counters
	//PWM_TCR = 0x0b;			// reset
	
	// delay(1000);
	
	// finally enable the show...
	PWM_TCR = 0x09;			// counter enable + pwm mode enable + release reset
}

// +-------------------------------+-------------------------+-----------------------

void adjust_pwm(int motor_1_pwm_setting, int motor_2_pwm_setting)
{
	if(motor_1_pwm_setting < 0) motor_1_pwm_setting = 0;
	if(motor_1_pwm_setting > 1000) motor_1_pwm_setting = 1000;
	PWM_MR2 = motor_1_pwm_setting;
	//PWM_MR2 = motor_1_pwm_setting * PWM_ALTERNATE_PRESCALER;
	if(motor_2_pwm_setting < 0) motor_2_pwm_setting = 0;
	if(motor_2_pwm_setting > 1000) motor_2_pwm_setting = 1000;
	PWM_MR5 = motor_2_pwm_setting;

	//PWM_MR5 = motor_2_pwm_setting*2;
	//PWM_MR6 = motor_1_pwm_setting*2;
	
	PWM_LER = 0x24;			// not bottom bit for reasons I can't remember
	//PWM_LER = 0x60;
}


// +-------------------------------+-------------------------+-----------------------
// | TITLE:        
// | AUTHOR(s):    Rob & Alan
// | DATE STARTED: 
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void disable_pwm()
{
	PWM_TCR = 0x00;			// disable counter and pwm mode
	pin_function_select(MAIN_motor1_enable_port,0);
	pin_function_select(MAIN_motor2_enable_port,0);
}


// +-------------------------------+-------------------------+-----------------------
// +-------------------------------+-------------------------+-----------------------
// +-------------------------------+-------------------------+-----------------------
// +-------------------------------+-------------------------+-----------------------
// +-------------------------------+-------------------------+-----------------------

static uint8_t pwm_enabled_map = 0;

void set_single_pwm(int motor, int motor_pwm_setting)
{
	if(motor_pwm_setting < 0) motor_pwm_setting = 0;
	if(motor_pwm_setting > 1000) motor_pwm_setting = 1000;
	
	if(pwm_enabled_map==0)
	{
		// motor 1 = pwm 2 (was: pwm 6)
		// motor 2 = pwm 5
		PWM_TC = 0;				// set timer counter to zero initially
		PWM_PC = 0;				// set the prescaler counter to zero
		PWM_PR = 0;				// set the prescale register to zero
		
		// *** OLD ***
		// currently cclk = FOsc = 14.7456MHz
		// currently pclk = FOsc * 4
		// currently pclk = 58.9824 MHz
		// To make minimum period:
		// prescaler = 0 
		// period of wrap = 58982400 / 1000 = 29491Hz = 33.9us
		// match 0 = 1000
		// match 5/6 = variable
		//PWM_PR = 0;				// prescale to divide by 1 from pclk
		PWM_PR = 6;				// prescale to divide by 1 from pclk
		//	PWM_PR = 0x0;			// 0=increment timer on every pclk, 1=increment timer on every 2 pclk
		
		// the pulses come in every 0.256mm, which at top speed of approx 1m/s = 4KHz.
		// If we want to do the entire PWM sweep (1 cycle of the PWM) twice in this period
		// we will want 29.491KHz / 8KHz = 3 approx.
		//PWM_MR0 = (PWM_ALTERNATE_PRESCALER*1000);			// this is the overall period, i.e. 100 steps
		PWM_MR0 = 1000;			// this is the overall period, i.e. 100 steps

		// set both to something...
		PWM_MR2 = 0;
		PWM_MR5 = 0;
	}

	if(motor == 1)
	{
		//PWM_MR2 = motor_pwm_setting * PWM_ALTERNATE_PRESCALER;
		PWM_MR2 = motor_pwm_setting;
	}
	else
	{
		//PWM_MR5 = motor_pwm_setting * PWM_ALTERNATE_PRESCALER;
		PWM_MR5 = motor_pwm_setting;
	}	

	if(pwm_enabled_map==0)
	{		
		
		PWM_MCR = 0x02;			// reset main timer on Match 0
		
		PWM_PCR = 0x2400;		// enable PWM 5 and 2 output
		PWM_LER = 0x25;			// load match 0, match 5 and match 2 into shadow registers
				
		// finally enable the show...
		PWM_TCR = 0x09;			// counter enable + pwm mode enable + release reset	
	}
	else // adjust mode
	{
		if(motor == 1)
		{
			PWM_LER = 0x04;			// not bottom bit - don't adjust basic cycle count
		}
		else
		{
			PWM_LER = 0x20;			// not bottom bit - don't adjust basic cycle count
		}
	}

	if(motor==1)
	{
		pwm_enabled_map |= 1;
		pin_function_select(MAIN_motor1_enable_port,2);
	}
	else
	{
		pwm_enabled_map |= 2;
		pin_function_select(MAIN_motor2_enable_port,1);
	}

}




void disable_single_pwm(int motor)
{
	if(motor==1)
	{
		pwm_enabled_map &= ~1;
		pin_function_select(MAIN_motor1_enable_port,0);
	}
	else
	{
		pwm_enabled_map &= ~2;
		pin_function_select(MAIN_motor2_enable_port,0);
	}	
	if(pwm_enabled_map == 0)
	{
		PWM_TCR = 0x00;			// disable counter and pwm mode
	}
}


