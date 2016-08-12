/*
 *  tick_timer.cpp
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

#include "tick_timer.h"

#include "lpc210x_gnuarm.h"
#include "robot_hardware_defs.h"
#include "language_support.h"
#include "hardware_support.h" // for buzzer code
#include "basic_motor_primitives.h"

//volatile static unsigned int interrupt_ticks = 0;
volatile static unsigned int subticks = 0;
//static unsigned int cpu_ticks_upper = 0;

const unsigned int REQUIRED_TICK_FREQUENCY = 100;		// in Hz
//const unsigned int REQUIRED_TIMER_FREQUENCY = 1600;		// in Hz ... contraints: 
//const int REQUIRED_TIMER_FREQUENCY = 2400;		// in Hz ... contraints: 
												//   1. must be divisor of main clock
												//   2. half is frequency of buzzer
												//   3. lower means less interrupts and processor cycles lost
												//   4. higher means more resolution for 
												//   5. easy divide to get 100Hz means less complexity here and in timer routines
//const unsigned int SHIFT_TO_GET_REQUIRED_FREQUENCY = 4;
const unsigned int REQUIRED_TIMER_FREQUENCY = 3200;		// in Hz ... contraints: 
const unsigned int SHIFT_TO_GET_REQUIRED_FREQUENCY = 5;
#define TIMER_DIVIDE_FOR_REGISTER_CALLBACK 2		// runs at 1600Hz, i.e. 3200/2


//class Timer_Callbacks {
//	typedef void (*fptr_t)(void);
//	static void add_func(fptr_t function);
//	static void remove_func(fptr_t function);
//	Timer_Callbacks();
//	~Timer_Callbacks();
//	void interrupt();
//private:
//	const static number_functions = 2;
//	static fptr_t flist[number_functions];
//	static Timer_Callbacks* 
//};
//
//void Timer_Callbacks::add_func(fptr_t function)
//{
//	for(int i=0; i<number_of_functions; i++)
//	{
//		if(flist[i]==0)
//		{
//			flist[i] = function;
//			return;
//		}
//	}
//	halt("interrupt f() full");
//}
//void Timer_Callbacks::remove_func(fptr_t function)
//{
//	for(int i=0; i<number_of_functions; i++)
//	{
//		if(flist[i]==function)
//		{
//			flist[i]=0;
//			return;
//		}
//	}
//	halt("interrupt f() no found");
//}
//void Timer_Callbacks::interrupt()
//{
//	for(int i=0; i<number_of_functions; i++)
//	{
//		if(flist[i]==0)
//		{
//			flist[i]();
//		}
//	}
//}
//
//Timer_Callbacks ifunc;

#define NUMBER_INTERRUPT_FUNCTIONS 2	// allow preprocessor
const static int number_interrupt_functions = NUMBER_INTERRUPT_FUNCTIONS;
static Timer_Callback* flist[number_interrupt_functions] = { 0, 0 };
static unsigned int callback_count[number_interrupt_functions];

void register_timer_callback(Timer_Callback* callback_object, unsigned int ticks_till_call)
{
	for(int i=0; i<number_interrupt_functions; i++)
	{
		if(flist[i]==0)
		{
			callback_count[i] = ticks_till_call;
			flist[i] = callback_object;
			return;
		}
	}
	halt("interrupt f() full");
}

void delete_timer_callback(Timer_Callback* callback_object)
{
	for(int i=0; i<number_interrupt_functions; i++)
	{
		if(flist[i]==callback_object)
		{
			flist[i]=0;
			return;
		}
	}
	halt("interrupt f() no found");
}

#if NUMBER_INTERRUPT_FUNCTIONS != 2
static void interrupt_callbacks()
{
	for(int i=0; i<number_interrupt_functions; i++)
	{
		if(flist[i] != 0)
		{
			callback_count[i] --;
			if(callback_count[i] == 0)
			{
				callback_count[i] = flist[i]->interrupt();
			}
		}
	}
}
#endif

//#define ENABLE_CURRENT_LIMIT

bool is_current_limit_on()
{
#if ENABLE_CURRENT_LIMIT
	return true;
#else
	return false;
#endif
}

#ifdef ENABLE_CURRENT_LIMIT_WARNING
static int current_limit_warning_channel = 3;	// can be 1 or 2 or 3 for both
static int current_sense_hold = 0;

void set_current_limit_warning_channel(int channel)
{
	current_limit_warning_channel = channel;
}
#endif


void  __attribute__ ((interrupt("IRQ"))) timer_interrupt()
{
	subticks++;
	//if(subticks==(REQUIRED_TIMER_FREQUENCY/REQUIRED_TICK_FREQUENCY))
	//{
	//	subticks = 0;
	//	interrupt_ticks++;
	//}
	//cpu_ticks_upper+= pclk / REQUIRED_TIMER_FREQUENCY;

	//
	// Monitor motor current limit in software and stop motor if exceeds that amount
	//
#ifdef ENABLE_CURRENT_LIMIT
	if(!QUICK_GET_MOTOR1_CURRENT_SENSE())
	{
		// there is a current overload
		// turn off the motor
		override_and_turn_motor1_off();
		
#if defined(ENABLE_CURRENT_LIMIT_WARNING)
		if(current_limit_warning_channel & 1) current_sense_hold = 16;		// 16 / 1600Hz = 0.01s = 10ms
#endif
	}
	else
	{
		restore_motor1();		
#if defined(ENABLE_CURRENT_LIMIT_WARNING)
		if(current_limit_warning_channel & 1) { if(current_sense_hold > 0) { current_sense_hold--; }} // decrement until zero if off
#endif
	}
	if(!QUICK_GET_MOTOR2_CURRENT_SENSE())
	{   // there is a current overload so turn off the motor
		override_and_turn_motor2_off();

#if defined(ENABLE_CURRENT_LIMIT_WARNING)
		if(current_limit_warning_channel & 2) current_sense_hold = 16;		// 16 / 1600Hz = 0.01s = 10ms
#endif
	}
	else
	{
		restore_motor2();
#if defined(ENABLE_CURRENT_LIMIT_WARNING)
		if(current_limit_warning_channel & 2)  { if(current_sense_hold > 0) { current_sense_hold--; }} // decrement until zero if off
#endif
	}
	
#ifdef ENABLE_CURRENT_LIMIT_WARNING
	if(current_sense_hold)
	{	// light up LED / Buzzer
		pin_function_select(MAIN_sound_output_port, 2);	// select the timer match output
		GPIO_IOSET=(1<<MAIN_blue_led_port); 
	}
	else
	{	// turn off LED / Buzzer
		pin_function_select(MAIN_sound_output_port, 0);	
		GPIO_IOCLR=(1<<MAIN_blue_led_port);
	}
#endif
#endif	// ENABLE_CURRENT_LIMIT

#if (TIMER_DIVIDE_FOR_REGISTER_CALLBACK == 2)
	if(subticks & 0x01)				// divide by two
#elif (TIMER_DIVIDE_FOR_REGISTER_CALLBACK == 4)
	if((subticks & 0x03)==0)		// divide by four
#elif (TIMER_DIVIDE_FOR_REGISTER_CALLBACK != 1)			// if it's one we don't need a divider at all
	Need to add code in here
#endif
	{
#if (NUMBER_INTERRUPT_FUNCTIONS != 2)
		interrupt_callbacks();		
#else
		if(flist[0] != 0)
		{
			callback_count[0] --;
			if(callback_count[0] == 0)
			{
				callback_count[0] = flist[0]->interrupt();
			}
		}
		if(flist[1] != 0)
		{
			callback_count[1] --;
			if(callback_count[1] == 0)
			{
				callback_count[1] = flist[1]->interrupt();
			}
		}
#endif
	}
	
	T0_IR = 0x01; // Clear interrupt flag for timer match 0 by writing 1 to Bit 0
				  // write the vector address register to update the priority
	VICVectAddr = 0;       // Acknowledge Interrupt for the VIC
}




#define VIC_Channel_Timer0  4

void set_up_timer_interrupt()
{
	T0_TC = 0;			// set timer counter to zero initially
	T0_PC = 0;			// set the prescaler counter to zero
	T0_PR = 0;			// set the prescale register to zero ... divide by 1
	
	T0_TCR = 0x02;		// reset
	
	T0_MCR = 0x03;	// reset and interrupt on match register 0 (don't stop though)
	T0_CCR = 0;		// no capture
	
	T0_MR0 = pclk / REQUIRED_TIMER_FREQUENCY;		// set up the timer frequency
	
	T0_EMR = 0;			// don't output anything on a match
	
	// enable interrupts for photo interrupters
	VICVectAddr1 = (unsigned long)timer_interrupt;   // set interrupt vector in 1
	VICVectCntl1 = VICVectCntl_ENABLE | VIC_Channel_Timer0; // use it for Timer 1 Interrupt:
	VICIntEnable = (1<<VIC_Channel_Timer0);    // Enable Timer 1 Interrupt		
	
	// testing purposes....
	//VICDefVectAddr = (unsigned long)timer_interrupt;   // set interrupt vector in 1
	
	// finally enable timer
	T0_TCR = 0x01;		// release reset and enable	
}


void alter_tick_timer_frequency(unsigned int frequency)
{
	T0_TC = 0;			// set timer counter to zero initially
	T0_MR0 = pclk / frequency;		// set up the timer frequency	
}
void alter_tick_timer_frequency_cHz(unsigned int frequency_in_cHz)
{
	T0_TC = 0;			// set timer counter to zero initially
	
	// can't multiple pclk by 100 - we will overflow.
	unsigned int pclk_mod = static_cast<unsigned int>(pclk)*50;
	// convert to same units
	frequency_in_cHz /= 2;
	T0_MR0 = pclk_mod / frequency_in_cHz;		// set up the timer frequency	
}
void default_tick_timer_frequency()
{
	T0_TC = 0;			// set timer counter to zero initially	
	T0_MR0 = pclk / REQUIRED_TIMER_FREQUENCY;		// set up the timer frequency
}

unsigned int fetch_timer_ticks()
{
	return (subticks>>SHIFT_TO_GET_REQUIRED_FREQUENCY);
}

unsigned int fetch_fast_ticks()
{
	return subticks;
}

unsigned int how_many_fast_ticks_per_second()
{
	return REQUIRED_TIMER_FREQUENCY;
}


// assumes pclk = 58.9...MHz
// assumes interrupt_ticks are 10ms each
//unsigned int nanoseconds()
//{
//	unsigned int tticks = T0_TC;
//	unsigned int ten_ms_ticks = interrupt_ticks;
//	
//	// each T0_TC is 16.954ns each.
//	// We assume T0_TC max is 589824 (10ms*58.9...MHz)
//	// therefore we can multiply by 1695 and still be safe
//	unsigned int result = (tticks * 1695);	// 1695 is 100 times 1/58.9...MHz
//	
//	// scale back to actual value
//	result /= 100;
//	
//	// 10ms = 10,000us = 10,000,000ns
//	result += ten_ms_ticks*10000000;
//	
//	return result;
//}


unsigned int cpu_ticks()
{
	unsigned int subticks_copy = subticks;
	unsigned int tticks = T0_TC;
		
	while(subticks_copy != subticks)		// if subticks changes readings are invalid
	{
		subticks_copy = subticks;
		tticks = T0_TC;
	}

	unsigned int our_cpu_ticks = tticks + subticks_copy * (pclk / REQUIRED_TIMER_FREQUENCY);
	
	return our_cpu_ticks;
}


#define WRITE_TICKS_ENABLED 0

#if WRITE_TICKS_ENABLED
#include "pc_uart.h"

unsigned int s1[100];
unsigned int s2[100];
unsigned int s3[100];
#endif

void write_ticks()
{
#if WRITE_TICKS_ENABLED

#warning "Need to fix this ... no more interrupt ticks"
	
	for(int i=0; i<100; i++)
	{
		unsigned int ten_ms_ticks = interrupt_ticks;
		unsigned int ticks_2400 = subticks;
		unsigned int tticks = T0_TC;
		
		unsigned int combined_ticks = ten_ms_ticks*(REQUIRED_TIMER_FREQUENCY/REQUIRED_TICK_FREQUENCY)+ticks_2400;
		
		while(combined_ticks != interrupt_ticks*(REQUIRED_TIMER_FREQUENCY/REQUIRED_TICK_FREQUENCY)+subticks)		// if ticks2400 changes then tticks is invalid
		{
			ten_ms_ticks = interrupt_ticks;
			ticks_2400 = subticks;
			tticks = T0_TC;

			combined_ticks = ten_ms_ticks*(REQUIRED_TIMER_FREQUENCY/REQUIRED_TICK_FREQUENCY)+ticks_2400;
		}
		unsigned int our_cpu_ticks = tticks + combined_ticks * (pclk / REQUIRED_TIMER_FREQUENCY);
		
		s1[i]=our_cpu_ticks;
		s2[i]=tticks;
		s3[i]=combined_ticks;
		write_line(">",i);
	}
	
	for(int i=0; i<100; i++)
	{
		write_uint(s1[i]); write_string("=");
		write_uint(s2[i]); write_string(" ");
		write_uint(s3[i]); write_cr();
	}

//	
//	unsigned int cpu_ticks_upper_copy = cpu_ticks_upper;
//	unsigned int tticks = T0_TC;
//	while(cpu_ticks_upper_copy != cpu_ticks_upper_copy)		// if cpu_ticks_upper changes then tticks is invalid
//	{
//		cpu_ticks_upper_copy = cpu_ticks_upper;
//		tticks = T0_TC;
//	}
//	unsigned int our_cpu_ticks = tticks + cpu_ticks_upper_copy;
//	
//	write_uint(our_cpu_ticks); write_string("=");
//	write_uint(tticks); write_string(" ");
//	write_uint(cpu_ticks_upper_copy); write_cr();
#endif
}
