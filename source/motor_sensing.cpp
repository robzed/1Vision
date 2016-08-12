/*
 *  motor_sensing.cpp
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

#include "motor_sensing.h"

#include "lpc210x_gnuarm.h"
#include "robot_hardware_defs.h"
#include "hardware_support.h"

#include "pc_uart.h"
#include "timing.h"
#include "tick_timer.h"

#include "basic_motor_primitives.h"
#include "led_control.h"


enum { motor1_sense_data, motor2_sense_data, number_of_motors };

Motor_input_data motor_data[number_of_motors];

//
// Return the motor sense object related to a specific motor
// 
Motor_input_data* motor_sense(int motor_number)
{
	// protection
	if(motor_number != 1)
	{
		motor_number = 2;
	}
	
	return &motor_data[motor_number-1];
}

//
// Constructor
//
Motor_input_data::Motor_input_data()
: edge_count(0)
#if STORE_READINGS
,reading_index(0)
#endif
, old_timer_value(0)
, timer_average(0x7fffffff)
, sense_ptr(0)			// no sensing callback initially
{
#if STORE_READINGS
	for(int i=0; i<number_of_readings; i++)
	{
		timer_readings[i] = 0;
	}
#endif	
}

const int FIN_INTERRUPTS_PER_ROTATION = 24;		// twelve holes in interrupt disk
const int WHEEL_CIRCUM_IN_um = 102100;		// radius * pi, where radius = 32.5mm / 2
const int GEARBOX_RATIO = 16;
const int DISTANCE_PER_EDGE_INTERRUPT_IN_um = WHEEL_CIRCUM_IN_um / (FIN_INTERRUPTS_PER_ROTATION*GEARBOX_RATIO);

speed_t convert_to_speed(motortime_t timer_count)
{
	// speed = freq*edge_dist / timer_count
	// watch for overflow of frequency*distance
	return (((pclk/8)*DISTANCE_PER_EDGE_INTERRUPT_IN_um) / timer_count) * 8;
}

motortime_t convert_to_timer_count(speed_t speed_in_um_per_sec)
{
	// timer_count = freq*edge_dist / speed
	// watch for overflow of frequency*distance
	return (((pclk/8)*DISTANCE_PER_EDGE_INTERRUPT_IN_um) / speed_in_um_per_sec) * 8;	
}

speed_t get_average_time(int motor_number)
{	
	return motor_data[motor_number-1].return_average_time();
}

int get_motor_input_count(int motor_number)
{
	return motor_data[motor_number-1].return_tick_count();
}

void clear_motor_input_count(int motor_number)
{
	motor_data[motor_number-1].clear_count();
}

int convert_to_um(int ticks)
{
	return (ticks * DISTANCE_PER_EDGE_INTERRUPT_IN_um);
}

unsigned int convert_to_sensor_ticks(unsigned um)
{
	return um / DISTANCE_PER_EDGE_INTERRUPT_IN_um;
	// more accurate contains a divide and multiply
	//
	// is accuracy really warranted?
	//return (um * FIN_INTERRUPTS_PER_ROTATION*GEARBOX_RATIO) / WHEEL_CIRCUM_IN_um;
}


void Motor_input_data::pulse(timertick_t timer_val)
{
	// only record value if we don't have a duplicate of the previous value
	if(timer_val != old_timer_value)
	{
		// record the reading so we can calculate speed at a later date
		motortime_t timer_difference = timer_val - old_timer_value;
#if STORE_READINGS
		timer_readings[reading_index] = timer_difference;

		reading_index++;
		if(reading_index >= number_of_readings) { reading_index = 0; }
#endif

		// increment the count for position dead reckoning
		edge_count++;

		// record old value for next time
		old_timer_value = timer_val;
		// calculate the average
		//timer_average = timer_average - (timer_average>>3) + (timer_difference>>3); // 7/8 of old + 1/8 of new

		// average two samples because mark is so not equal to space!
		// but gives *almost* immediate change of speed data
		timer_average = (old_timer_difference + timer_difference) / 2;
		old_timer_difference = timer_difference;
		
		if(sense_ptr)
		{
			sense_ptr->edge_happened(timer_average, timer_difference, edge_count);
		}
	}
}

//
// Change routine called when timer expires
//
Sensing_Callback* Motor_input_data::set_edge_reading(Sensing_Callback* new_routine)
{
	Sensing_Callback *old = sense_ptr;
	sense_ptr = new_routine;
	return old;
}



//void Motor_input_data:make_average()
//{
//	int total;
//	for(int i=0; i<number_of_readings; i++)
//	{
//		total+=timer_readings[i];
//	}
//	current_average = total / number_of_readings;
//}


void Motor_input_data::print()
{
	//write_string("  count = "); write_int(edge_count); write_cr();
	//write_string("idx="); write_int(reading_index); write_cr();
	//write_int(old_value); write_cr();
	//write_cr();
	//write_line("Timer values");
	
	write_string("  average = "); write_int(timer_average); write_cr();
	
#if STORE_READINGS
	for(int i=reading_index; i<number_of_readings; i++)
	{
		write_int(timer_readings[i]); 
		write_cr();
		delay(10);
	}
	for(int i=0; i<reading_index; i++)
	{
		write_int(timer_readings[i]); 
		write_cr();
		delay(10);
	}
	write_cr();
	write_cr();
#endif
}

void print_motor_data()
{
	for(int i=0; i<number_of_motors; i++)
	{
		write_string("Motor "); write_int(i+1); write_cr();
		motor_data[i].print();
	}

}


#define VIC_Channel_Timer1  5

//#define TxTCR_COUNTER_ENABLE (1<<0)
//#define TxTCR_COUNTER_RESET  (1<<1)
//#define TxMCR_INT_ON_MR0     (1<<0)
//#define TxMCR_RESET_ON_MR0   (1<<1)
//#define TxIR_MR0_FLAG        (1<<0)

static bool off_switch = 0;

void __attribute__ ((interrupt("IRQ"))) motor_photo_interrupt()
{
	//static bool led_toggle;	
	//if(led_toggle) { ENABLE_LED(); led_toggle=false; } else { DISABLE_LED(); led_toggle=true; } 
	

	// store both before & after 
	timertick_t first_capture_value0 = T1_CR0;
	timertick_t first_capture_value1 = T1_CR1;
	T1_IR = 0x30;						// clear channel 0 and 1 capture event interrupt flag
	timertick_t second_capture_value0 = T1_CR0;	
	timertick_t second_capture_value1 = T1_CR1;

	if(!off_switch) {
		motor_data[motor1_sense_data].pulse(first_capture_value0);		// record captured timer data
		motor_data[motor2_sense_data].pulse(first_capture_value1);

		motor_data[motor1_sense_data].pulse(second_capture_value0);
		motor_data[motor2_sense_data].pulse(second_capture_value1);
	}
	
	// NOTE 1: We will get another interrupt request straight away for any
	// IR's not serviced.

	// NOTE 2:
	// DO BUG FIX
	// try to get around "TIMER.1 missed interrupt potential errata" (see below routine for details)
	//
	// It's advised that you can compare it with the previous value and use this if 
	// they are different after clearing. We actually do this inside pulse. This avoids any complexity
	// in this routine and avoid general duplicate values otherwise.

	//
	// write the vector address register to update the priority
	//
	VICVectAddr = 0;       // Acknowledge Interrupt for the VIC
}

/* From Philips Semiconductors LPC2106 Erratasheet  

TIMER.1 Missed Interrupt Potential 

Introduction: The Timers may be configured so that events such as Match and Capture, cause interrupts. Bits in 
the Interrupt Register (IR) indicate the source of the interrupt, whether from Capture or Match. 
Problem: If more than one interrupt for multiple Match events using the same Timer are enabled, it is possible 
that one of the match interrupts may not be recognized. If this occurs no more interrupts from that 
specific match register will be recognized. This could happen in a scenario where the match events 
are very close to each other. This issue also affects the Capture functionality. 

Specific details: 
Suppose that two match events are very close to each other (Say Match0 and Match1). Also 
assume that the Match0 event occurs first. When the Match0 interrupt occurs the 0th bit of the 
Interrupt Register will be set. To exit the Interrupt Service Routine of Match0, this bit has to be 
cleared in the Interrupt Register. The clearing of this bit might be done by using the following 
statement: 

T0_IR = 0x1; 

It is possible that software will be writing a 1 to bit 0 of the Interrupt Register while a Match1 event 
occurs, meaning that hardware needs to set the bit 1 of the Interrupt Register. In this case, since 
hardware is accessing the register at the same time as software, bit 1 for Match1 never gets set, 
causing the interrupt to be missed. 

In summary, while software is writing to the Interrupt Register, any Match or Capture event (which 
are configured to interrupt the core) occurring at the same time may result in the subsequent 
interrupt not being recognized. 

Similarly for the Capture event, if a capture event occurs while a Match event is being is serviced 
then the Capture event might be missed if the software and hardware accesses coincide. 

Affected features: 
1. Interrupt on Match for Timer0/1. 
2. Interrupt on Capture for Timer0/1. 
3. These same features will be affected when using PWM. 

Work-around: There is no clear workaround for this problem but some of the below mentioned solutions could 
work with some applications. 

Possible workarounds for Match functionality: 
1. If the application only needs two Match registers then distribute them between Timer 0 and       
Timer 1 to avoid this problem. 
2. Stop the timer before accessing the Interrupt register for clearing the interrupt and then start timer 
again after the access is completed. 

3. Polling for interrupt: Supposing that there are two Match events (Match X and Match Y). At the 
end of the Interrupt Service Routine (ISR) for Match X, compare the Timer Counter value with the 
Match Register Y value. If the Timer Counter value is more than the Match Register Y value then 
it is possible that this event might have been missed. In this case jump to the ISR directly and 
service Match event Y. 
Possible workarounds for Capture functionality: 

1. Try to spread the capture events between both timers if there are two capture events. If the 
application also has a match event then one of the capture events may suffer. 

2. Polling for Capture: At the end of a Match interrupt ISR or Capture event ISR compare the 
previous Capture value with the current Capture value. If the Capture value has changed then 
the Capture event might have been missed. In this case, jump to the ISR directly and service the 
Capture event. 


PWM.1 Missed Interrupt Potential for the Match functionality. The description is same as above. 

*/


void switch_sensing_off()
{
	off_switch = 1;
}



void set_up_motor_photo_interruptors()
{
	pin_function_select(10,2);			// select timer input
	pin_function_select(11,2);			// select timer input
	
	T1_TC = 0;			// set timer counter to zero initially
	T1_PC = 0;			// set the prescaler counter to zero
	T1_PR = 0;			// set the prescale register to zero ... divide by 1
	
	T1_MCR = 0;			// don't take any action on a match (actions are reset, stop, interrupt)
//	T1_CCR = 0x03f;		// for both capture 0 and 1, we want a capture on rising, falling and an interrupt, please.	
	T1_CCR = 0x03f;		// for both capture 0 and 1, we want a capture on rising, falling and an interrupt, please.	
	
	// T1_MRx ... we don't write to these
	T1_EMR = 0;			// don't output anything on a match
	
	// enable interrupts for photo interrupters
	VICVectAddr0 = (unsigned long)motor_photo_interrupt;   // set interrupt vector in 0
	VICVectCntl0 = VICVectCntl_ENABLE | VIC_Channel_Timer1; // use it for Timer 1 Interrupt:
	VICIntEnable = (1<<VIC_Channel_Timer1);    // Enable Timer 1 Interrupt		
	
	// finally enable timer
	T1_TCR = 0x01;		// release reset and enable

}

