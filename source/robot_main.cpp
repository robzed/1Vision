/* Robot Main Function
 *
 * Copyright 2006-2012 Rob Probin
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

//#define CPP_VERSION		// if defined changes the code to C++ class.

#define GLOBALOBJECT
// if definded this demonstrates calling the constructors
// for "global" objects from the startup-code (see crt0.S and *.ld)

#include "VIClowlevel.h"

#include "pc_uart.h"
#include "motor_sensing.h"
#include "motor_pwm.h"
#include "tick_timer.h"
#include "led_control.h"
#include "timing.h"
#include "basic_motor_primitives.h"
#include "hardware_support.h"
#include "command_line.h"
#include "speed_adjust.h"
#include "pos_adjust.h"
#include "hardware_manager.h"
#include "interprocessor_comms.h"
#include "language_support.h"
#include "stored_parameters.h"
#include "Real_World.h"
#include "arctan.h"
#include "simple_wall_follower_engine.h"


void processing_loop();

enum run_type { RT_stop, RT_run_straight, RT_turn_left, RT_turn_right };

static run_type run_main_prog = RT_stop;
static int distance_to_move = 180000;
static int turn_angle = 90;

void run() { run_main_prog = RT_run_straight; }
void stop() { run_main_prog = RT_stop; }
void left() { run_main_prog = RT_turn_left; }
void right() { run_main_prog = RT_turn_right; }

void dist(int x) { distance_to_move = x; }
void angle(int the_angle) { turn_angle = the_angle; };

// 
// 
//

//#define RED_LED 29
//#define YELLOW_LED 27
//#define BLUE_LED 25
//
//#define RED_SWITCH 26
//#define YELLOW_SWITCH 28
//#define BLUE_SWITCH 30

//LED blueLED(BLUE_LED);


int main(void)
{
//	SCB_PCONP = 0;			// disable all on board peripherals
//	SCB_PCON = 0x01;		// set idle mode
//	SCB_PCON = 0x02;		// set power down mode
//	while(1) { };		// uncomment to test the micro just spinning
	

	processor_speed_init();
	enableIRQ();
	set_up_timer_interrupt();
	set_up_uart();
	init_dynamic_memory();
	
	// get the hardware
	Hardware_Manager* hw= Hardware_Manager::Instance();
	
	if(hw->is_camera_processor())
	{
		LED& l1 = hw->status_LED();
		LED& l2 = hw->activity_LED();
		LED& l3 = hw->error_LED();
		l1.flash(2);
		l2.flash(2);
		l3.flash(2);

		write_line("Hi from Camera   IPC v", get_ipc_protocol_version());
		//ipc_command_processor ipc();
		
		
		while(1)
		{
			ipc_slave_service();
		}
		

	}
	
	Buzzer *bz = hw->buzzer();	
	LED& l1 = hw->status_LED();
	bz->on();
	l1.flash(2);
	bz->off();
	LED& l2 = hw->activity_LED();
	LED& l3 = hw->error_LED();
	l2.flash(2);
	l3.flash(2);
	
	write_line("Hi from Main Controller   IPC v", get_ipc_protocol_version());

	set_replay_buffer(new CharBuffer);			// if we want replaying of data
	
	//IOPort buzzer_pin(MAIN_sound_output_port, true);
	//buzzer_pin.set();
		

	
#define BUZZER_TEST
#ifdef BUZZER_TEST
	Button *b1 = hw->button1();
	bool pressed = false;
#endif
	
	arctan_tests();

	// get parameters for main robot controller
	stored_parameters param;
	hw->set_params(&param);
	
	write_string("Size = ("); write_uint(param.get_map_size_x()); write_string(", "); write_uint(param.get_map_size_y());
	write_string(") Target = ("); write_uint(param.get_map_target_x()); write_string(", "); write_uint(param.get_map_target_y());
	write_string(") Target Size = ("); write_uint(param.get_map_target_size_x()); write_string(", "); write_uint(param.get_map_target_size_y());
	write_line(")");
	write_string("Camera Angle = "); write_uint(param.get_camera_angle_in_tenths_of_a_degree()); write_string("  x-offset = "); write_uint(param.get_x_offset_in_pixels()); write_cr();
	Real_World_Constant_Update(param.get_camera_angle_in_tenths_of_a_degree(), param.get_x_offset_in_pixels());
	write_string("Red level = "); write_uint(param.get_red_level()); write_cr();
	write_line("Blue compare?", param.get_is_blue_compare_on());
	write_line("Current limit?", is_current_limit_on());

//	write_hex(PCB_PINSEL0); write_cr();
//	write_hex(PCB_PINSEL1); write_cr();
//	pin_function_select(MAIN_ipc_tx_port,0);		// select TxD0
//	pin_function_select(MAIN_ipc_rx_port,0);		// select RxD0
//
//	write_hex(PCB_PINSEL0); write_cr();
//	write_hex(PCB_PINSEL1); write_cr();
//	GPIO_IODIR |= (1<<MAIN_ipc_tx_port);		// output
//	bool state = false;
//	while(1)
//	{
//		bool port_state = SIMPLE_GET_INPUT_MACRO(MAIN_ipc_rx_port) ? true : false;
//		if(port_state != state)
//		{
//			state = port_state;
//			if(port_state)
//			{
//				write_line("High");
//			}
//			else
//			{
//				write_line("Low");
//			}
//		}
//		
//		if(key_ready())
//		{
//			byte data = read_char();
//			if(data == '1')
//			{
//				write_string("^");
//				SET_PORT(MAIN_ipc_tx_port);
//			}
//			else if(data == '0')
//			{
//				write_string("v");
//				CLEAR_PORT(MAIN_ipc_tx_port);
//			}
//			
//		}
//			
//	}

//	ipc_serial_driver ipc_serial;
//	//ipc_serial.print_registers();
//	while(1)
//	{
//
//		//ipc_slave_service(&ipc_serial);
//		
//		// bit of test code to transmit keys over link
//		if(key_ready())
//		{
//			ipc_serial.write_byte(read_char());
//		}
//		//ipc.decode_command();
//		
//		if(ipc_serial.byte_incoming_ready())
//		{
//			write_int(ipc_serial.read_byte()); write_string(" ");
//		}
//	}

//	}
	
	Command_line cmd2;
//#define IPC_ECHO_TEST
#ifdef IPC_ECHO_TEST
	Interprocessor_Comms ipc;
	byte current_data = 0;
	int loops=0;
	int errors=0;
#endif
	
	init_motor_direction_pins();
	set_up_motor_photo_interruptors();
	


	while(1) {		
		
#ifdef BUZZER_TEST
		if(b1->pressed() && pressed == false)
		{
			bz->on();
			write_line("Button 1 pressed");
			pressed = true;
		}
		else if(!b1->pressed() && pressed == true)
		{
			write_line("Button 1 released");
			bz->off();
			write_line("Run wall follower");
			SimpleWallFollowerEngine* wf = new SimpleWallFollowerEngine(param.get_red_level());
			wf->run();
			delete wf;
			bz->on(); delay(150); bz->off(); delay(150); bz->on(); delay(150); bz->off();
			pressed = false;
		}
#endif

#ifdef IPC_ECHO_TEST
		if(hw->is_main_processor())
		{
			// send lpc bytes here
			ipc.write_byte(current_data);
			byte data = ipc.read_byte();
			byte expected = 0xff & ~current_data;
			if(data != expected)
			{
				write_string("Sent: "); 
				write_hex(current_data);
				write_string(" Rcv: ");
				write_hex(data);
				write_string(" - wrong! Expected:");
				write_hex(expected);
				write_cr();
				errors++;
			}
			current_data++;
			if(current_data==0)
			{
				loops++;
				write_string("0-255 loops: ");
				write_int(loops);
				write_string(" with ");
				write_int(errors);
				write_string(" errors.");
				write_cr();
			}
		}
		else	// camera
		{
			// echo back characters, but inverted
			byte data = ipc.read_byte();
			write_hex(data);
			ipc.write_byte(~data);
		}
#endif
		cmd2.run();
		
	}
	
//	Button *b1 = hw->button1();
//	Button *b2 = hw->button2();
//	Button *b3 = hw->button3();
//	
//	while(1)
//	{
//		l1.flash(2);
//		if(b1->pressed()) { l1.on(); } else { l1.off(); }
//		if(b2->pressed()) { l2.on(); } else { l2.off(); }
//		if(b3->pressed()) { l3.on(); } else { l3.off(); }
//
//		l2.flash(3);
//
//		if(b1->pressed()) { l1.on(); } else { l1.off(); }
//		if(b2->pressed()) { l2.on(); } else { l2.off(); }
//		if(b3->pressed()) { l3.on(); } else { l3.off(); }
//
//		l3.flash(4);
//		
//		if(b1->pressed()) { l1.on(); } else { l1.off(); }
//		if(b2->pressed()) { l2.on(); } else { l2.off(); }
//		if(b3->pressed()) { l3.on(); } else { l3.off(); }
//	}

//
//	GPIO_IODIR |= (1<<RED_LED);	// define LED-Pin as output
//	GPIO_IODIR |= (1<<YELLOW_LED);	// define LED-Pin as output
//	GPIO_IODIR |= (1<<BLUE_LED);	// define LED-Pin as output
//
//
//	while(1)
//	{
//		if(!SIMPLE_GET_INPUT_MACRO(RED_SWITCH))
//		{
//			SET_PORT(RED_LED);
//		}
//		else
//		{
//			CLEAR_PORT(RED_LED);
//		}
//		if(!SIMPLE_GET_INPUT_MACRO(YELLOW_SWITCH))
//		{
//			SET_PORT(YELLOW_LED);
//		}
//		else
//		{
//			CLEAR_PORT(YELLOW_LED);
//		}
//		if(!SIMPLE_GET_INPUT_MACRO(BLUE_SWITCH))
//		{
//			SET_PORT(BLUE_LED);
//		}
//		else
//		{
//			CLEAR_PORT(BLUE_LED);
//		}
//		
//	}
	
	
	//ledInit();
	init_motor_direction_pins();
	
	
	set_up_motor_photo_interruptors();

	//
	// Flash the motor 1 enable pin LED to show we have booted and are ready to run.
	// 
	//blueLED.flash(2);
	write_line("Hi");
	
//	delay(1000);

//	write_line("Press a key");
//	read_char();
	//write_line("Running motor test...");

	// turn off the motors initially...
	motor_remove_power(2);
	motor_remove_power(1);

	
	while(1)
	{		
		Command_line cmd;
		while(run_main_prog == RT_stop)			// !key_ready())
		{
			cmd.run();
		}
	
		write_line("Run");
		processing_loop();
		write_line("Stop");
	}
	
#if 0
	//
	// set up the motors for test
	//
	MOTOR_1_SELECT_FORWARD(); 
	//MOTOR_2_SELECT_BACKWARD(); 
	//MOTOR_1_SELECT_BACKWARD(); 
	motor_1_full_forward();		

	//for(int j=0; j<1000; j+=10)
	//{
	//	enable_pwm(1000-j, 1000-j);
	//	delay();
	//}
		
	MOTOR_2_SELECT_FORWARD(); 
	motor_2_full_forward();	
	delay(500);
	switch_sensing_off();
	
	// turn off the motors finally...	
	disable_pwm();
	motor_2_remove_power();
	motor_1_remove_power();	

	delay(200);
	print_motor_data();
	
	write_string("Ints=");
	write_int(fetch_timer_ticks());
	write_cr();

	// turn off the motors finally...	
	disable_pwm();
	motor_2_remove_power();
	motor_1_remove_power();	

	write_line("End");
#endif
	
	// at the end of the cycle do a different flash
	while(1) { 
	
//		flash_led(3);
//		ENABLE_LED();
		delay(700);
	}

	return 0;
}



void processing_loop()
{
#if SPEED_ADJUST_WITH_PWM	
	motor_1_full_forward();		
	motor_2_full_forward();		
	enable_pwm(0,0);
#endif
	
//	Speed_adjust s1(1), s2(2);
//	Pos_adjust p1(1,&s1), p2(2,&s2);
	//Speed_adjust2 s1(1), s2(2);			// motor for new version, table for old version
	Pos_adjust2 p;//&s1, &s2);

//	s1.start();
//	s2.start();

	if(run_main_prog == RT_run_straight)
	{
		p.move_straight_with_stop(distance_to_move);
	}
	else if(run_main_prog == RT_turn_left)
	{
		p.turn_on_spot_from_to_stop(turn_angle);
	}
	else if(run_main_prog == RT_turn_right)
	{
		p.turn_on_spot_from_to_stop(-turn_angle);
	}

//	p1.move(400000);
//	p2.move(400000);
	
	//bool reverse = false;
	//int ticks = fetch_timer_ticks();

	Command_line cmd;
	
	int old_timer_ticks=-1;	// this is an unlikely value

#if 0
	int count = 0;
	int count_one_second_ticks = fetch_timer_ticks();
#endif
	
	while(run_main_prog != RT_stop)			// !key_ready())
	{
		cmd.run();

		// first get the position to decide what it's doing
		p.update();
		
		if(p.is_finished())
		{
			run_main_prog = RT_stop;
		}
			
		// next set the speed based on that
		//s1.update();
		//s2.update();
	
		//if(!reverse && (fetch_timer_ticks()-ticks) > 100)
		//{
		//	reverse=true;
		//	s1.stop();
		//	s2.stop();
		//}
	
#if SPEED_ADJUST_WITH_PWM	
		adjust_pwm(s1.get(),s2.get());
#endif
		

#if 0
		//
		// Count the number of loops per second
		//
		count++;
		int current_ticks_x = fetch_timer_ticks();
		if(current_ticks_x >= (count_one_second_ticks+100))
		{
			count_one_second_ticks = current_ticks_x;
			write_int(count); write_string(" "); write_int(current_ticks_x); write_cr();
			count=0;
		}
#endif		
		// 
		// data stream
		//
		int write_data_to_uart = 0;
		
		if(write_data_to_uart)
		{
			int current_ticks = fetch_timer_ticks();
			if(current_ticks != old_timer_ticks)		// every 1 tick (10ms)
			{
#if SPEED_ADJUST_WITH_PWM	
				write_int(s1.get()); write_string(" "); write_int(s2.get()); write_string(" "); 
#endif
				write_int(fetch_timer_ticks()); write_string(" ");
				unsigned int x1 = get_motor_input_count(1);
				unsigned int x2 = get_motor_input_count(2);
				//int x3 = p1.get_remainder();
				//int x4 = p2.get_remainder();
				write_int(convert_to_um(x1)/1000); write_string(" "); write_int(convert_to_um(x2)/1000); write_string(" ");
				
				//write_int(x3/1000);  write_string(" "); write_int(x4/1000);  write_string(" ");

				write_cr(); //write_int(p1.get()); write_cr();
				old_timer_ticks = current_ticks;
			}
		}
	}
	
#if SPEED_ADJUST_WITH_PWM	
	disable_pwm();
#endif
	motor_remove_power(2);
	motor_remove_power(1);	
	
	//read_char();
	//delay(1000);
}



// =============================================================================
// =============================================================================
// =============================================================================
// =============================================================================
#if 0	// C++ Version - currently unused

class LedHandler {
private:
	int ledstate;
public:
	LedHandler() { /* constructor */
		GPIO_IODIR |= (1<<LEDPIN);	// define LED-Pin as output
		GPIO_IOSET = (1<<LEDPIN);	// set Bit = LED off (active low)
		ledstate = 1;
	}
void on() 	{
	GPIO_IOCLR=(1<<LEDPIN);		// clear I/O bit -> LED on (active low)
}
void off() 	{
	GPIO_IOSET=(1<<LEDPIN);		// set I/O bit -> LED off
}
void showstate() {
	if (ledstate) on();
	else off();
}
void setstate(const int newstate) {
	ledstate = newstate;
}
};


#ifdef GLOBALOBJECT
// #warning "using 'global' LedHandler-Object"
LedHandler lh; 
#endif
 
int main(void)
{
	int i;

	#ifndef GLOBALOBJECT
	// #warning "using 'local' LedHandler-Object"
	LedHandler lh; 
	#endif
		
	MAM_MAMCR = 2;	// MAM functions fully enabled
	
	GPIO_IODIR &= ~(1<<SWPIN);	// define Switch-Pin as input
	
	lh.showstate(); // test state init from contructor
	for (i=0;i<15;i++) delay();
	
	i=0;
	while (i<10)	
	{
		lh.on();
		delay();
		lh.off();
		delay();
		i++;
	}
	
	while (1)	
	{
		if (GPIO_IOPIN & (1<<SWPIN))	{ // true if button released (active low)
			lh.setstate(1);
			lh.showstate();
			// lh.on();   // led on if button is released
		}
		else {
			lh.setstate(0);
			lh.showstate();
			//lh.off();  // led off if button is pressed
		}
	}
	
	return 0; // never reached
}

#endif	// C++ Version - currently unused


// number of write cycles for LPC2106 flash? 
// From Philips data:
//   endurance (write and erase) 100,000 - - cycles 
// Rob note: 100 times a day, flash would write 3 years.
//
// Can we write to ram so debug doesn't use write cycles?
//  Leave for the moment, investigate later


