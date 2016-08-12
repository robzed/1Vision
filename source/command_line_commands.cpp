/*
 *  command_line_commands.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 27/02/2007.
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
#include "command_line_commands.h"

#include "pc_uart.h"
#include "led_control.h"
#include "motor_pwm.h"
#include "basic_motor_primitives.h"
#include "motor_sensing.h"
#include "language_support.h"
#include "speed_control.h"
#include "i2c_driver.h"
#include "camera_serial.h"
#include "camera_image_port.h"
#include "timing.h"
#include "xmodem_send.h"
#include "hardware_manager.h"
#include "interprocessor_comms.h"
#include "reverse_perspective_projection.h"
#include "advanced_line_vector.h"
#include "Real_World.h"
#include "stored_parameters.h"
#include "sine_cosine.h"
#include "Zw_calculate.h"
#include "wall_detector.h"
#include "local_map.h"
#include "heading_calculation.h"
#include "arctan.h"
#include "position_calculation.h"
#include "simple_wall_follower_engine.h"
#include "misc_vision_utilities.h"
#include "motor_pwm_control.h"
#include "motor.h"
#include "logger.h"
#include "tick_timer.h"

/*
//
// Command List
//
command_function_t led_command;
command_function_t motor_command;

int motor_1_remove_power_command(int argc, const char *const argv[]);
int motor_2_remove_power_command(int argc, const char *const argv[]);
int motor_1_full_forward_command(int argc, const char *const argv[]);
int motor_2_full_forward_command(int argc, const char *const argv[]);
int motor_1_full_reverse_command(int argc, const char *const argv[]);
int motor_2_full_reverse_command(int argc, const char *const argv[]);
int motor_1_fast_stop_command(int argc, const char *const argv[]);
int motor_2_fast_stop_command(int argc, const char *const argv[]);
// pwm
int enable_pwm_command(int argc, const char *const argv[]);
int adjust_pwm_command(int argc, const char *const argv[]);
int disable_pwm_command(int argc, const char *const argv[]);
//
int m1_input_command(int argc, const char *const argv[]);
int m2_input_command(int argc, const char *const argv[]);

command_function_t printnum_command;
command_function_t run_command;
command_function_t stop_command;
command_function_t dist_command;
command_function_t left_command;
command_function_t right_command;
command_function_t angle_command;
command_function_t avt_command;

command_function_t malloc_command;
command_function_t free_command;
command_function_t memstat_command;
command_function_t total_free_command;
command_function_t largest_free_command;

command_function_t new_obj_command;
command_function_t delete_obj_command;
command_function_t obj_action_command;

command_function_t new_speed_ctrl_command;
command_function_t delete_speed_ctrl_command;
command_function_t speed_ctrl_on_command;
command_function_t speed_ctrl_off_command;

command_function_t new_i2c_command;
command_function_t delete_i2c_command;
command_function_t i2c_status_command;
command_function_t i2c_data_command;
command_function_t i2c_test_command;
command_function_t i2c_test2_command;

command_function_t camser_read_command;
command_function_t camser_write_command;

command_function_t camser_completed_command;
command_function_t pictest_command;
command_function_t piciotest_command;
command_function_t picmeasure_command;
command_function_t picsetparam_command;
command_function_t picdutycycles_command;

command_function_t picget_image_command;
command_function_t test_xmodem_command;
command_function_t ipc_led_command;
command_function_t ipc_device_command;

command_function_t ipc_button_command;
command_function_t ipc_i2c_command;

command_function_t test_command;
//command_function_t ee_read_command;
//command_function_t ee_write_command;
command_function_t init_camera_command;
command_function_t lines_command;
command_function_t multiple_lines_command;
command_function_t plan_command;
command_function_t multiple_plan_command;
command_function_t lines2_command;
*/






//
// led_command
//
int led_command(int argc, const char *const argv[])
{
	if(are_strings_equal(argv[1], "on"))
	{
		Hardware_Manager* hw= Hardware_Manager::Instance();
		hw->status_LED().on();
	}
	else if(are_strings_equal(argv[1], "off"))
	{
		Hardware_Manager::Instance()->status_LED().off();
	}
	else
	{
		write_line("Second parameter should be on or off");
	}
	
	return 0;
}



// sound the buzzer
int beep_command(int argc, const char *const argv[])
{
	
	if(argc == 2)
	{
		int length;
		if(are_strings_equal(argv[1], "on"))
		{
			Hardware_Manager* hw= Hardware_Manager::Instance();
			hw->buzzer()->on();
		}
		else if(are_strings_equal(argv[1], "off"))
		{
			Hardware_Manager::Instance()->buzzer()->off();
		}
		else if(are_strings_equal(argv[1], "music"))
		{
			write_line("Frere Gustav");
			Buzzer* b = Hardware_Manager::Instance()->buzzer();
			b->beep(100,0); b->beep(100,2); b->beep(50,3); b->beep(50,2); b->beep(100,0);
			b->beep(100,0); b->beep(100,2); b->beep(50,3); b->beep(50,2); b->beep(100,0);
			b->beep(100,3); b->beep(100,5); b->beep(200,7);
			b->beep(100,3); b->beep(100,5); b->beep(200,7);
			b->beep(75,7); b->beep(25,8); b->beep(50,7); b->beep(50,5); b->beep(50,3); b->beep(50,2); b->beep(100,0);
			b->beep(75,7); b->beep(25,8); b->beep(50,7); b->beep(50,5); b->beep(50,3); b->beep(50,2); b->beep(100,0);
			b->beep(100,0); b->beep(100,-5); b->beep(200,0);
			b->beep(100,0); b->beep(100,-5); b->beep(200,0);
		}
		else if(are_strings_equal(argv[1], "m2"))
		{
			write_line("Frere Gustav, fast");
			Buzzer* b = Hardware_Manager::Instance()->buzzer();
			int o = 24;
			b->beep(50,0+o); b->beep(50,2+o); b->beep(25,3+o); b->beep(25,2+o); b->beep(50,0+o);
			b->beep(50,0+o); b->beep(50,2+o); b->beep(25,3+o); b->beep(25,2+o); b->beep(50,0+o);
			b->beep(50,3+o); b->beep(50,5+o); b->beep(100,7+o);
			b->beep(50,3+o); b->beep(50,5+o); b->beep(100,7+o);
			b->beep(37,7+o); b->beep(13,8+o); b->beep(25,7+o); b->beep(25,5+o); b->beep(25,3+o); b->beep(25,2+o); b->beep(50,0+o);
			b->beep(37,7+o); b->beep(13,8+o); b->beep(25,7+o); b->beep(25,5+o); b->beep(25,3+o); b->beep(25,2+o); b->beep(50,0+o);
			b->beep(50,0+o); b->beep(50,-5+o); b->beep(100,0+o);
			b->beep(50,0+o); b->beep(50,-5+o); b->beep(100,0+o);
		}
		else if(are_strings_equal(argv[1], "zing"))
		{
			Buzzer* b = Hardware_Manager::Instance()->buzzer();
			for(int i=20; i<40; i++)
			{
				b->beep(5,i); 
			}
		}
		else if(convert_val(argv[1], &length, "length"))
		{
			Hardware_Manager::Instance()->buzzer()->beep(length);
		}
	}
	else
	{
		int length;
		int pitch;
		if(convert_val(argv[1], &length, "length") and convert_val(argv[2], &pitch, "pitch"))
		{
			Hardware_Manager::Instance()->buzzer()->beep(length, pitch);
		}
	}
	
	return 0;
}


//
// Motor control commands
//
int motor_1_remove_power_command(int argc, const char *const argv[])
{
	//motor_1_remove_power();
	motor_remove_power(1);
	return 0;
}

int motor_2_remove_power_command(int argc, const char *const argv[])
{
	//motor_2_remove_power();
	motor_remove_power(2);
	return 0;
}

int motor_1_full_forward_command(int argc, const char *const argv[])
{
	//motor_1_full_forward();	
	motor_full(1, true);
	return 0;
}

int motor_2_full_forward_command(int argc, const char *const argv[])
{
	//motor_2_full_forward();	
	motor_full(2, true);
	return 0;
}

int motor_1_full_reverse_command(int argc, const char *const argv[])
{
	//motor_1_full_reverse();
	motor_full(1, false);
	return 0;
}

int motor_2_full_reverse_command(int argc, const char *const argv[])
{
	//motor_2_full_reverse();
	motor_full(2, false);
	return 0;
}

int motor_1_fast_stop_command(int argc, const char *const argv[])
{
	//motor_1_fast_stop();
	write_line("Commented out");
	return 0;
}

int motor_2_fast_stop_command(int argc, const char *const argv[])
{
	//motor_2_fast_stop();		
	write_line("Commented out");
	return 0;
}


int enable_pwm_command(int argc, const char *const argv[])
{
	int m1 =0; int m2 =0;
	bool result1 = convert_val(argv[1], &m1);
	bool result2 = convert_val(argv[2], &m2);
	
	if(result1 && result2)
	{
		enable_pwm(m1, m2);
	}
	
	return 0;
}

int adjust_pwm_command(int argc, const char *const argv[])
{
	int m1 =0; int m2 =0;
	bool result1 = convert_val(argv[1], &m1);
	bool result2 = convert_val(argv[2], &m2);
	
	if(result1 && result2)
	{
		adjust_pwm(m1, m2);
	}
	return 0;
}

int disable_pwm_command(int argc, const char *const argv[])
{
	disable_pwm();
	return 0;
}

int set_single_pwm_command(int argc, const char *const argv[])
{
	int motor =0; int level =0;
	bool result1 = convert_val(argv[1], &motor);
	bool result2 = convert_val(argv[2], &level);
	
	if(result1 && result2)
	{
		set_single_pwm(motor, level);
	}
	
	return 0;
}
int disable_single_pwm_command(int argc, const char *const argv[])
{
	int motor = 0;
	bool result1 = convert_val(argv[1], &motor);
	
	if(result1)
	{
		disable_single_pwm(motor);
	}
	return 0;
}

int m1_input_command(int argc, const char *const argv[])
{
	unsigned int count = get_motor_input_count(1);
	write_int(count);
	write_cr();
	return 0;
}

int m2_input_command(int argc, const char *const argv[])
{
	unsigned int count = get_motor_input_count(2);
	write_int(count);
	write_cr();
	return 0;
}

int printnum_command(int argc, const char *const argv[])
{
	int num;
	if(convert_val(argv[1], &num))
	{
		write_int(num);
		write_cr();
	}
	else
	{
		write_line("Invalid number");
	}
	
	return 0;
}

//
// Include main program controls directly. 
// Poor programming, but minor.
//
void run();
void stop();

int run_command(int argc, const char *const argv[])
{
	run();
	return 0;
}

int stop_command(int argc, const char *const argv[])
{
	stop();
	return 0;
}

void right();
void left();

int right_command(int argc, const char *const argv[])
{
	right();
	return 0;
}

int left_command(int argc, const char *const argv[])
{
	left();
	return 0;
}

//
//
void dist(int distance);

int dist_command(int argc, const char *const argv[])
{
	int num;
	if(convert_val(argv[1], &num, "number"))
	{
		dist(num*1000);
	}	
	return 0;
}

void angle(int the_angle);

int angle_command(int argc, const char *const argv[])
{
	int num;
	if(convert_val(argv[1], &num, "number"))
	{
		angle(num);
	}
	return 0;
}




int avt_command(int argc, const char *const argv[])
{
	int motor;
	if(convert_val(argv[1], &motor))
	{
		write_int(get_average_time(motor));
		write_cr();
	}
	else
	{
		write_line("Invalid motor");
	}
	
	return 0;
}



// ********************************************************************
//
// Memory commands for memory system debugging
//
int malloc_command(int argc, const char *const argv[])
{
	int size;
	if(convert_val(argv[1], &size, "size"))
	{
		write_int(reinterpret_cast<unsigned int>(malloc(size)));
		write_cr();
	}
	
	return 0;
}

int free_command(int argc, const char *const argv[])
{
	int ptr;
	if(convert_val(argv[1], &ptr, "pointer"))
	{
		free(reinterpret_cast<void*>(ptr));
	}
	
	return 0;	
}

int memstat_command(int argc, const char *const argv[])
{
	write_memstat();
	//memstat_data_t d;
	//memstat(&d);
	
	//	write_string("total="); write_int(d.total_mem); write_cr();
	//	write_string("available="); write_int(d.space_available); write_cr();
	//	write_string("blocks alloc="); write_int(d.num_blocks_allocated); write_cr();
	//	write_string("largest unalloc="); write_int(d.largest_unallocated); write_cr();
	//	write_string("gap="); write_int(d.number_of_gaps); write_cr();
	//	write_string("ram start=0x"); write_hex(d.ram_start); write_cr();
	//	write_string("ram end=0x"); write_hex(d.ram_end); write_cr();
	//	write_string("free store start=0x"); write_hex(d.free_store_start); write_cr();
	//	write_string("free store end=0x"); write_hex(d.free_store_end); write_cr();
	return 0;
}

int total_free_command(int argc, const char *const argv[])
{
	write_int(total_free()); write_cr();
	return 0;
}

int largest_free_command(int argc, const char *const argv[])
{
	write_int(largest_free()); write_cr();
	return 0;
}

// ********************************************************************
//
// new and delete and constructor and destructor tests
//
class command_line_test_class {
public:
	command_line_test_class();
	~command_line_test_class();
	void action();
private:
	int data;
};

command_line_test_class::command_line_test_class()
:data(0)
{
	write_line("OBJECT: Inside Constructor");
}

command_line_test_class::~command_line_test_class()
{
	write_line("OBJECT: Inside Destructor");
}

void command_line_test_class::action()
{
	write_string("OBJECT: test value = ");
	write_int(data);
	write_cr();
	++data;
}


command_line_test_class* testobj;

int new_obj_command(int argc, const char *const argv[])
{
	testobj = new command_line_test_class;
	write_string("CALLER: Object address = 0x"); write_hex(reinterpret_cast<unsigned int>(testobj)); write_cr();
	return 0;
}

int delete_obj_command(int argc, const char *const argv[])
{
	delete testobj;
	write_line("CALLER: Object delete, call new_obj before using again");
	return 0;
}

int obj_action_command(int argc, const char *const argv[])
{
	testobj->action();
	return 0;
}

// ********************************************************************
//
// speed control test commands
//

Speed_Control* sc_test_obj = 0;
Motor* motor_obj = 0;

int new_speed_ctrl_command(int argc, const char *const argv[])
{
	int motor;
	if(convert_val(argv[1], &motor, "motor"))
	{
		
#if USE_MOTOR_CLASS
		motor_obj = new Motor(motor);
		sc_test_obj = new Speed_Control(motor, *motor_obj);
#else
		sc_test_obj = new Speed_Control(motor);
#endif
	}
	return 0;
}

int delete_speed_ctrl_command(int argc, const char *const argv[])
{
	delete sc_test_obj;
	delete motor_obj;
	sc_test_obj = 0;
	motor_obj = 0;
	return 0;
}


int speed_ctrl_off_command(int argc, const char *const argv[])
{
	if(!sc_test_obj)
	{
		write_line("No speed obj");
	}
	sc_test_obj->stop_speed_control();	
	return 0;
}

int speed_ctrl_on_command(int argc, const char *const argv[])
{
	if(!sc_test_obj)
	{
		write_line("No speed obj");
	}
	int mm_per_sec;
	if(convert_val(argv[1], &mm_per_sec, "speed"))
	{
		sc_test_obj->start_speed_control(mm_per_sec
#if !USE_MOTOR_CLASS
										 ,true
#endif
		);
	}
	
	return 0;
}

int speed_ctrl_command(int argc, const char *const argv[])
{
	// create
#if USE_MOTOR_CLASS
	Motor* motor_obj1 = new Motor(1);
	Speed_Control* sc_test_obj1 = new Speed_Control(1, *motor_obj1);
	Motor* motor_obj2 = new Motor(2);
	Speed_Control* sc_test_obj2 = new Speed_Control(2, *motor_obj2);
#else
	Speed_Control* sc_test_obj1 = new Speed_Control(1);
	Speed_Control* sc_test_obj2 = new Speed_Control(2);
#endif
	
	// tell motors to go forward...
	
	int mm_per_sec;
	if(convert_val(argv[1], &mm_per_sec, "speed"))
	{
#if USE_MOTOR_CLASS
		motor_obj1->set_direction(true);
		motor_obj2->set_direction(true);
#endif
		sc_test_obj1->start_speed_control(mm_per_sec
#if !USE_MOTOR_CLASS
										 ,true
#endif
										 );
		sc_test_obj2->start_speed_control(mm_per_sec
#if !USE_MOTOR_CLASS
										  ,true
#endif
										  );
		
		//
		// Stop once travelled 5 squares
		//
		int initial = ((convert_to_um(get_motor_input_count(1))+convert_to_um(get_motor_input_count(1)))/2);
		int target =  initial + 180*4*1000;		// number of squares in um

		while(((convert_to_um(get_motor_input_count(1))+convert_to_um(get_motor_input_count(1)))/2) < target)
		{
			// wait
			sc_test_obj1->update_motor_speed_adjust();
			sc_test_obj2->update_motor_speed_adjust();
			if(key_ready()) break;
		}
		
		sc_test_obj1->stop_speed_control();	
		sc_test_obj2->stop_speed_control();	
		
#if USE_MOTOR_CLASS
		motor_obj1->slow_down();
		motor_obj2->slow_down();
#endif
		
		while(!robot_stopped())
		{
			// wait...
			if(key_ready()) break;
		}
		if(key_ready()) read_char();	// throw away any character
		write_line("Distance travelled = ", ((convert_to_um(get_motor_input_count(1))+convert_to_um(get_motor_input_count(1)))/2)-initial);
		
	}

	delete sc_test_obj1;
	delete sc_test_obj2;
#if USE_MOTOR_CLASS
	delete motor_obj2;
	delete motor_obj1;
#endif
	
	return 0;
}
// ********************************************************************
//
// camera serial test commands
//

camera_serial* cam_ser_obj = 0;

static bool camser_common_checks()
{
	if(!cam_ser_obj) { cam_ser_obj = new camera_serial; }
	
	if(!cam_ser_obj->completed()) { 
		write_line("Previous operation not completed"); 
		return true;
	} 	
	
	return false;
}


int camser_write_command(int argc, const char *const argv[])
{
	if(camser_common_checks()) return 0;
	
	int reg;
	if(!convert_val(argv[1], &reg, "register")) { return 1; }
	
	const int max_data_params = 10;
	const int data_params_offset = 2;	// two are: command and register number
	int num_data_params = argc - data_params_offset;
	if(num_data_params > max_data_params)
	{
		write_line("Too much data");
		return 1;
	}
	byte data[max_data_params];
	for(int i=0; i < num_data_params; i++)		// plus 1 is register
	{
		int x;
		if(!convert_val(argv[i+data_params_offset], &x, "data")) { return 1; }
		data[i] = x;
	}
	
	cam_ser_obj->raw_write(reg, data, num_data_params);
	
	return 0;
}


int camser_read_command(int argc, const char *const argv[])
{
	if(camser_common_checks()) return 0;
	
	int reg, num_bytes;
	if(convert_val(argv[1], &reg, "number") && convert_val(argv[2], &num_bytes, "number"))
	{
		byte *data;
		data = new unsigned char[num_bytes];
		
		cam_ser_obj->raw_read(reg, data, num_bytes);
		
		while(!cam_ser_obj->completed())
		{
			cam_ser_obj->worker_function();
		}
		
		write_string("data:");
		for(int i=0; i<num_bytes; i++)
		{
			write_int(data[i]); //write_string(" ");
			write_cr();
		}
		write_cr();
		
		delete[] data;
	}
	
	return 0;
	
}


int camser_completed_command(int argc, const char *const argv[])
{
	if(camser_common_checks()) return 0;
	write_line("Finished");
	
	return 0;
}

// ********************************************************************
//
// Command line called periodic routines
//
void ipc_led_command_service();

void support_periodic_routines()
{
	if(sc_test_obj) sc_test_obj->update_motor_speed_adjust();
	if(cam_ser_obj) cam_ser_obj->worker_function();
	ipc_led_command_service();
	Hardware_Manager::Instance()->buzzer()->service();
}

// ********************************************************************
//
// I2C driver tests
//
i2c_driver* i2c_test_obj = 0;

int new_i2c_command(int argc, const char *const argv[])
{
	i2c_test_obj = new i2c_driver;
	write_line("new i2c driver");
	return 0;
}

int delete_i2c_command(int argc, const char *const argv[])
{
	delete i2c_test_obj;
	i2c_test_obj = 0;
	write_line("deleted i2c driver");
	return 0;
}

int i2c_status_command(int argc, const char *const argv[])
{
	if(!i2c_test_obj)
	{
		new_i2c_command(1,0);
	}
	
	i2c_test_obj->print_reg_status();
	
	if(argc != 1)
	{
		if(argc != 5)
		{
			write_line("Need 4 parameters AA SI STO STA");
		}
		else
		{
			int v1, v2, v3, v4;
			if(convert_val(argv[1], &v1, "AA") &&
			   convert_val(argv[2], &v2, "SI") &&
			   convert_val(argv[3], &v3, "STO") &&
			   convert_val(argv[4], &v4, "STA"))
			{
				i2c_test_obj->set_control_reg(v1, v2, v3, v4);
				i2c_test_obj->print_reg_status();
			}
			else
			{
				write_line("Ignoring parameters");
			}
		}
	}
	return 0;
}



int i2c_data_command(int argc, const char *const argv[])
{
	if(!i2c_test_obj)
	{
		new_i2c_command(1,0);
	}
	
	if( ! i2c_test_obj->get_serial_interrupt_flag())
	{
		write_line("SI clear - no data access");
	}
	else if(argc == 1)
	{
		write_int(i2c_test_obj->get_data_reg());
	}
	else // argc must be 2
	{
		int val;
		if(convert_val(argv[1], &val, "data byte"))
		{
			i2c_test_obj->set_data_reg(val);
		}
		
	}
	
	return 0;
}

//
// should have timer in here really...
//
void wait_for_i2c_serial_interrupt()
{
	if( ! i2c_test_obj->get_serial_interrupt_flag())
	{
		write_string("Waiting for SI ... ");
		while(!  i2c_test_obj->get_serial_interrupt_flag())
		{
			// do nothing
		}
	}
	
}


unsigned char test_data[] = { 0x07 };
unsigned char data_in[3];

int i2c_test2_command(int argc, const char *const argv[])
{
	if(!i2c_test_obj)
	{
		new_i2c_command(1,0);
	}
	
	write_line("Starting test ... write reg 7");
	i2c_test_obj->send_data(test_data, 1);
	while(!i2c_test_obj->completed())
	{
		i2c_test_obj->service_i2c();
	}
	write_string("Error = "); write_int(i2c_test_obj->error()); write_cr();
	
	if(! (i2c_test_obj->error()))
	{
		write_line("Reading...");
		i2c_test_obj->rx_data(data_in, 3);
		while(!i2c_test_obj->completed())
		{
			i2c_test_obj->service_i2c();
		}
		write_string("Error = "); write_int(i2c_test_obj->error()); write_cr();
		write_string("Data = "); write_hex(data_in[0]);	write_cr();
		write_string("Data = "); write_hex(data_in[1]);	write_cr();
		write_string("Data = "); write_hex(data_in[2]);	write_cr();
	}
	return 0;
}

int i2c_test_command(int argc, const char *const argv[])
{
	if(!i2c_test_obj)
	{
		new_i2c_command(1,0);
	}
	
	// check for status = f8
	
	//
	// start the i2c
	//
	i2c_test_obj->print_reg_status();
	write_line("STARTING I2C MASTER");
	i2c_test_obj->set_control_reg(0, 0, 0, 1);	// AA SI STO STA
	wait_for_i2c_serial_interrupt();
	i2c_test_obj->print_reg_status();
	// check for status = 8
	
	//
	// now write address out
	//
	const int camera_write_address = 0x42;
	const int camera_read_address = 0x43;
	write_line("WRITING CAMERA ADDRESS");
	i2c_test_obj->set_data_reg(camera_write_address);
	i2c_test_obj->set_control_reg(0, 0, 0, 0);	// AA SI STO STA
	wait_for_i2c_serial_interrupt();
	i2c_test_obj->print_reg_status();
	// check for status = 18
	
	// select analogue sharpness control
	write_line("WRITING REGISTER ADDRESS");
	i2c_test_obj->set_data_reg(0x07);
	i2c_test_obj->set_control_reg(0, 0, 0, 0);	// AA SI STO STA
	wait_for_i2c_serial_interrupt();
	i2c_test_obj->print_reg_status();
	// check for status = 28
	
	//
	// Now write stop
	//
	write_line("STOPPING I2C");
	i2c_test_obj->set_control_reg(0, 0, 1, 0);	// AA SI STO STA
	i2c_test_obj->print_reg_status();
	// check for stop then status = f8
	
	
	// restart doesn't work for some reason
#if 0
	//
	// Now write restart, which turns mode into read mode...
	//
	//write_line("RESTARTING I2C");
	//i2c_test_obj->set_control_reg(0, 0, 0, 1);	// AA SI STO STA
	//wait_for_i2c_serial_interrupt();
	//i2c_test_obj->print_reg_status();
	// check for status = 10
#endif 
	
	// check for status = f8
	
	//
	// start the i2c
	//
	i2c_test_obj->print_reg_status();
	write_line("STARTING I2C MASTER");
	i2c_test_obj->set_control_reg(0, 0, 0, 1);	// AA SI STO STA
	wait_for_i2c_serial_interrupt();
	i2c_test_obj->print_reg_status();
	// check for status = 8
	
	//
	// now write address out
	//
	write_line("WRITING CAMERA ADDRESS + READ");
	i2c_test_obj->set_data_reg(camera_read_address);
	i2c_test_obj->set_control_reg(0, 0, 0, 0);	// AA SI STO STA
	wait_for_i2c_serial_interrupt();
	i2c_test_obj->print_reg_status();
	// check for status = 40
	
	write_line("READING REGISTER");
	i2c_test_obj->set_control_reg(1, 0, 0, 0);	// AA SI STO STA
	wait_for_i2c_serial_interrupt();
	i2c_test_obj->print_reg_status();
	write_string("data = 0x"); write_hex(i2c_test_obj->get_data_reg()); write_cr();
	// check for status = 50
	
	//
	// Now write stop
	//
	write_line("STOPPING I2C for second time");
	i2c_test_obj->set_control_reg(0, 0, 1, 0);	// AA SI STO STA
	i2c_test_obj->print_reg_status();
	// check for stop then status = f8
	return 0;
}


//
//
//
int picdutycycles_command(int argc, const char *const argv[])
{
	camera_image_port cam;
	
	cam.dutycycles();
	
	return 0;
}

//
//
//
int pictest_command(int argc, const char *const argv[])
{
	camera_image_port cam;	
	cam.test();
	
	return 0;
}



//
//
//
int picmeasure_command(int argc, const char *const argv[])
{
	camera_image_port cam;
	
	cam.measure();
	
	return 0;
}


//
//
//
int piciotest_command(int argc, const char *const argv[])
{
	camera_image_port cam;
	
	cam.iotest();
	
	return 0;
}

int picsetparam_command(int argc, const char *const argv[])
{
	camera_image_port cam;
	
	int clock;
	int mode8;
	
	if(convert_val(argv[1], &clock, "clock") && convert_val(argv[2], &mode8, "mode"))
	{
		bool mode8_bool = false;;
		if(mode8) mode8_bool = true;
		cam.setparam(clock, mode8_bool);
	}
	
	return 0;
}



int test_xmodem_command(int argc, const char *const argv[])
{
	const int width = 1280;
	const int lines_to_store = 40;
	const int total_lines = 240;
	
	byte* data = new byte[width*lines_to_store+1];
	XModem_Send tx;
	//
	// Do Xmodem binary download ... timed at 34 seconds (a whole 30 seconds faster than above method)
	//
	write_line("Start Xmodem download **NOW**");
	if(true == tx.send_data(data, lines_to_store*width))
	{
		bool failed = false;
		for(int i=lines_to_store; i<total_lines; i+=lines_to_store)
		{
			if(false == tx.send_data(data, lines_to_store*width))
			{
				failed = true;
				break;
			}
		}
		if(!failed)
		{
			failed = ! (tx.finish_transfer());
		}
		// wait for timer to expire - so to avoid text getting eaten by the reciever
		Timer t(1,0);
		while(!t.expired()) { }
		if(failed)
		{
			write_line("*** Xmodem download ABORTED ***");
		}
		else
		{
			write_line("Finished Xmodem download");
		}
	}
	
	
	delete[] data; 
	return 0;
}

IPC_led_command *cmd_ipc_led_ptr;
ipc_serial_driver *cmd_ipc_serial;

int ipc_led_command(int argc, const char *const argv[])
{
	int led;
	int state;
	
	if(convert_val(argv[1], &led, "led") && convert_val(argv[2], &state, "state"))
	{		
		if(cmd_ipc_led_ptr==0)
		{
			cmd_ipc_serial = new ipc_serial_driver;
			cmd_ipc_led_ptr = new IPC_led_command(cmd_ipc_serial, led, state);
		}
		else
		{
			write_line("ipc LED already!");
		}
	}
	
	return 0;
}

void ipc_led_command_service()
{
	if(cmd_ipc_led_ptr)
	{		
		cmd_ipc_led_ptr->service();
		if(cmd_ipc_led_ptr->completed())
		{
			write_string("ipc LED completed (");
			if(cmd_ipc_led_ptr->command_failed())
			{
				write_line("failed)");
			}
			else
			{
				write_line("succeeded)");
			}
			delete cmd_ipc_led_ptr;
			cmd_ipc_led_ptr = 0;
			delete cmd_ipc_serial;
			cmd_ipc_serial = 0;
		}
	}
}


int ipc_device_command(int argc, const char *const argv[])
{
	ipc_serial_driver ipc_serial;
	IPC_device_data_command ipc(&ipc_serial);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("Oh no command failed");
	}
	else
	{
		write_line("Camera Version = ", ipc.get_version());
	}
	
	
	return 0;
}

int ipc_button_command(int argc, const char *const argv[])
{
	ipc_serial_driver ipc_serial;
	IPC_button_command ipc(&ipc_serial);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("Oh no command failed");
	}
	else
	{
		write_line("B=", ipc.button_status());
	}
	
	
	return 0;
}


int ipc_i2c_command(int argc, const char *const argv[])
{
	int address;
	if(! convert_val(argv[1], &address, "addr"))
	{
		return 0;
	}
	
	ipc_serial_driver ipc_serial;
	if(argc==3)
	{	
		int data;
		if(! convert_val(argv[2], &data, "data"))
		{
			return 0;
		}
		
		IPC_i2c_write ipc(&ipc_serial, address, data);
		
		while(!ipc.completed())
		{
			ipc.service();
		}
		if(ipc.command_failed())
		{
			write_line("Oh no command failed");
		}
		else
		{
			write_line("Success = ", ipc.write_success());
		}
	}
	else
	{
		IPC_i2c_read ipc(&ipc_serial, address);
		
		while(!ipc.completed())
		{
			ipc.service();
		}
		if(ipc.command_failed())
		{
			write_line("Oh no command failed");
		}
		else
		{
			write_line("Data = ", ipc.data());
		}
	}
	
	return 0;
}



void test_read(ipc_serial_driver* ipc_serial, byte address)
{
	IPC_i2c_read ipc(ipc_serial, address);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("Oh no command failed");
	}
	else
	{
		write_string("Addr = ");
		write_hex(address);
		write_string(" Data = ");
		write_hex(ipc.data());
		write_cr();
	}
}

void test_write(ipc_serial_driver* ipc_serial, byte address, byte data)
{
	IPC_i2c_write ipc(ipc_serial, address, data);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("Oh no command failed");
	}
	else
	{
		write_line("Success = ", ipc.write_success());
	}
}

int test_command(int argc, const char *const argv[])
{
	DISABLE_MOTOR_2(); 
	MOTOR_2_SELECT_BACKWARD(); 
	ENABLE_MOTOR_2();
	
	
#if 0
	Logger &l = *new Logger;
	
	for(int i=0; i<31002; i++)
	{
		l.log(i);
	}
	l.print();
	
	delete &l;
#endif
	
#if 0
	ipc_serial_driver ipc_serial;
	
	//test_write(&ipc_serial, 0, 0);
	
	for(int i=0; i<0x7d; i++)
	{
		test_read(&ipc_serial, i);
	}
	/*
	 test_read(&ipc_serial, 0);
	 test_read(&ipc_serial, 1);
	 test_read(&ipc_serial, 2);
	 test_read(&ipc_serial, 0x11);
	 test_read(&ipc_serial, 0x17);
	 test_read(&ipc_serial, 0x18);
	 test_read(&ipc_serial, 0x19);
	 test_read(&ipc_serial, 0x1a);
	 test_read(&ipc_serial, 0x1b);
	 test_read(&ipc_serial, 0x1c);
	 test_read(&ipc_serial, 0x1d);
	 test_read(&ipc_serial, 0x20);
	 test_read(&ipc_serial, 0x21);
	 test_read(&ipc_serial, 0x22);
	 test_read(&ipc_serial, 0x23);
	 test_read(&ipc_serial, 0x24);
	 test_read(&ipc_serial, 0x25);
	 */
#endif
	return 0;
}


//
//
//
int picget_image_command(int argc, const char *const argv[])
{
	//camera_image_port cam;
	//
	int type;
	if(!convert_val(argv[1], &type, "type")) { return 1; }
	
	int level;
	if(argc==3)
	{
		if(!convert_val(argv[2], &level, "level")) { return 1; }
	}
	else
	{
		level = 75;
	}
	//
	//cam.get_image(type);
	
	ipc_serial_driver ipc_serial;
	IPC_spool_image_out ipc(&ipc_serial, static_cast<byte>(type), static_cast<byte>(level));
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("cmd fail");
	}
	
	return 0;
}


//
//
//
int init_camera_command(int argc, const char *const argv[])
{
	ipc_serial_driver ipc_serial;
	IPC_init_camera ipc(&ipc_serial);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("cmd fail");
	}
	else
	{
		//write_line("completed");
	}
	return 0;
}



void write_lines(line_list_custom_vector& lines1, line_list_custom_vector& lines2, int* longest_lines)
{
	
	uint16_t number = lines1.size();
	write_line("#1 ", number);
	for(int i = 0; i < number; i++)
	{
		write_string("# ");
		write_int(lines1[i].start.x);
		write_string(",");
		write_int(lines1[i].start.y);
		write_string(",");
		write_int(lines1[i].end.x);
		write_string(",");
		write_int(lines1[i].end.y);
		write_line("");
	}
	number = lines2.size();
	write_line("#2 ", number);
	for(int i = 0; i < number; i++)
	{
		write_string("# ");
		write_int(lines2[i].start.x);
		write_string(",");
		write_int(lines2[i].start.y);
		write_string(",");
		write_int(lines2[i].end.x);
		write_string(",");
		write_int(lines2[i].end.y);
		write_line("");
	}
	write_line("#3 ", 3);
	for(int i=0; i<3; i++)
	{
		write_line("# ", longest_lines[i]);
	}
	write_line("#end");
	
}

void write_lines(advanced_line_vector& lines, int* longest_lines)
{
	
	uint16_t number = lines.size();
	write_line("#1 ", number);
	for(int i = 0; i < number; i++)
	{
		write_string("# ");
		write_int(lines[i].start_x);
		write_string(",");
		write_int(lines[i].start_y);
		write_string(",");
		write_int(lines[i].end_x);
		write_string(",");
		write_int(lines[i].end_y);
		write_cr();
		//write_string(" >> ");
		//write_int(lines[i].start_wx);
		//write_string(",");
		//write_int(lines[i].start_wz);
		//write_string("  ");
		//write_int(lines[i].end_wx);
		//write_string(",");
		//write_int(lines[i].end_wz);
		
		//write_line(" ",lines[i].out_of_bounds);
	}
	write_line("#2 ", 0);
	// no class 2 elements
	write_line("#3 ", 3);
	for(int i=0; i<3; i++)
	{
		write_line("# ", longest_lines[i]);
	}
	write_line("#end");
	
}

void write_lines_plan(advanced_line_vector& lines, int *longest_lines)
{	
	uint16_t number = lines.size();
	write_line("#a ", number);
	for(int i = 0; i < number; i++)
	{
		write_string("# ");
		write_int(lines[i].start_wx);
		write_string(",");
		write_int(lines[i].start_wz);
		write_string(",");
		write_int(lines[i].end_wx);
		write_string(",");
		write_int(lines[i].end_wz);
		write_cr();
	}
	write_line("#b 0");
	
	write_line("#c ", 3);
	for(int i=0; i<3; i++)
	{
		write_line("# ", longest_lines[i]);
	}
	write_line("#end");
	
}


void lines_common(int level)
{
	ipc_serial_driver ipc_serial;
	line_list_custom_vector& lines1 = *new line_list_custom_vector;
	line_list_custom_vector& lines2 = *new line_list_custom_vector;
	IPC_get_lines ipc(&ipc_serial, static_cast<byte>(level), static_cast<byte>(get_blue_compare()), lines1, lines2);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("cmd fail");
	}
	else
	{
		write_line("completed");
		int longest_indexes[3] = {-1, -1, -1};
		// do this on the 2d data to determine the biggest resolution data
		//longest_three(lines1, lines2, longest_indexes);
		longest_indexes[0] = longest_vertical(lines1, lines2);
		write_lines(lines1, lines2, longest_indexes);
	}
	delete& lines1;
	delete& lines2;
}


int find_red_level(int argc, const char *const argv[])
{
	stored_parameters* stored = 0;
	Hardware_Manager* hw= Hardware_Manager::Instance();
	stored = hw->get_params();

	int level = 50;		// some default
	if(stored)
	{
		level = stored->get_red_level();
		//write_line("Got parameter from EEPROM ... level=", level);
	}
	else
	{
		//write_line("No parameter from EEPROM ... default level=", level);
	}
	
	if(argc == 2) 
	{
		int level_from_param;
		if(!convert_val(argv[1], &level_from_param, "level")) { return level; }
		
		level = level_from_param;
		//write_line("Got parameter from command .. level=", level);
		// we have the level from the parameter, use this...
		if(stored)
		{
			stored->set_red_level(level);
			//write_line("Stored parameter in EEPROM .. level=", level);			
		}
	}
	
	return level;
}
//
//
//
int lines_command(int argc, const char *const argv[])
{
	int level = find_red_level(argc, argv);
	
	lines_common(level);
	
	return 0;
}


int multiple_lines_command(int argc, const char *const argv[])
{
	int level = find_red_level(argc, argv);
	
	while(!key_ready()) {
		
		lines_common(level);
		delay(100);
	}
	
	read_char();		// get rid of the stop character from the buffer
	return 0;
}





void lines2_common(int level, bool plan_view, bool rotated_plan_view)
{
	ipc_serial_driver ipc_serial;
	line_list_custom_vector& lines1 = *new line_list_custom_vector;
	line_list_custom_vector& lines2 = *new line_list_custom_vector;
	IPC_get_lines ipc(&ipc_serial, static_cast<byte>(level), static_cast<byte>(get_blue_compare()), lines1, lines2);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("cmd fail");
	}
	else
	{
		//write_line("completed");
		advanced_line_vector& lines = *new advanced_line_vector();
		vvappend(lines, lines1);
		vvappend(lines, lines2);
		
		// convert to 3d
		make_lines_plan_view(lines);
		
		// compare lines 60 vs. lines 6
		//...now we need to invalidate lines that are outside the map as we see it
		// do we need to clip lines that pass outside the scene?
		// for now we just:
		// (a) reject lines where both points are beyond 5 cells forward. We probably 
		// can't see this far anyway. This is in the untranslated z direction.
		// (b) reject lines that are off the map in the left-right direction beyond
		// the map edge. We don't do this yet.
		reject_lines(lines);
		
		int longest_indexes[3] = {-1, -1, -1};
		// do this on the 2d data to determine the biggest resolution data
		two_longest_vertical(lines, longest_indexes);
		//longest_indexes[0] = longest_vertical(lines);
		//longest_three(lines, longest_indexes);

		//write_memstat();
		advanced_line_vector& rotated_lines = *new advanced_line_vector();
		//write_memstat();
		
		if(longest_indexes[0] != -1)
		{
			int raw_angle = get_heading(lines, longest_indexes[0], longest_indexes[1]);
			write_string("r= ", raw_angle);
			write_line(" ang=", trig_angle_to_degrees(raw_angle));

			rotate_plan_view(rotated_lines, lines, raw_angle);
		}
		
		//get_position(lines, longest_indexes);
		
		if(plan_view)
		{
			if(rotated_plan_view) {
				// rotated lines
				write_lines_plan(rotated_lines, longest_indexes);
			} else {
				// unrotated lines
				write_lines_plan(lines, longest_indexes);
			}
		}
		else
		{
			write_lines(lines, longest_indexes);
		}
		
		
		delete& rotated_lines;
		delete& lines;
		
	}
	delete& lines1;
	delete& lines2;
}

void common_write_detection_boxes(int inside_cell_x, int inside_cell_z)
{
	write64_16(num_detection_boxes());
	for(int i=0; i<num_detection_boxes(); i++)
	{
		detection_box_rect rect;
		get_detection_box(rect, i, inside_cell_x, inside_cell_z);
		
		write64_16(rect.min_x);
		write64_16(rect.min_z);
		write64_16(rect.max_x);
		write64_16(rect.max_z);
	}
}

void lines3_common(int level)
{
	ipc_serial_driver ipc_serial;
	line_list_custom_vector& lines1 = *new line_list_custom_vector;
	line_list_custom_vector& lines2 = *new line_list_custom_vector;
	IPC_get_lines ipc(&ipc_serial, static_cast<byte>(level), static_cast<byte>(get_blue_compare()), lines1, lines2);
	
	while(!ipc.completed())
	{
		ipc.service();
	}
	if(ipc.command_failed())
	{
		write_line("cmd fail");
	}
	else
	{
		//write_line("completed");
		advanced_line_vector& lines = *new advanced_line_vector();
		vvappend(lines, lines1);
		vvappend(lines, lines2);
		
		// convert to 3d
		make_lines_plan_view(lines);
		
		// compare lines 60 vs. lines 6
		//...now we need to invalidate lines that are outside the map as we see it
		// do we need to clip lines that pass outside the scene?
		// for now we just:
		// (a) reject lines where both points are beyond 5 cells forward. We probably 
		// can't see this far anyway. This is in the untranslated z direction.
		// (b) reject lines that are off the map in the left-right direction beyond
		// the map edge. We don't do this yet.
		reject_lines(lines);
		
		int longest_indexes[3] = {-1, -1, -1};
		// do this on the 2d data to determine the biggest resolution data
		two_longest_vertical(lines, longest_indexes);
		//longest_indexes[0] = longest_vertical(lines);
		//longest_three(lines, longest_indexes);
		
		//write_memstat();
		advanced_line_vector& rotated_lines = *new advanced_line_vector();
		//write_memstat();
		
		int raw_angle = 9999;
		if(longest_indexes[0] != -1)
		{
			raw_angle = get_heading(lines, longest_indexes[0], longest_indexes[1]);
			//write_string("r= ", raw_angle);
			//write_line(" ang=", trig_angle_to_degrees(raw_angle));
			
			rotate_plan_view(rotated_lines, lines, raw_angle);
			copy_2d_and_bounds(rotated_lines, lines);
		}
		
		//get_position(lines, longest_indexes);
		PositionCalculation position(longest_indexes[0], rotated_lines);	

		// rotated lines
		//write_lines_plan(rotated_lines, longest_indexes);
		int offset_x = 0;
		int offset_z = 66;
		if(position.x_valid()) { offset_x = position.get_cell_offset_x(); }
		if(position.z_valid()) { offset_z = position.get_cell_offset_z(); }

		// XXXXXXXXXXXXXXXXX
		LocalMap map;
		interpret_lines(rotated_lines, map, offset_x, offset_z);
		// XXXXXXXXXXXXXXXXX
		
		write_string("#!2");		// 2 is the type for basic detection map plus maze image
		common_write_detection_boxes(offset_x, offset_z);
		
		write64_16(5);
		for(int z=0; z<5; z++)
		{
			uint16_t front_map = 0;
			uint16_t left_map = 0;
			for(int x=-2; x<3; x++)
			{
				front_map = front_map << 1;
				if(map.forward_wall(x, z)) { front_map |= 1; }
				left_map = left_map << 1;
				if(map.left_wall(x, z)) { left_map |= 1; }
			}
			write64_16(front_map);
			write64_16(left_map);
		}
		
		// show all the lines
		write64_16(rotated_lines.size());
		for(int i = 0; i < rotated_lines.size(); i++)
		{
			write64_16(rotated_lines[i].start_wx);
			write64_16(rotated_lines[i].start_wz);
			write64_16(rotated_lines[i].end_wx);
			write64_16(rotated_lines[i].end_wz);
		}
		
		// show longest lines
		write64_16(3);
		for(int i=0; i<3; i++)
		{
			write64_16(longest_indexes[i]);
		}
		
		// write out the estimated position
		if(position.x_valid()) { write64_16(position.get_cell_offset_x()); }
		else { write64_16(9999); }
		if(position.z_valid()) { write64_16(position.get_cell_offset_z()); }
		else { write64_16(9999); }
		
		int heading_degrees = 9999;
		if(raw_angle != 9999) { heading_degrees = trig_angle_to_degrees(raw_angle); }
		   
		write64_16(heading_degrees);	// write out the angle as well
		   
		write_string("!!");
		
		delete& rotated_lines;
		delete& lines;
		
	}
	delete& lines1;
	delete& lines2;
}


int lines2_command(int argc, const char *const argv[])
{
	int level = find_red_level(argc, argv);
	
	lines2_common(level, false, false);
	
	return 0;
}

//int ee_read_command(int argc, const char *const argv[])
//{
//}

//int ee_write_command(int argc, const char *const argv[])
//{
//}






//void plan](int level)
//{
//	ipc_serial_driver ipc_serial;
//	line_list_custom_vector& lines1 = *new line_list_custom_vector;
//	line_list_custom_vector& lines2 = *new line_list_custom_vector;
//	IPC_get_lines ipc(&ipc_serial, static_cast<byte>(level), static_cast<byte>(get_blue_compare()), lines1, lines2);
//	
//	while(!ipc.completed())
//	{
//		ipc.service();
//	}
//	if(ipc.command_failed())
//	{
//		write_line("cmd fail");
//	}
//	else
//	{
//		write_line("completed");
//		int longest_indexes[3];
//		// do this on the 2d data to determine the biggest resolution data
//		longest_three(lines1, lines2, longest_indexes);
//		
//		make_lines_plan_view(lines1);
//		make_lines_plan_view(lines2);
//		write_lines_plan(lines1, lines2, longest_indexes);
//	}
//	delete& lines1;
//	delete& lines2;
//}

//
//
//
int plan_command(int argc, const char *const argv[])
{
	int level = find_red_level(argc, argv);
	
	lines2_common(level, true, false);
	
	return 0;
}


int multiple_plan_command(int argc, const char *const argv[])
{
	int level = find_red_level(argc, argv);
	
	while(!key_ready()) {
		lines2_common(level, true, false);
		
		delay(100);
	}
	
	read_char();		// get rid of the stop character from the buffer
	return 0;
}

int rotation_command(int argc, const char *const argv[])
{
	int level = find_red_level(argc, argv);
	
	lines2_common(level, true, true);
	
	return 0;
}


int multiple_rotation_command(int argc, const char *const argv[])
{
	int level = find_red_level(argc, argv);
	
	while(!key_ready()) {
		lines2_common(level, true, true);
		
		delay(100);
	}
	
	read_char();		// get rid of the stop character from the buffer
	return 0;
}

int show_params(int argc, const char * const argv[])
{
	stored_parameters* stored = 0;
	Hardware_Manager* hw= Hardware_Manager::Instance();
	stored = hw->get_params();
	if(stored)
	{
		write_cr();
		write_line("Red Level=", stored->get_red_level());
		write_line("Blue compare?", stored->get_is_blue_compare_on());
		write_line("Camera Angle(10ths deg)=", stored->get_camera_angle_in_tenths_of_a_degree());
		write_line("X-offset=", stored->get_x_offset_in_pixels());
		write_string("Size = ("); write_uint(stored->get_map_size_x()); write_string(", "); write_uint(stored->get_map_size_y());
		write_string(") Target = ("); write_uint(stored->get_map_target_x()); write_string(", "); write_uint(stored->get_map_target_y());
		write_string(") Target Size = ("); write_uint(stored->get_map_target_size_x()); write_string(", "); write_uint(stored->get_map_target_size_y());
		write_line(")");
		
		stored->list();
	}
	
	return 0;
}

int list_maths_command(int argc, const char * const argv[])
{
	write_line("CosA=", CosA_scaled);
	write_line("SinA=", SinA_scaled);
	write_line("Yw*SinA_um=",Yw_multiplied_SinA_in_um);
	for(int i=0; i<240; i++)
	{
		write_string(" ", get_Zw_in_mm(i));
	}
	write_cr();

	return 0;
}


int set_red_level(int argc, const char * const argv[])
{
	int level;
	if(!convert_val(argv[1], &level, "level")) { return 1; }
	
	stored_parameters* stored = 0;
	Hardware_Manager* hw= Hardware_Manager::Instance();
	stored = hw->get_params();
	if(stored)
	{
		stored->set_red_level(level);
		write_line("Red Level=", stored->get_red_level());
	}
	return 0;
}
int set_xoffset(int argc, const char * const argv[])
{
	int xoffset;
	if(!convert_val(argv[1], &xoffset, "xoffset")) { return 1; }
	
	stored_parameters* stored = 0;
	Hardware_Manager* hw= Hardware_Manager::Instance();
	stored = hw->get_params();
	if(stored)
	{
		stored->set_x_offset_in_pixels(xoffset);
		Real_World_Constant_Update(stored->get_camera_angle_in_tenths_of_a_degree(), stored->get_x_offset_in_pixels());
		write_line("X-offset=", stored->get_x_offset_in_pixels());
	}
	return 0;
}
int set_camera_angle(int argc, const char * const argv[])
{
	int camera_angle;
	if(!convert_val(argv[1], &camera_angle, "camera angle")) { return 1; }
	
	stored_parameters* stored = 0;
	Hardware_Manager* hw= Hardware_Manager::Instance();
	stored = hw->get_params();
	if(stored)
	{
		stored->set_camera_angle_in_tenths_of_a_degree(camera_angle);
		Real_World_Constant_Update(stored->get_camera_angle_in_tenths_of_a_degree(), stored->get_x_offset_in_pixels());
		write_line("Camera Angle(10ths deg)=", stored->get_camera_angle_in_tenths_of_a_degree());
	}
	return 0;
}
int set5x5(int argc, const char * const argv[])
{	
	stored_parameters* stored = 0;
	Hardware_Manager* hw= Hardware_Manager::Instance();
	stored = hw->get_params();
	if(stored)
	{
		stored->set_test_5x5_maze();
		show_params(0, 0);
	}
	return 0;
}
int set16x16(int argc, const char * const argv[])
{	
	stored_parameters* stored = 0;
	Hardware_Manager* hw= Hardware_Manager::Instance();
	stored = hw->get_params();
	if(stored)
	{
		stored->set_standard_16x16_maze();
		show_params(0, 0);
	}
	return 0;
}

int local_map_command(int argc, const char *const argv[])
{
	int level = find_red_level(argc, argv);
	
	lines3_common(level);
	
	return 0;
}



int show_detection_map_command(int argc, const char *const argv[])
{
	write_string("#!1");		// 1 is the type for basic detection map
	common_write_detection_boxes(0, 66);	
	write_string("!!");
	return 0;
}

int list_detection_map_command(int argc, const char *const argv[])
{
	for(int i=0; i<num_detection_boxes(); i++)
	{
		detection_box_rect rect;
		get_detection_box(rect, i, 0, 66);
		
		write_int(i);
		write_string(": min_x=", rect.min_x);
		write_string(" min_z=", rect.min_z);
		write_string(" max_x=", rect.max_x);
		write_line(" max_z=", rect.max_z);
	}
	return 0;
}

int write64_command(int argc, const char *const argv[])
{
	int value;
	if(! convert_val(argv[1], &value, "val"))
	{
		return 0;
	}
	write64_16(value);
	return 0;
}

int blue_compare_command(int argc, const char *const argv[])
{
	int value;
	if(! convert_val(argv[1], &value, "val"))
	{
		return 0;
	}
	stored_parameters* stored = 0;
	Hardware_Manager* hw= Hardware_Manager::Instance();
	stored = hw->get_params();
	if(stored)
	{
		stored->set_is_blue_compare_on(value);
		write_line("Blue compare=", stored->get_is_blue_compare_on());
	}
	return 0;
}

int test_wdet_command(int argc, const char *const argv[])
{
	return wall_detector_unit_tests();
}


int wall_follower_command(int argc, const char *const argv[])
{
	int level = find_red_level(argc, argv);

	SimpleWallFollowerEngine wf(level, true);
	wf.run();	
	return 0;
}


//#define FOLLOWER_MAP_TEST_COMMANDS
#ifdef FOLLOWER_MAP_TEST_COMMANDS

FollowerMap testmap;

int forward_wall_command(int argc, const char *const argv[])
{
	int x;
	if(! convert_val(argv[1], &x, "x"))
	{
		return 0;
	}
	int z;
	if(! convert_val(argv[2], &z, "z"))
	{
		return 0;
	}
	if(testmap.forward_wall(x,z))
	{
		write_line("Wall exists");
	}
	else
	{
		write_line("No Wall");
	}	
		return 0;
}
int right_wall_command(int argc, const char *const argv[])
{
	int x;
	if(! convert_val(argv[1], &x, "x"))
	{
		return 0;
	}
	int z;
	if(! convert_val(argv[2], &z, "z"))
	{
		return 0;
	}
	if(testmap.right_wall(x,z))
	{
		write_line("Wall exists");
	}
	else
	{
		write_line("No Wall");
	}	
	return 0;
}
int left_wall_command(int argc, const char *const argv[])
{
	int x;
	if(! convert_val(argv[1], &x, "x"))
	{
		return 0;
	}
	int z;
	if(! convert_val(argv[2], &z, "z"))
	{
		return 0;
	}
	if(testmap.left_wall(x,z))
	{
		write_line("Wall exists");
	}
	else
	{
		write_line("No Wall");
	}	
	return 0;
}
int back_wall_command(int argc, const char *const argv[])
{
	int x;
	if(! convert_val(argv[1], &x, "x"))
	{
		return 0;
	}
	int z;
	if(! convert_val(argv[2], &z, "z"))
	{
		return 0;
	}
	if(testmap.back_wall(x,z))
	{
		write_line("Wall exists");
	}
	else
	{
		write_line("No Wall");
	}	
	return 0;
}
int set_forward_wall_command(int argc, const char *const argv[])
{
	int x;
	if(! convert_val(argv[1], &x, "x"))
	{
		return 0;
	}
	int z;
	if(! convert_val(argv[2], &z, "z"))
	{
		return 0;
	}
	testmap.set_forward_wall(x,z);
	testmap.print();
	return 0;
}

int set_right_wall_command(int argc, const char *const argv[])
{
	int x;
	if(! convert_val(argv[1], &x, "x"))
	{
		return 0;
	}
	int z;
	if(! convert_val(argv[2], &z, "z"))
	{
		return 0;
	}
	testmap.set_right_wall(x,z);
	testmap.print();
	return 0;
}
int set_left_wall_command(int argc, const char *const argv[])
{
	int x;
	if(! convert_val(argv[1], &x, "x"))
	{
		return 0;
	}
	int z;
	if(! convert_val(argv[2], &z, "z"))
	{
		return 0;
	}
	testmap.set_left_wall(x,z);
	testmap.print();
	return 0;
}

int set_back_wall_command(int argc, const char *const argv[])
{
	int x;
	if(! convert_val(argv[1], &x, "x"))
	{
		return 0;
	}
	int z;
	if(! convert_val(argv[2], &z, "z"))
	{
		return 0;
	}
	testmap.set_back_wall(x,z);
	testmap.print();
	return 0;
}

int move_forward_command(int argc, const char *const argv[])
{
	testmap.move_forward();
	testmap.print();
	return 0;
}
int rotate_right90_command(int argc, const char *const argv[])
{
	testmap.rotate_right90();
	testmap.print();
	return 0;
}
int rotate_left90_command(int argc, const char *const argv[])
{
	testmap.rotate_left90();
	testmap.print();
	return 0;
}
int rotate_180_command(int argc, const char *const argv[])
{
	testmap.rotate_180();
	testmap.print();
	return 0;
}
#endif

int replay_write_command(int argc, const char *const argv[])
{
	replay_write();
	return 0;
}

int rotate_command(int argc, const char *const argv[])
{
	int angle_deg;
	if(! convert_val(argv[1], &angle_deg, "angle"))
	{
		return 0;
	}
	
	Pos_adjust2 p;
	p.turn_on_spot_from_to_stop(angle_deg);
	while(! p.is_finished())
	{
		p.update();
	}
	
	return 0;
}
int move_command(int argc, const char *const argv[])
{
	int distance;
	if(! convert_val(argv[1], &distance, "distance"))
	{
		return 0;
	}
	
	Pos_adjust2 p;
	p.move_straight_with_stop(distance*1000);
	while(! p.is_finished())
	{
		p.update();
	}
	
	return 0;
}

int ramp_motor_command(int argc, const char *const argv[])
{
#if !MOTOR_DEBUG_LOG
	int motor;
	if(! convert_val(argv[1], &motor, "motor"))
	{
		return 0;
	}
	int time;
	if(! convert_val(argv[2], &time, "time"))
	{
		return 0;
	}
	int forward;
	if(! convert_val(argv[3], &forward, "forward"))
	{
		return 0;
	}

	complex_pwm_drive_control p(motor, time, time);
	p.do_ramp_up_from_stop();
	while(! p.is_idle())
	{
		// wait
	}
	write_line("done, key=stop");
	while(!key_ready())
	{
	}
	read_char();
	p.stop_drive();
	return 0;
#endif
}
int dual_ramp_command(int argc, const char *const argv[])
{
#if !MOTOR_DEBUG_LOG
	int time;
	if(! convert_val(argv[1], &time, "time"))
	{
		return 0;
	}
	
	complex_pwm_drive_control p1(1, time, time);
	complex_pwm_drive_control p2(2, time, time);
	p1.do_ramp_up_from_stop();
	p2.do_ramp_up_from_stop();
	while(! p1.is_idle() || ! p2.is_idle())
	{
		// wait
	}
	write_line("done, key=stop");
	while(!key_ready())
	{
	}
	read_char();
	p1.stop_drive();
	p2.stop_drive();
	return 0;
#endif
}
int pwmf_command(int argc, const char *const argv[])
{
	int level =0;
	bool result1 = convert_val(argv[1], &level);
	
	if(result1)
	{
		DISABLE_MOTOR_1();
		set_single_pwm(1, level);
		set_single_pwm(2, level);
		MOTOR_1_SELECT_FORWARD();
		MOTOR_2_SELECT_FORWARD();

		while(!key_ready())
		{
		}
		read_char();
		set_single_pwm(1, 0);
		set_single_pwm(2, 0);
		disable_single_pwm(1);
		disable_single_pwm(2);
	}
	
	return 0;
}
int motor_demo_command(int argc, const char *const argv[])
{	
	Motor m1(1);
	Motor m2(2);
	m1.set_direction(true);
	m2.set_direction(true);
	m1.speed_up();
	m2.speed_up();
	while(! m1.motor_ramp_finished() || ! m2.motor_ramp_finished())
	{
		// wait
	}
	write_line("top");
	m1.slow_down();
	m2.slow_down();
	while(! m1.motor_ramp_finished() || ! m2.motor_ramp_finished())
	{
		// wait
	}
	write_line("done");
	m1.unhook_pwm();
	m2.unhook_pwm();
	return 0;
}

int ticks_command(int argc, const char *const argv[])
{
	write_uint(fetch_timer_ticks()/100);
	write_string("s  ");
	write_uint(fetch_timer_ticks());
	write_string("cs  cpu=");
	write_uint(cpu_ticks());
	write_line("  1600Hz=", fetch_fast_ticks());
	return 0; 
}

#ifdef ENABLE_CURRENT_LIMIT_WARNING
int I_warn_channel_command(int argc, const char *const argv[])
{
	int channel;
	void set_current_limit_warning_channel(int channel);
	if(convert_val(argv[1], &channel))
	{
		set_current_limit_warning_channel(channel);
	}
	return 0;
}
#endif

//Duncan's movement method: 
//"The phototransistors generate interrupts to the microprocessor. When it receives an interrupt, the microprocessor stops what it is doing and turns the relevant motor off. It also increments a count for that wheel and then returns to what it was doing before. The foreground task in the microprocessor periodically waits until the motors are turned off and then turns each motor on in the appropriate direction. The rate at which it does this determines the speed of the motor. The motors are phase-locked to the foreground task by the action of the interrupts.
//
//The foreground task can also read the counts for each motor to find out how far the motor has turned. This is used to determine when the mouse has moved to the next maze cell.
//
//On DASH-TOO this is basically all there is to it. The speed at which the foreground task gets around to turning the motors on determines the speed and the line sensors are used to determine whether one or both motors should run and in which direction."
//
//So … 
//1. interrupt routine turns off motor and increments count
//2. Foreground routine can either:
//(a) runs a timer and turns motor back on if motor is off (otherwise wait for motor to be off then turn it on), or,
//(b) when the motor is off, run a timer then turn the motor on.
//
//Method (2a) is timing the entire mark-space. This will allow us to set a frequency of the 'steps', which effectively related directly to the speed we want. 
//
//The edge cases are:
//(i) If the 'on' time is longer than the timer period, then the motor must be going as fast as it can.
//(ii) If the poll rate / check rate of the foreground task is slower than the timer expiring, then we'll get a slower speed then we've asked for. 
//(iii) The speed will be effected by the poll rate of the foreground tasks. Slower poll rate means slower then expected speed.
//
//Method (2b) is timing just the off period. 
//
//Of the two, method 2a looks the best and most logical.
//
//We can get over (2a-ii) and (2a-iii) to a large extent by having interrupt based timer expiry tasks.




class dsc_tester_sensor_callback : public Sensing_Callback {
public:
	dsc_tester_sensor_callback(int motor) 
		: edges_till_turn_off(0), position(0),  motor_number(motor) { }
	dsc_tester_sensor_callback() 
		: edges_till_turn_off(0), position(0), motor_number(1) { }
	void edge_happened(motortime_t timer_average, motortime_t last_timer_sample, int edge_count)
	{
		position++;
		if(edges_till_turn_off==0) return;
		edges_till_turn_off--;
		if(edges_till_turn_off==0)
		{
			// turn the motor off
			motor_remove_power(motor_number);
		}
	}
	unsigned int edges_till_turn_off;
	void set_motor_number(int motor) { motor_number = motor; }
	unsigned get_position() { return position; }
	void clear_position() { position = 0; };
private:
	unsigned position;		// how do we get atomic access to this so we can read and clear??
	// alternative to just to read difference from last time...
	// 32 bit unsigned int can go ... 1000 km. Probably good enough :-)

	int motor_number;
};
//int get_position() { return position; }
//void clear_position() { position = 0; };


int duncans_speed_control_command(int argc, const char *const argv[])
{
	int speed;
	if(convert_val(argv[1], &speed))
	{
		speed = speed * 1000;	// make from mm/s into um/s
		
		dsc_tester_sensor_callback m_controller[2];
		m_controller[0].set_motor_number(1);
		m_controller[1].set_motor_number(2);
		Motor_input_data* sensor[2];
		Sensing_Callback* old[2];

		// install new sensor callbacks
		for(int i = 0; i < 2; i++)
		{
		sensor[i] = motor_sense(i+1);
		old[i] = sensor[i]->set_edge_reading(&m_controller[i]);
		}

		// calculate the timer value for the speed
		// say we want 500mm/s ... 
		// that's so many sensor interrupts ('clicks')
		// then we can do inversion to get 
		//   clicks_per_second = speed / (convert_to_um(1)/1000.0)
		// but we want seconds_per_click (or ms_per_click) for the timer value
		//   timer_value = (convert_to_um(1)/1000.0) / speed;
		// e.g: 12.5mm/s = ~50clicks/sec = 0.05 clicks/ms = 20 ms/click
		// e.g: 25mm/s = ~100clicks/sec = 0.1 clicks/ms = 10 ms/click
		// e.g: 50mm/s = ~200clicks/sec = 0.2 clicks/ms = 5 ms/click
		// e.g: 100mm/s = ~400clicks/sec = 0.4 clicks/ms = 2.5 ms/click
		// e.g: 200mm/s = ~ 800clicks/sec = 0.8 clicks/ms = 1.25 ms/click
		// e.g. 250mm/s = ~1000 clicks/sec = 1.0 clicks/ms = 1.0 ms/click
		// e.g: 500mm/s = ~2000clicks/sec = 2 clicks/ms = 0.5 ms/click
		// e.g: 1000mm/s = ~4000clicks/sec = 4 clicks/ms = 0.25 ms/click
		// therefore we probably want to do it over a longer time period?
		// fast timer is 1600 or 3200Hz, so sub-ms resolution
		//
		// at slower speeds we can do it with 1 click (or 2)
		// at faster speeds we will need multiple clicks
		//
		// We could also consider using 2 clicks always as current it's for the mark
		// and space which are uneven...
		//
		//
		// Speed accuracy required? 10% 5%? 20%?
		// At 1600Hz period = 0.625ms. At 3200Hz period = 0.3125ms.
		// For 10% ... @1600Hz 6.25ms minimum, @3200Hz 3.125ms.
		// Assume 10%, 3200Hz = 5ms.
		// Ignore jitter caused by slower polling (if necessary set up interrupt 
		// based timers with function on expiry)
		//
		// Table (even clicks only)
		// e.g: 12.5mm/s = 40 ms / 2 clicks	
		// e.g: 25mm/s = 20 ms / 2 clicks
		// e.g: 50mm/s = 10 ms / 2 clicks
		// e.g: 100mm/s = 5 ms / 2 clicks  ...or... 10 ms / 4 clicks
		// e.g: 200mm/s = 5 ms / 4 clicks
		// e.g. 250mm/s = 6 ms / 6 click
		// e.g: 500mm/s = 5 ms / 10 click
		// e.g: 1000mm/s = 5ms / 20 clicks
		//
		//
		unsigned int timer_value;
		unsigned int counts = 2;
		// this is not the fastest way of doing things, but this is only a command
		// line test - divide and multiply per loop - doesn't matter here.
		// Ensure we have a timer value of at least 5 before we exit the loop. 
		// Counts are always even to stop the problem with {uneven mark/space 
		// ratio on phototransistor inputs}??
		// 
		do {
			timer_value = convert_to_um(counts)*1000 / speed;
			counts += 2;
		} while(timer_value < 5);
		counts -= 2;
		write_string("Timer=", timer_value); 
		write_string("ms Counts=", counts); 
		write_string("speed=", speed);
		write_line("um/s");
		write_line("q to quit");
		//
		// put both motors on
		bool forward = true;
		motor_full(1, forward);
		motor_full(2, forward);
		
		// set up the sensor flags
		m_controller[0].edges_till_turn_off = counts;
		m_controller[1].edges_till_turn_off = counts;
		
		// now we go into a loop waiting for the timer to expire and the 
		msTimer* t[2];
		for(int i = 0; i < 2; i++) { t[i] = new msTimer(timer_value); }

		//sint32_t old_position1 = m_controller[0].position;
		//sint32_t old_position2 = m_controller[1].position;

		// this is the control loop for speed (no position control here ... e.g. watching/counting clicks)
		while(1)
		{
			if(key_ready())
			{
				char c = read_char();
				if(c == 'q' || c == 'Q') break;
			}
			for(int i = 0; i < 2; i++)
			{
				// deal with each motor in turn
				if(t[i]->expired() && m_controller[i].edges_till_turn_off==0)
				{
					// runs a timer and turns motor back on if motor is off 
					// (otherwise wait for motor to be off then turn it on)
					t[i]->set(timer_value);
					m_controller[i].edges_till_turn_off = counts;
					motor_full(i+1, forward);
					
					// if we turn the motor controllers off seperately (in order to get the 
					// positions right) we are likely to skew the positions at the end...
					//
					// if((m_controller[0].position - old_position1) < 0) { /* stop both? */ } 
					// if((m_controller[1].position - old_position2) < 0) { /* stop both? */ }

				}
			}

		}

		// delete timers
		for(int i = 0; i < 2; i++) { delete t[i]; }

		// reinstate old sensor callbacks
		for(int i = 0; i < 2; i++)
		{
			sensor[i]->set_edge_reading(old[i]);
		}

		motor_remove_power(1);
		motor_remove_power(2);

	}
	return 0;
}

// ===================================
// -                                 -
// --==  A C C E L E R A T I O N  ==--
// -                                 -
// ===================================

// Acceleration: We want a constant acceleration to avoid wheel slip
//
// Therefore speed (up to target speed, should increase a constant amount per unit
// of time.
//
// E.g. target speed = 1000 mm/s, acceleration = 100 mm/s/s
// @ 1 second speed will be 100 mm/s
// @ 5 seconds speed will be 500 mm/s 
// @ 10 seconds speed will be 1000 mm/s
//
// (Similarly for deceleration)
//
// What should the fixed unit of time be between changes in speed?
// Ideally, very close to zero otherwise
// Realistically, it's pointless changing it between photointerruptor interrupts
// because the motor is controlled via an on/off method.
// 
// Either: Change in photointerrupt (only really affects high speeds, > 250mm/s)
// or change in timer interrupt. Only problematic at very slow interrupts.
// However speed control is ONLY at level of timer, so if we are change in 
// photo-interrupt we are not controlling the speed (since timer is 
// on-off cycle).
//
// So in timer we re-calculate the next speed.
//
// Acceleration Implementation Notes
// =================================
// We probably need a start speed.
// Then increase by acceleration over period of timer.
//  - acceleration in um/s/s multiplied by next timer period in seconds
//  - ALTERNATIVELY: acceleration in um/s/s divided by next timer frequency
// but cap at max speed.
//
// We could precalculate all of these speed values and cache them all.
//
// Deceleration
// ============
// 
// Is deceleration simply stepping back through the table?
// Seems that if acceleration is a constant, speed table will be (pretty much)
// a straight line. Therefore it seems likely./ 

//const unsigned int frequency_of_timer = 1000;
const unsigned int frequency_of_timer = 3200;		// assumption checked in set()
const unsigned int minimum_time_in_ms = 5;
const unsigned int minimum_time_in_ticks = (minimum_time_in_ms*frequency_of_timer)/1000;

unsigned int DSC_calculate_speed(unsigned int* timer_value_in_ticks_return, unsigned int speed_in_um_per_s)
{
	unsigned int timer_value_in_ticks;
	unsigned int counts = 2;

	// this is not the fastest way of doing things, but this is only a command
	// line test - divide and multiply per loop - doesn't matter here.
	// Ensure we have a timer value of at least 5 before we exit the loop. 
	// Counts are always even to stop the problem with {uneven mark/space 
	// ratio on phototransistor inputs}??
	// 
	do {
		timer_value_in_ticks = convert_to_um(counts)*frequency_of_timer / speed_in_um_per_s;
		counts += 2;
	} while(timer_value_in_ticks < minimum_time_in_ticks);
	counts -= 2;
	
	*timer_value_in_ticks_return = timer_value_in_ticks;
	return counts;
}


class SpeedTableCache {
public:
	SpeedTableCache()
	{
		unsigned int speed_in_mm_s = 400;
		unsigned int accceleration_in_mm_s_s = 500;
		set(speed_in_mm_s * 1000, accceleration_in_mm_s_s * 1000);
	}
	SpeedTableCache(unsigned int max_speed_in_um_per_sec, unsigned int acceleration_in_um_per_sec_per_sec)
	{
		set(max_speed_in_um_per_sec, acceleration_in_um_per_sec_per_sec);
	}
	void set(unsigned int max_speed_in_um_per_sec, unsigned int acceleration_in_um_per_sec_per_sec);
	
	void set_speed_only(unsigned int max_speed_in_um_per_sec)
	{
		set(max_speed_in_um_per_sec, acceleration_set);
	}
	// these two functions provie a way of acessing the data by an external source
	// the alternative was a friend class
	unsigned int get_max_valid_index() const
	{
		return max_valid_index;
	}
	//void get_table_accessors(const unsigned int*& timer_table_pref, const unsigned int*& count_table_pref) const
	//{
	//	timer_table_pref = timer_table;
	//	count_table_pref = count_table;
	//}
	const unsigned int* get_timer_table() const
	{
		return timer_table;
	}
	const unsigned int* get_count_table() const
	{
		return count_table;
	}
//	unsigned int get_count()
//	{
//		return count_table[index];
//	}
//	unsigned int get_timer_value()
//	{
//		return timer_table[index];
//	}
	void print_table() const;
private:
	unsigned int speed_set;
	unsigned int acceleration_set;
	enum { 
		max_steps = 200,
		start_speed_in_um_per_sec = 10000,
		// very jerky, even at 10 mm/s
//		start_speed_in_um_per_sec = 2000,
	};
	unsigned int count_table[max_steps];
	unsigned int timer_table[max_steps];
	unsigned int max_valid_index;
};

//
// IF the speed changes, SpeedTableController breaks because:
// 
//  1. It has no way of updating the index to reflect the current speed (although
//     you can reset the index, e.g. if the robot it stopped)
//  2. It has not way of updating the max_valid_index.
//
// Both these are trivial to fix.
//  Either have a function that does the speed change on request (code 
//  to do this is commneted out in SpeedTableCache::set), or a callback from 
//  SpeedTableCache::set when the speed changes. (This can update the max_valid_index
//  as well). You could use SpeedTableController::init() for part of this solution.
//
class SpeedTableController {
public:
	// Constructor where we know what we are binding it too.
	SpeedTableController(const SpeedTableCache& cache)
	: index(0)
	, timer_table(cache.get_timer_table())
	, count_table(cache.get_count_table())
	, max_valid_index(cache.get_max_valid_index())
	//, stc(&cache)
	{
		//cache.get_table_accessors(timer_table, count_table);
	}
	// Constructor where we will will bind it at a later date. Useful for
	// stupid C arrays.
	SpeedTableController()
	: index(0)
	// do some magic here with timer_table and count_table to stop invalid 
	// accesses of random variables...
	//
	// we could point it at a single VALID entry, but at least point it at 
	// a known variable
	, timer_table(&index)
	, count_table(&index)
	, max_valid_index(0)
	//, stc(0)
	{
	}
	// set up (or change) the basic bindings and data.
	void init(const SpeedTableCache& cache)
	{
		index = 0;
		timer_table = cache.get_timer_table();
		count_table = cache.get_count_table();
		max_valid_index = cache.get_max_valid_index();
		//cache.get_table_accessors(timer_table, count_table);
		//stc = &cache;
	}
//	void set_this_channels_max_speed(unsigned int speed_in_um_per_s)
//	{
//		// walk the table here... and pick a similar speed.
//	}
	void speed_zero_reset_index()
	{
		index = 0;
	}	
	void faster()
	{
		if(index < max_valid_index) { index ++; }
	}
	void slower()
	{
		if(index > 0) { index --; }
	}
	bool at_max_speed()
	{
		return index >= max_valid_index;
	}
	bool at_min_speed()
	{ 
		return index == 0;
	}
	unsigned int get_count()
	{
		return count_table[index];
	}
	unsigned int get_timer_value()
	{
		return timer_table[index];
	}
protected:
	unsigned int index;
	const unsigned int* timer_table;
	const unsigned int* count_table;
	unsigned int max_valid_index;
	//SpeedTableCache* stc;			// is we need to call SpeedTableCache routines ... but be warned doing this on a single channel is a bad idea since the data is shared...
};


// set()
//
// This function sets the table to a specific speed & acceleration.
//
// It also is responsible for setting all object variables EXCEPT index.
// But index is modified to the correct speed in the new table.
//
void SpeedTableCache::set(unsigned int max_speed_in_um_per_sec, unsigned int acceleration_in_um_per_sec_per_sec)
{
	if(how_many_fast_ticks_per_second() != 3200)
	{
		halt("ticks must be 3200Hz");
	}
#if 0		// see note about not containing any state
	unsigned old_speed = convert_to_um(count_table[index]) / timer_table[index];
#endif 
	speed_set = max_speed_in_um_per_sec;
	acceleration_set = acceleration_in_um_per_sec_per_sec;
	
	unsigned i = 0;
	unsigned speed = start_speed_in_um_per_sec;
	// allow us to test slower speeds
	if(max_speed_in_um_per_sec < start_speed_in_um_per_sec)
	{
		speed = max_speed_in_um_per_sec;
	}
	do {
		unsigned int timer_value_in_ticks;
		unsigned int counts = DSC_calculate_speed(&timer_value_in_ticks, speed);
		
		count_table[i] = counts;
		timer_table[i] = timer_value_in_ticks;
		
		unsigned speed_increment = (acceleration_in_um_per_sec_per_sec * timer_value_in_ticks) / frequency_of_timer;
		speed += speed_increment;
		i ++;
		
	} while(speed < max_speed_in_um_per_sec && i < max_steps);

	max_valid_index = i - 1;

	// since this class doesn't have an idea of current state, it can't compensate
	// therefore, don't change the acceleration table when moving!!!!
#if 0
	// ...recalculate index here... if necessary
	if(index != 0)
	{
		// if someone is mad enough to change the speed table whilst
		// the mouse is moving, we have to recalculate the index to be
		// close to the old speed as possible ... probably not used..
		// definately not tested :-)
		if(old_speed > max_speed_in_um_per_sec)
		{
			index = max_valid_index;
		}
		else
		{
			index = max_valid_index;	// just in case
			for(i = 0; i <= max_valid_index; i++)
			{
				unsigned new_speed = convert_to_um(count_table[i]) / timer_table[i];
				if(new_speed >= old_speed)
				{
					index = i;
					break;
				}
			}
		}
		
	}
#endif
}

//
// Prints the table (and related variables) for debugging.
//
void SpeedTableCache::print_table() const
{
	write_string("Requested Speed=", speed_set);
	write_string("um/s ");
	write_string("Requested Acceleration=", acceleration_set);
	write_line("um/s/s");
	
	unsigned accum_time = 0;
	unsigned last_speed = 0;
	for(unsigned i = 0; i <= max_valid_index ; i++)
	{
		write_string("[", i);
		write_string("] ");
		write_string(" T=", accum_time);
		write_string(" Timer=", timer_table[i]); 
		write_string(" Count=", count_table[i]); 
		accum_time += timer_table[i];
		// speed = convert_to_um(count)/timer_value
		// NOT TRUE ANYMORE: *1000 is because timer_table = ms
		unsigned this_speed = convert_to_um(count_table[i])*frequency_of_timer / timer_table[i];
		write_string(" (Calc speed = ", this_speed);
		write_string(" um/s, increment=", this_speed - last_speed);
		write_line("um/s)");
		last_speed = this_speed;
	}
	//write_line("Current Index = ", index);
}


SpeedTableCache DSC_speed_cache;

#define CALLBACK_TIMER_RATHER_THAN_POLLED_TIMER 1

#if CALLBACK_TIMER_RATHER_THAN_POLLED_TIMER
class DSC_timer : public Timer_Callback {
public:
	virtual unsigned int interrupt()
	{
		return time;
	}
private:
};
#endif

// distance in um, zero if no distance set
// speed in um/s
// accel in um/s/s, zero if no acceleration set
void DSC_table_based_controller(unsigned distance, unsigned speed, unsigned accel, bool synchronise_motors = false)
{
	dsc_tester_sensor_callback m_controller[2];
	m_controller[0].set_motor_number(1);
	m_controller[1].set_motor_number(2);
	Motor_input_data* sensor[2];
	Sensing_Callback* old[2];
	
	//SpeedTableController DSC_speed_cache_controller0(DSC_speed_cache);
	//SpeedTableController DSC_speed_cache_controller1(DSC_speed_cache);
	//SpeedTableController& DSC_speed_cache_controller[2] = 
	//{
	//	DSC_speed_cache_controller0,
	//	DSC_speed_cache_controller1,
	//};
	
	//std::vector<SpeedTableController>DSC_speed_cache_controller;	
	//DSC_speed_cache_controller.push_back(copy_of_controller);
	//DSC_speed_cache_controller.push_back(copy_of_controller);

	SpeedTableController DSC_speed_cache_controller[2];
	DSC_speed_cache_controller[0].init(DSC_speed_cache);
	DSC_speed_cache_controller[1].init(DSC_speed_cache);
	
	// install new sensor callbacks
	for(int i = 0; i < 2; i++)
	{
		sensor[i] = motor_sense(i+1);
		old[i] = sensor[i]->set_edge_reading(&m_controller[i]);
	}
	
	// make sure the controllers are reset

	DSC_speed_cache_controller[0].speed_zero_reset_index();
	DSC_speed_cache_controller[1].speed_zero_reset_index();
	// calculate the speeds
	if(accel == 0)
	{
		DSC_speed_cache.set_speed_only(speed);
	}
	else
	{
		DSC_speed_cache.set(speed, accel);
	}
	DSC_speed_cache.print_table();
	
	// will be the same for both motors since both indexes are reset
	unsigned int timer_value = DSC_speed_cache_controller[0].get_timer_value();
	unsigned int counts = DSC_speed_cache_controller[0].get_count();
	
	write_string("Timer=", timer_value); 
	write_string("ms Counts=", counts); 
	write_string("speed=", speed);
	write_line("um/s");
	write_line("q to quit");
	//
	// put both motors on
	bool forward = true;
	motor_full(1, forward);
	motor_full(2, forward);
	
	// set up the sensor flags
	m_controller[0].edges_till_turn_off = counts;
	m_controller[1].edges_till_turn_off = counts;
	
#if not CALLBACK_TIMER_RATHER_THAN_POLLED_TIMER
	// now we go into a loop waiting for the timer to expire and the 
	msTimer t[2];
	for(int i = 0; i < 2; i++) { t[i].set_raw(timer_value); }
#else
	register_timer_callback(, timer_value);
#endif
	
	//sint32_t old_position1 = m_controller[0].position;
	//sint32_t old_position2 = m_controller[1].position;
	
	// this is the control loop for speed (no position control here ... e.g. watching/counting clicks)
	bool request_stop = false;
	bool channel_expired[2] = { false, false };
	// should we accelerate both motors at the same time???
	// probably not because this would put them out of sync more?
	
	if(distance)
	{
		write_string("Request to travel: ", m_controller[0].get_position());
		// convert to a format that's easy to compare
		distance = convert_to_sensor_ticks(distance);
		write_string(" um (sensor ticks ", distance);
		write_line(" )");
		// distance covered by 2 motors
		distance *= 2;
	}
	while(channel_expired[0] == false or channel_expired[1] == false)
	{
		if(key_ready())
		{
			char c = read_char();
			if(c == 'q' || c == 'Q') { request_stop = true; }
		}
		if(synchronise_motors)
		{
			// if both motors have stepped the required number of edges in the 
			// required time, go onto the next edge
			if(t[0].expired() && m_controller[0].edges_till_turn_off==0 &&
			   t[1].expired() && m_controller[1].edges_till_turn_off==0)
			{
				// deal with the speed adjustment in common
				if(not request_stop)
				{
					DSC_speed_cache_controller[0].faster();
				}
				else
				{
					if(DSC_speed_cache_controller[0].at_min_speed())
					{
						channel_expired[0] = true;
						channel_expired[1] = true;
						continue;
					}
					DSC_speed_cache_controller[0].slower();
				}

				timer_value = DSC_speed_cache_controller[0].get_timer_value();
				counts = DSC_speed_cache_controller[0].get_count();
				
				// deal with each motor in turn
				for(int i = 0; i < 2; i++)
				{
					// runs a timer and turns motor back on if motor is off 
					// (otherwise wait for motor to be off then turn it on)
					t[i].set_raw(timer_value);
					m_controller[i].edges_till_turn_off = counts;
					motor_full(i+1, forward);
				}
			}
		}
		else	// the motors are not synchnorised and might be running at different
			// speeds. Therefore we need to treat them as seperate units.
		{
			for(int i = 0; i < 2; i++)
			{
				// deal with each motor in turn
				if(t[i].expired() && m_controller[i].edges_till_turn_off==0)
				{
					if(not request_stop)
					{
						DSC_speed_cache_controller[i].faster();
					}
					else
					{
						if(DSC_speed_cache_controller[i].at_min_speed())
						{
							channel_expired[i] = true;
							continue;
						}
						DSC_speed_cache_controller[i].slower();
					}
					
					timer_value = DSC_speed_cache_controller[i].get_timer_value();
					counts = DSC_speed_cache_controller[i].get_count();
					
					// runs a timer and turns motor back on if motor is off 
					// (otherwise wait for motor to be off then turn it on)
					t[i].set_raw(timer_value);
					m_controller[i].edges_till_turn_off = counts;
					motor_full(i+1, forward);
					
					// if we turn the motor controllers off seperately (in order to get the 
					// positions right) we are likely to skew the positions at the end...
					//
					// if((m_controller[0].position - old_position1) < 0) { /* stop both? */ } 
					// if((m_controller[1].position - old_position2) < 0) { /* stop both? */ }
				}
			}
		}

		if(distance)
		{
			unsigned travelled = m_controller[0].get_position()+m_controller[1].get_position();
			if(travelled >= distance)
			{
				request_stop = true;
			}
		}
		
	}
	write_line("Motor1 position = ", m_controller[0].get_position());
	write_line("Motor2 position = ", m_controller[1].get_position());
	write_string("Distance travelled = ", convert_to_um((m_controller[0].get_position()+m_controller[0].get_position())/2)/1000);
	write_line(" mm");
	// reinstate old sensor callbacks
	for(int i = 0; i < 2; i++)
	{
		sensor[i]->set_edge_reading(old[i]);
	}
	
	motor_remove_power(1);
	motor_remove_power(2);
}

// command to do just speed and acceleration
int duncans_speed_control_acceleration_command(int argc, const char *const argv[])
{
	int speed;
	int accel = 0;	// DSC_table_based_controller takes 0 for accel if not acceleration
	bool success = convert_val(argv[1], &speed);
	if(argc == 3)
	{
		success = success && convert_val(argv[2], &accel);
	}
	if(success)
	{
		speed = speed * 1000;	// make from mm/s into um/s
		accel = accel * 1000;	// make from mm/s/s into um/s/s
		
		DSC_table_based_controller(0, speed, accel);
	}
	return 0;
}

// movement command
int duncans_speed_control_move_command(int argc, const char *const argv[])
{
	int speed;
	int distance;
	bool success = convert_val(argv[1], &speed);
	success = success && convert_val(argv[2], &distance);

	if(success)
	{
		speed = speed * 1000;	// make from mm/s into um/s
		distance = distance * 1000;	// make from mm into um
		
		DSC_table_based_controller(distance, speed, 0);
	}
	return 0;
}
								   
int DSC_print_table_command(int argc, const char *const argv[])
{
	DSC_speed_cache.print_table();
	return 0;
}

int DSC_sync_motor_command(int argc, const char *const argv[])
{
	int speed;
	int distance;
	int accel = 0;	// DSC_table_based_controller takes 0 for accel if not acceleration
	
	bool success = convert_val(argv[1], &speed);
	success = success && convert_val(argv[2], &distance);
	if(argc == 4)
	{
		success = success && convert_val(argv[3], &accel);
	}
	if(success)
	{
		speed = speed * 1000;	// make from mm/s into um/s
		accel = accel * 1000;	// make from mm/s/s into um/s/s
		distance = distance * 1000;
		
		DSC_table_based_controller(distance, speed, accel, true);
	}
	return 0;
}


//const command_list_t robot_command_list[] = {
const command_list_t cmds[] = {
	{"help", help, "Type 'help command'. Also see list", 1, 2},
	{"list", list, "Lists commands", 1, 1},
	{"led", led_command, "Led on/off", 2, 2},
	{ "m1_remove_power", motor_1_remove_power_command, "freewheel stop", 1, 1, },
	{ "m2_remove_power", motor_2_remove_power_command, "freewheel stop", 1, 1, },
	{ "m1_full_forward", motor_1_full_forward_command, "fast forward", 1, 1, },
	{ "m2_full_forward", motor_2_full_forward_command, "fast forward", 1, 1, },
	{ "m1_full_reverse", motor_1_full_reverse_command, "fast reverse", 1, 1, },
	{ "m2_full_reverse", motor_2_full_reverse_command, "fast reverse", 1, 1, },
	{ "m1_fast_stop", motor_1_fast_stop_command, "hard break", 1, 1, },
	{ "m2_fast_stop", motor_2_fast_stop_command, "hard break", 1, 1, },
	{ "enable_pwm", enable_pwm_command, "two params, 0 to 1000", 3, 3, },
	{ "adjust_pwm", adjust_pwm_command, "two params, 0 to 1000", 3, 3, },
	{ "disable_pwm", disable_pwm_command, "disable motor pwm", 1, 1, },
	{ "pwm", set_single_pwm_command, "motor#, lvl=0 to 1000", 3, 3, },
	{ "pwm_off", disable_single_pwm_command, "motor#", 2, 2, },
	{ "m1_input", m1_input_command, "print motor 1 ticks", 1, 1, },
	{ "m2_input", m2_input_command, "print motor 2 ticks", 1, 1, },
	{ "printnum", printnum_command, "arg is number", 1, 1, },
	
	//
	// Movement commands
	//
	{ "run", run_command, "starts main program", 1, 1 },
	{ "stop", stop_command, "stops main program", 1, 1 },
	{ "dist", dist_command, "distance to move in mm", 2, 2, },
	{ "left", left_command, "turn left", 1, 1 },
	{ "right", right_command, "turn right", 1, 1 },
	{ "angle", angle_command, "angle in degrees", 2, 2, },
	
	{ "avt", avt_command, "motor average time, param motor", 2, 2, },
	
	// memory primitives
	{ "malloc", malloc_command, "space to allocate, returns pointer", 2, 2, },
	{ "free", free_command, "pointer to free", 2, 2, },
	{ "memstat", memstat_command, "prints memory system stats", 1, 1, },
	{ "total_free", total_free_command, "prints total free memory", 1, 1, },
	{ "largest_free", largest_free_command, "prints largest block", 1, 1, },
	
	// new and delete test
	{ "new_obj", new_obj_command, "create a test object with new", 1, 1, },
	{ "delete_obj", delete_obj_command, "delete a test object", 1, 1, },
	{ "obj_action", obj_action_command, "perform action on a test object", 1, 1, },
	
	// speed control object testing
	{ "new_speed_ctrl", new_speed_ctrl_command, "motor", 2, 2, },
	{ "delete_speed_ctrl", delete_speed_ctrl_command, "remove object", 1, 1, },
	{ "speed_ctrl_on", speed_ctrl_on_command, "speed in mm/sec", 2, 2, },
	{ "speed_ctrl_off", speed_ctrl_off_command, "", 1, 1, },
	{ "sp_ctrl", speed_ctrl_command, "move squares at x mm/s", 2, 2, },
	
	{ "new_i2c", new_i2c_command, "low level i2c driver", 1, 1, },
	{ "delete_i2c", delete_i2c_command, "", 1, 1, },
	{ "i2c_status", i2c_status_command, "read (no params) or write status", 1, 5, },
	{ "i2c_data", i2c_data_command, "read or write data", 1, 2, },
	{ "i2c_test", i2c_test_command, "test i2c to camera", 1, 1, },
	{ "i2c_test2", i2c_test2_command, "high-level test to camera", 1, 1, },
	
	{ "camser_read", camser_read_command, "read camera command: register, #bytes", 3, 3, },
	{ "camser_write", camser_write_command, "write camera command: register, byte1, [byte2, ... byte10]", 3, 12, },
	{ "camser", camser_completed_command, "tell if camser completed", 1, 1, },
	
	{ "pictest", pictest_command, "simple picture read", 1, 1, },
	{ "piciotest", piciotest_command, "test camera i/o", 1, 1, },
	{ "picmeasure", picmeasure_command, "measure camera params", 1, 1, },
	{ "picsetparam", picsetparam_command, "pixel clock div, 8bitmode", 3, 3, },
	{ "picdutycycles", picdutycycles_command, "8 bit duty cycles for 64 divider", 1, 1, },
	{ "pic", picget_image_command, "get image from cam ser, param1: 0=320x240 1=160x120 3=bit no filt 4=bit filtered", 2, 3, },
	{ "test_xmodem", test_xmodem_command, "Send a file for timing", 1, 1, },
	{ "ipc_led", ipc_led_command, "which led, state", 3, 3, },
	{ "ipc_device", ipc_device_command, "get camera device", 1, 1, },
	{ "ipc_button", ipc_button_command, "get camera button", 1, 1, },
	{ "ipc_i2c", ipc_i2c_command, "get/set i2c", 2, 3, },
	{ "test", test_command, "generic developer test", 1, 1, },
	//{ "ee_read", ee_read_command, "address", 2, 2, },
	//{ "ee_write", ee_write_command, "address, data", 3, 3, },
	{ "init_cam", init_camera_command, "setup camera", 1, 1, },
	{ "lines", lines_command, "get lines from cam board param=level", 1, 2, },
	{ "mlines", multiple_lines_command, "multiple lines param=level", 1, 2, },
	{ "plan", plan_command, "plan view param=level", 1, 2, },
	{ "mplan", multiple_plan_command, "multiple plan views param=level", 1, 2, },
	{ "lines2", lines2_command, "get maze lines param=level", 1, 2, },
	{ "show_params", show_params, "params from eeproms", 1, 1, },
	{ "set_red_level", set_red_level, "red level 0-100", 2, 2, },
	{ "set_xoffset", set_xoffset, "x-offset 0-319", 2, 2, },
	{ "set_camera_angle", set_camera_angle, "camera angle in 10ths degrees", 2, 2, },
	{ "set5x5", set5x5, "set to 5x5 maze", 1, 1, },
	{ "set16x16", set16x16, "set to standard maze", 1, 1, },
	{ "rot", rotation_command, "rotated plan view param=level", 1, 2, },
	{ "mrot", multiple_rotation_command, "multiple rotated plan views param=level", 1, 2, },
	{ "list_maths", list_maths_command, "list trig data", 1, 1, },
	{ "local_map", local_map_command, "get local map, param=level", 1, 2, },
	{ "list_det_map", list_detection_map_command, "list detection map", 1, 1, },
	{ "show_det_map", show_detection_map_command, "display detection map", 1, 1, },
	{ "write64", write64_command, "param = number to show", 2, 2, },
	{ "blue_compare", blue_compare_command, "param 0=R>G, 1=R>B", 2, 2, },
	{ "test_wdet", test_wdet_command, "test wall detector", 1, 1, },
	{ "wf", wall_follower_command, "wall follower", 1, 2, },
#ifdef FOLLOWER_MAP_TEST_COMMANDS
	{ "forward_wall", forward_wall_command, "", 3, 3, },
	{ "right_wall", right_wall_command, "", 3, 3, },
	{ "left_wall", left_wall_command, "", 3, 3, },
	{ "back_wall", back_wall_command, "", 3, 3, },
	{ "set_forward", set_forward_wall_command, "", 3, 3, },
	{ "set_right", set_right_wall_command, "", 3, 3, },
	{ "set_left", set_left_wall_command, "", 3, 3, },
	{ "set_back", set_back_wall_command, "", 3, 3, },
	{ "move_forward", move_forward_command, "", 1, 1, },
	{ "rotate_right90", rotate_right90_command, "", 1, 1, },
	{ "rotate_left90", rotate_left90_command, "", 1, 1, },
	{ "rotate_180", rotate_180_command, "", 1, 1, },
#endif
	{ "replay_write", replay_write_command, "replays printing", 1, 1, },
	{ "rotate", rotate_command, "rotate robot x degrees", 2, 2, },
	{ "move", move_command, "move robot x mm", 2, 2, },
	{ "ramp_motor", ramp_motor_command, "motor, time_ms, forward", 4, 4, },
	{ "pwmf", pwmf_command, "pwm until key(value)", 2, 2, },
	{ "dual_ramp", dual_ramp_command, "time_ms", 2, 2, },
	{ "motor_demo", motor_demo_command, "speed up then slow down", 1, 1, },
	{ "ticks", ticks_command, "show various system ticks", 1, 1, },
	{ "beep", beep_command, "on/off/duration/duration,pitch/zing/music/m2", 2, 3},
#ifdef ENABLE_CURRENT_LIMIT_WARNING
	{ "I_warn_chann", I_warn_channel_command, "1 2 3=both", 2, 2},
#endif
	{ "Duncans_spd_ctrl", duncans_speed_control_command, "mm/s", 2, 2 },
	{ "DSC_accel", duncans_speed_control_acceleration_command, "speed max mm/s [, accel mm/s/s]", 2, 3 },
	{ "DSC_move", duncans_speed_control_move_command, "speed mm/s, distance mm", 3, 3 },
	{ "DSC_sync_motors", DSC_sync_motor_command, "speed, distance, [accel]", 3, 4 },
	{ "DSC_print_table", DSC_print_table_command, "", 1, 1 },
};

//unsigned int robot_size()
unsigned int size()
{
	//	return sizeof(command_list)/sizeof(command_list_t);
	return sizeof(cmds)/sizeof(command_list_t);
}
