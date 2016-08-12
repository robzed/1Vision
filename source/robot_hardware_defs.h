/*
 *  robot_hardware_defs.h
 *  
 *
 *  Created by Rob Probin on 09/06/2006.
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
 *
 * Definitions for Mouse Robot core program in LPC-2106
 */

#ifndef ROBOT_HARDWARE_DEFS_H
#define ROBOT_HARDWARE_DEFS_H

#include "lpc210x_gnuarm.h"
#include "robot_basic_types.h"

//
// hardware definition support macros
//
#define SIMPLE_GET_INPUT_MACRO(port) (GPIO_IOPIN & (1<<(port)))
#define SET_PORT(portnum) GPIO_IOSET=(1<<portnum);
#define CLEAR_PORT(portnum) GPIO_IOCLR=(1<<portnum);

class IOPort {
public:
	IOPort(int port_num, bool output) { 
		portnum = port_num; 
		if(output) { GPIO_IODIR |= (1<<portnum); } 
		else { GPIO_IODIR &= ~(1<<portnum); }
	}
	void set() { GPIO_IOSET=(1<<portnum); }
	void clear()  { GPIO_IOCLR=(1<<portnum); }
	bool get() { return (GPIO_IOPIN & (1<<(portnum))); }
protected:	// protected is evil, generally...
	byte portnum;
};

//inline void set_port(int portnum) { GPIO_IOSET=(1<<portnum); }
//inline void clear_port(int portnum) { GPIO_IOCLR=(1<<portnum); }
//inline bool get_port(int portnum) { return (GPIO_IOPIN & (1<<(portnum))); } 

//
// *** CLOCK ASSIGNMENTS ****
//

// currently FOsc = 14.7456MHz
// currently cclk = Fosc * 4 = 58.9824MHz
// currently pclk = cclk = 58.9824 MHz
// don't alter these without changing the PLL configuration!
//
const int FOsc = 14745600;
const int cclk = FOsc*4;		// multiply by 4 because of PLL
const int pclk = cclk;			// peripheral divider = 1

//
// System Config
//

/* 
PLL
 
 - Main clock F_OSC=14,7MHz @ Olimex LPC-P2106) [limits: 10 MHz to 25 MHz]
 - System should run at max. Frequency (60MHz) [limit: max 60 MHz]
 - Choose multiplier M=4 
	so cclk = M * F_OSC= 4 * 14745000Hz = 58980000 Hz
 - MSEL-Bit in PLLCFG (bits 0-4) MSEL = M-1
 - F_CCO must be inbetween the limits 156 MHz to 320 MHz
 datasheet: F_CCO = F_OSC * M * 2 * P
 - choose divider P=2 => F_CCO = 14745000Hz * 4 * 2 * 2
	= 235920000 ~=236 MHz
 - PSEL0 (Bit5 in PLLCFG) = 1, PSEL1 (Bit6) = 0 (0b01)
	*/
#define PLL_M		4				// multiplier
#define MSEL		(PLL_M-1)		// off by 1
#define PSEL0 		5
#define PSEL1 		6

#define PLLE		0				// pll enable
#define PLLC		1				// pll connect

#define PLOCK		10

#define PLL_FEED1	0xAA
#define PLL_FEED2	0x55

/*
 MAM(Memory Accelerator Module)
 - choosen: MAM fully enabled = MAM-Mode 2
 - System-Clock cclk=59MHz -> 3 CCLKs are proposed as fetch timing
 */
#define MAM_MODE 	2
#define MAM_FETCH   3

/*
 VPB (V... Pheriphal Bus)
 - choosen: VPB should run at full speed -> devider VPBDIV=1
 => pclk = cclk = 59MHz
 */
#define VPBDIV_VAL	1


/* 
SCB
*/
#define MEMMAP_BOOT_LOADER_MODE   0       // Interrupt vectors are re-mapped to Boot Block.
#define MEMMAP_USER_FLASH_MODE    (1<<0)  // Interrupt vectors are not re-mapped and reside in Flash.
#define MEMMAP_USER_RAM_MODE      (1<<1)  // Interrupt vectors are re-mapped to Static RAM.


//
// Additional constants
//

#define VICVectCntl_ENABLE (1<<5)

//
// Memory constants
//

extern unsigned int _stack;			// argh... global variable... :-)
// these are for the LPC2106
#define RAM_START        (0x40000000)
#define RAM_END          (RAM_START+(64*1024)-1)		// 64K after RAM start
#define FREE_STORE_START (reinterpret_cast<unsigned int>(&_stack))	// _end == _stack, '&' is because extern's are accessed via pointer
#define FREE_STORE_END   (RAM_END)
extern unsigned int _stack_bottom;
#define STACK_BOTTOM     (reinterpret_cast<unsigned int>(&_stack_bottom))
//
// **** PORT ASSIGNMENTS FOR THE MAIN BOARD ****
//


// use enums rather than const int ... allows us to have a specific type.
//
enum MAIN_proc_port_t { 
	// the ports have a specific number, therefore each one is numbered
	MAIN_pc_comms_tx_port = 0,
	MAIN_pc_comms_rx_port = 1,
	MAIN_spare_p2_port = 2,
	MAIN_spare_p3_port = 3,
	MAIN_spare_p4_port = 4,
	MAIN_spare_p5_port = 5,
	MAIN_spare_p6_port = 6,
	MAIN_motor1_enable_port = 7,
	MAIN_ipc_tx_port = 8,
	MAIN_ipc_rx_port = 9,
	MAIN_motor1_photo_interrupter_port = 10,
	MAIN_motor2_photo_interrupter_port = 11,
	MAIN_motor1_in2_backwards = 12,
	MAIN_motor1_current_limit_sense = 13,
	MAIN_bootloader_select_port = 14,
	MAIN_motor2_current_limit_sense = 15,
	MAIN_sound_output_port = 16,
	MAIN_board_identifier_port = 17,
	MAIN_spare_p18_port = 18,
	MAIN_spare_p19_port = 19,
	MAIN_spare_p20_port = 20,
	MAIN_motor2_enable_port = 21,
	MAIN_motor1_in1_forwards_port = 22,
	MAIN_motor2_in1_forwards_port = 23,
	MAIN_motor2_in2_backwards_port = 24,
	MAIN_blue_led_port = 25,
	MAIN_red_switch_1_port = 26,
	MAIN_yellow_led_port = 27,
	MAIN_yellow_switch_2_port = 28,
	MAIN_red_led_port = 29,
	MAIN_blue_switch_3_port = 30,
	MAIN_spare_p31_port = 31
};


//#define setbit(b) ;GPIO_IOSET=(1<<b);
//#define clrbit(b) ;GPIO_IOCLR=(1<<b);

// P0.0 - TxD0, PC comms (out)
// P0.1 - RxT0, PC comms (in)
// P0.2 - SCL, Camera I2C (out)
// P0.3 - SDA, Camera I2C (I/O)
// P0.4 - SCK, Butterfly comms
// P0.5 - MISO, Butterfly comms
// P0.6 - MOSI, Butterfly comms

// P0.7 - Motor 2 IN 2 (out) - backwards
//#define MOTOR_2_IN_2_PIN  7
#define MOTOR_2_IN_2_PIN MAIN_motor2_in2_backwards_port

// P0.8 - Motor 2 IN 1 (out) - forwards
#define MOTOR_2_IN_1_PIN MAIN_motor2_in1_forwards_port

// basic input selecting for motor 2
#define MOTOR_2_SELECT_FORWARD() ;GPIO_IOCLR=(1<<MOTOR_2_IN_1_PIN); GPIO_IOSET=(1<<MOTOR_2_IN_2_PIN);
#define MOTOR_2_SELECT_BACKWARD() ;GPIO_IOSET=(1<<MOTOR_2_IN_1_PIN); GPIO_IOCLR=(1<<MOTOR_2_IN_2_PIN);
#define MOTOR_2_SELECT_FAST_STOP() ;GPIO_IOCLR=(1<<MOTOR_2_IN_1_PIN); GPIO_IOCLR=(1<<MOTOR_2_IN_2_PIN);

// P0.9 - Motor 1 enable (out, active high, also green led, should be PWM)
//#define MOTOR_1_ENABLE_PIN  9
#define MOTOR_1_ENABLE_PIN MAIN_motor1_enable_port

#define ENABLE_MOTOR_1() ;GPIO_IOSET=(1<<MOTOR_1_ENABLE_PIN);
#define DISABLE_MOTOR_1() ;GPIO_IOCLR=(1<<MOTOR_1_ENABLE_PIN);

// P0.10 - Motor 1 photo interrupter
// P0.11 - Motor 2 photo interrupter

// P0.12 - Motor 1 IN 2 (out) - backwards
//#define MOTOR_1_IN_2_PIN  12
#define MOTOR_1_IN_2_PIN MAIN_motor1_in2_backwards

// P0.13 - Reset camera
// P0.13 - motor 1 current limit
#define QUICK_GET_MOTOR1_CURRENT_SENSE() (GPIO_IOPIN & (1<<(MAIN_motor1_current_limit_sense)))

// P0.14 - Bootloader select (active low on reset/power up) + status LED (red)
//#define LED_PIN 14
//#define ENABLE_LED() GPIO_IOSET=(1<<LED_PIN);
//#define DISABLE_LED() GPIO_IOCLR=(1<<LED_PIN);

// P0.15 - motor 2 current limit
#define QUICK_GET_MOTOR2_CURRENT_SENSE() (GPIO_IOPIN & (1<<(MAIN_motor2_current_limit_sense)))

// P0.16 - EINT0, Camera Pixel Clock Output (PCLK)
//#define PCLK_PIN 16
//#define PCLK() SIMPLE_GET_INPUT_MACRO(PCLK_PIN)

// P0.17 - Camera Odd Field Flag (FODD, high = off field)
//#define FODD_PIN 17
//#define FODD() SIMPLE_GET_INPUT_MACRO(FODD_PIN)

// P0.18 - Camera Horizontal Window Reference
//#define HREF_PIN 18
//#define HREF() SIMPLE_GET_INPUT_MACRO(HREF_PIN)

// P0.19 - Camera Vertical Sync Input
//#define VSYNC_PIN 19
//#define VSYNC() SIMPLE_GET_INPUT_MACRO(VSYNC_PIN)

// P0.20 - Camera Powerdown (PWDN)

// P0.21 - Motor 2 enable (out, active high, also red led, should be PWM)
#define MOTOR_2_ENABLE_PIN MAIN_motor2_enable_port
#define ENABLE_MOTOR_2() ;GPIO_IOSET=(1<<MOTOR_2_ENABLE_PIN);
#define DISABLE_MOTOR_2() ;GPIO_IOCLR=(1<<MOTOR_2_ENABLE_PIN);

// P0.22 - Motor 1 IN 1 (out)
//#define MOTOR_1_IN_1_PIN  22
#define MOTOR_1_IN_1_PIN MAIN_motor1_in1_forwards_port

// basic input selecting for motor 1
#define MOTOR_1_SELECT_FORWARD() ;GPIO_IOCLR=(1<<MOTOR_1_IN_1_PIN); GPIO_IOSET=(1<<MOTOR_1_IN_2_PIN);
#define MOTOR_1_SELECT_BACKWARD() ;GPIO_IOSET=(1<<MOTOR_1_IN_1_PIN); GPIO_IOCLR=(1<<MOTOR_1_IN_2_PIN);
#define MOTOR_1_SELECT_FAST_STOP() ;GPIO_IOCLR=(1<<MOTOR_1_IN_1_PIN); GPIO_IOCLR=(1<<MOTOR_1_IN_2_PIN);

// P0.23 - Camera D0
// P0.24 - Camera D1
// P0.25 - Camera D2
// P0.26 - Camera D3
// P0.27 - Camera D4
// P0.28 - Camera D5
// P0.29 - Camera D6
// P0.30 - Camera D7

//#define CAMERA_D0_PIN 23
//#define CAMERA_DATA() (GPIO_IOPIN >> (CAMERA_D0_PIN))

// P0.31 - Butterfly slave select (move to P0.7?)


// Old (unused) definitions:
//
// olimex LPC-P2106: one switch on P0.31 (active low)
//#define SWPIN 	31


//
// CMUCam3 defines
//
enum CAMERA_proc_port_t
{
	CC3_pc_comms_tx_port = 0,
	CC3_pc_comms_rx_port = 1,
	CC3_camera_hef_port = 2,
	CC3_fifo_reset_port = 3,
	CC3_sdcard_slot_spi_sck_port= 4,
	CC3_sdcard_slot_spi_miso_port = 5,
	CC3_sdcard_slot_spi_mosi_plus_led_port = 6,
	CC3_spare_p7_pull_up_port = 7,
	CC3_ipc_tx_port = 8,
	CC3_ipc_rx_port = 9,
	CC3_fifo_write_enable_port = 10,
	CC3_fifo_read_clk_port = 11,
	CC3_fifo_read_reset_port = 12,
	CC3_fifo_write_reset_port = 13,
	CC3_bootloader_select_sdcard_slot_spi_cs_port = 14,
	CC3_camera_reset_port = 15,
	CC3_camera_vsync_port = 16,
	CC3_board_identifier_port = 17,		// same port as MAIN
	CC3_spare_p18_port = 18,
	CC3_led2_port = 19,
	CC3_led3_port = 20,
	CC3_camera_power_down_port = 21,
	CC3_camera_serial_clock_port = 22,
	CC3_camera_serial_data_port = 23,
	CC3_camera_data0 = 24,
	CC3_camera_data1 = 25,
	CC3_camera_data2 = 26,
	CC3_camera_data3 = 27,
	CC3_camera_data4 = 28,
	CC3_camera_data5 = 29,
	CC3_camera_data6 = 30,
	CC3_camera_data7 = 31
};

#define bit_mask(bit_num) (1<<(bit_num))

//
// These emulate the names of the CC3 project just in case we need to use their code
//

// ideally we want to emulate them from above
//#define _CC3_CAM_VSYNC		bit_mask(CC3_CAM_VSYNC_pin) // 0x10000

// realistically we want to avoid coding :-)
#include "camera_board_hardware_defs.h"

#endif // ROBOT_HARDWARE_DEFS_H

