/*
 *  hardware_support.cpp
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

#include "hardware_support.h"
#include "lpc210x_gnuarm.h"
#include "robot_hardware_defs.h"


// +-------------------------------+-------------------------+-----------------------
// | TITLE:        
// | AUTHOR(s):    Rob & Alan
// | DATE STARTED: 5-7-2006
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void pin_function_select(int port, int function)
{
	if(port<0 || port>31) return;
	if(function<0 || function>3) return;
	
	int portsel_bitnumber = (port&0x0F) << 1;
	int portsel_mask = 0x03 << portsel_bitnumber;
	
	if(port<=15)
	{
		int portsel_copy = PCB_PINSEL0;
		portsel_copy &= ~portsel_mask;
		portsel_copy |= function << portsel_bitnumber;
		PCB_PINSEL0 = portsel_copy;
	}
	else
	{
		int portsel_copy = PCB_PINSEL1;
		portsel_copy &= ~portsel_mask;
		portsel_copy |= function << portsel_bitnumber;
		PCB_PINSEL1 = portsel_copy;
	}
}


// +-------------------------------+-------------------------+-----------------------
// | TITLE:        
// | AUTHOR(s):    Rob & Alan
// | DATE STARTED: 12-10-2006
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----


//
// from Blinky.c (love to M.Thomas)
//

void processor_speed_init()
{
	// --- enable and connect the PLL (Phase Locked Loop) ---
	// a. set multiplier and divider
	SCB_PLLCFG = MSEL | (1<<PSEL1) | (0<<PSEL0);			// P = 2 (values = 1, 2, 4, 8)
	// b. enable PLL
	SCB_PLLCON = (1<<PLLE);
	// c. feed sequence
	SCB_PLLFEED = PLL_FEED1;
	SCB_PLLFEED = PLL_FEED2;
	// d. wait for PLL lock (PLOCK bit is set if locked)
	while ( !( SCB_PLLSTAT & (1<<PLOCK) ) );
	// e. connect (and enable) PLL
	SCB_PLLCON = (1<<PLLE) | (1<<PLLC);
	// f. feed sequence
	SCB_PLLFEED = PLL_FEED1;
	SCB_PLLFEED = PLL_FEED2;
	
	// --- setup and enable the MAM (Memory Accelerator Module) ---
	// a. start change by turning off the MAM (redundant)
	MAM_MAMCR = 0;
	// b. set MAM-Fetch cycle to 3 cclk as recommended for >40MHz
	MAM_MAMTIM = MAM_FETCH;
	// c. enable MAM 
	MAM_MAMCR = MAM_MODE;

	
	// --- set VPB speed ---
	SCB_VPBDIV = VPBDIV_VAL;

    // --- map INT-vector ---

// commented out by Rob
//#if defined(RAM_RUN)
//	SCB_MEMMAP = MEMMAP_USER_RAM_MODE;
//#elif defined(ROM_RUN)
//	SCB_MEMMAP = MEMMAP_USER_FLASH_MODE;
//#else
//#error RUN_MODE not defined!
//#endif
}


// +-------------------------------+-------------------------+-----------------------
// | TITLE:        StartISP
// | AUTHOR(s):    Rob
// | DATE STARTED: 07-12-2006
// +
// | DESCRIPTION:  
// +----------------------------------------------------------------ROUTINE HEADER----

void StartISP(unsigned long wdticks)
{
	typedef void (*void_func_t)(void);
    void_func_t bootloader_entry = reinterpret_cast<void_func_t>(0);
	
    /* reset PINSEL (set all pins to GPIO) */
    PCB_PINSEL0 = 0x00000000;
    PCB_PINSEL1 = 0x00000000;
	
    /* reset GPIO, but drive P0.14 low (output) */
    GPIO_IODIR = 0x00004000;    /* P0.14 = output, others are input */
    GPIO_IOCLR = 0x00004000;
	
    /* power up all peripherals */
    SCB_PCONP = 0x000003be;     /* for LPC2104/5/6
		* use 0x001817be for LPC2131/2/8 */
	
    /* disconnect PLL */
    SCB_PLLCON = 0x00;
    SCB_PLLFEED = 0xaa; SCB_PLLFEED = 0x55;
	
    /* set peripheral bus to 1/4th of the system clock */
    SCB_VPBDIV = 0x00;
	
    /* map bootloader vectors */
    SCB_MEMMAP = 0;
	
    /* optionally set up a watchdog timer to exit ISP mode */
    if (wdticks != 0) {
        WD_WDTC = wdticks;
        WD_WDMOD = 0x03;
        WD_WDFEED = 0xaa; WD_WDFEED = 0x55;
    } /* if */

    /* jump to the bootloader address */
    bootloader_entry();
}

/*

LPC2100: Entering ISP mode from user code

"Entering ISP mode from user code" is the title of NXP application note AN10356. Here, I explain the trick and simplify the code from the application note. The focus is on an "in-field" firmware update procedure.

The LPC2100 microcontroller series is built on the ARM7 core and provides a generous assortment of on-chip peripherals. To load the firmware into its internal Flash ROM, the LPC2100 series support In-System Programming (ISP) via a bootloader and In-Application Programming for updating firmware from code that runs in SRAM. In-System Programming is an attractive option for in-field firmware update, because the bootloader supports code uploading via an RS232.

The standard way to start ISP is to hold pin P0.14 low while issuing a processor reset. Many designs provide two switches (or a jumpers) for the P0.14 and reset lines. For an in-field update, this may be inconvienent ("cumbersome" is the word NXP uses in relation to this procedure) because you will have to ask the customer to push these switches (and to release the reset switch before releasing the P0.14 switch). The "LPC2000 Flash ISP Utility" supports a design (which they describe in the application note AN10302) where the RTS signal (from the RS232 plug) is wired to the reset and DTR is wired to P0.14. The LPC2000 Flash Utility can then toggle the RTS and DTR lines in the appropriate sequence. This, in turn, may also become cumbersome if you will want to use the serial port of the LPC2000 microcontroller for other purposes too (i.e. other than just firmware update). If a serial port is provided for anyway, most designs can find a useful task for it. In that case, however, you cannot simply assume that the external hardware (e.g. a simple terminal, a barcode scanner, a ticket printer) does not use the DTR and RTS signals; if it does, it might inadvertently reset the microcontroller or toggle the P0.14 pin which is likely to be used for a completely unrelated purpose in field operation. The assertion of NXP that the RTS and DTR signals are "unused" refers only to the serial port design of the LPC2000 series. Computer systems and peripherals that communicates through RS232 interfaces do toggle the RTS and DTR signals. Obviously, if your design hard-wires these signals to reset and P0.14, this will lead to "interesting" support calls.

In brief, the two options for firmware updates discussed so far are both undesirable:

> Asking the user to enter ISP mode by pushing switches that you provide on your circuit specifically for this purpose is inconvenient and error prone.
Applying a modified serial port carries more problems than that it solves; it may be convenient during development but it is inappropriate for the end product.
There has always existed a third option, "In Application Programming", but this is quite a bit more difficult to implement, especially if your microcontroller has far more Flash ROM than SRAM (e.g. the LPC2138).

Then came application note AN10356: a programmatic switch to ISP. The advantage of this design is that your firmware can use the RS232 itself for any particular purpose, and decide to switch to ISP (for a firmware update) on receiving a special instruction. No switches need to be pressed by the customer doing the firmware update, and no danger exists in external hardware to toggle "reserved" lines. No special circuitry needs to be attached to the microcontroller either. And the use of the bootloader in ISP mode allows a simple firmware update program; you can even use the existing utilities, such as the "LPC2000 Flash ISP Utility".

How it works
The NXP manuals and the application note AN10302 describe in detail the operation of the LPC2000 after a reset. In brief, after approximately 3 ms it checks the status of pin P0.14 and decides whether to branch to ISP mode or to proceed with the user code (i.e. "firmware") already in Flash ROM. (I am ignoring the special case where there is no valid user program in Flash ROM and the case of a reset invoked from the watchdog timer.) If you jump, from inside the user code, to the reset address, the bootloader code of the microcontroller will behave as if it came from a reset, and check pin P0.14 to optionally enter ISP. The trick is to configure pin P0.14 as "output" and to drive it low. The bootloader code will then still see it low 3 ms later.

On a true reset, all gpio pins will have been toggled to "input" and the only way that pin P0.14 can be down is through a switch, a jumper or external logic. Jumping to the bootloader address is not a true reset, however, and if your firmware configures pin P0.14 as "output" (and drives it low), it will still be a "low output" when the bootloader code checks the pin. That is, the bootloader does not re-configure the pin's I/O direction. Note that you can still read a gpio pin's value if it is configured as output: the iopin register will just return the same value that you wrote to the pin earlier.

The remainder of the necessary code to enter ISP mode is to return the microcontroller to its state as if it came from a reset. This is needed because the bootloader code assumes that the microcontroller is in that state. So the code snippet below configures all pins to gpio, powers-up all devices, disconnects the Phase Locked Loop (PLL) and sets the peripheral bus to low speed, and then jumps to the reset address.

<<<PROGRAM WAS HERE>>>

Your firmware can now enter ISP mode by simply calling StartISP(). While running user code, the firmware also needs to check for some criterion to start ISP mode. A straigtforward solution is to accept a special request on the serial port, possibly with a challenge/handshake protocol to avoid unintentional activation of ISP mode.

Apart from setting the watchdog timer, which I will come to in a moment, the code snippet on this page is quite in line with that in application note AN10356. My version resets a few more registers (which makes my version a little more robust) and I do not bother keeping some pins or registers in a non-reset state (which makes the code shorter and simpler).

Obviously, if you want to add all this in a stream-lined firmware update procedure, the "LPC2000 Flash ISP Utility" by itself will not do: it does not know how to enter ISP mode from your code. You can, of course, make a tiny program that starts ISP mode and then launches the NXP/Philips utility. As documented on the web page for the lpc21isp utility, the NXP/Philips Flash utility can take its options from the command line. Alternatively, you can build your own upload utility --the protocol is fairly simple.

Exiting ISP mode
There is one slight complication in the procedure described so far: after having refreshed your firmware, you will want to restart it, and run automatically with the new code. The LPC2100 bootloader has a special command for jumping to an address that you specify: "G <address>". What the lpc21isp utility does is to jump to address 0; this re-invokes the bootloader which, in the normal case, finds the correct signature and pin P0.14 up, so that it jumps to the user code. After our trick, however, jumping to address 0 is not going to work: pin P0.14 is still down. The only option that I have found that works reliably is to cause a reset from the Watchdog Timer. The bootloader does not "pulse" the watchdog timer and therefore the watchdog timer will "fire" after the configured number of ticks and reset the microcontroller: a true reset, which toggles pin P0.14 to input mode, so that the bootloader will see it "up" and proceed with the brand new firmware.

I have left it to the caller of the code to determine the number of ticks that the watchdog timer must delay before resetting the chip. It is often easiest to configure this count from the firmware update program, e.g. during the handshaking protocol to enter ISP mode that I mentioned earlier. To set the appropriate value, you need to determine the tick rate of the watchdog timer, and the time that you need to load the new firmware in the microcontroller. Probably you will want to add a savety margin to avoid resetting the chip before the update has completed.

The watchdog timer ticks at a rate of 1/4th of the peripheral clock, StartISP() has just reset to the peripheral clock to run at 1/4th of the CPU clock, and the CPU clock is reset to the frequency of the crystal. Therefore, the watchdog timer tick rate is the crystal frequency divided by 16. Note that your firmware update program will need to know the crystal frequency anyway, because it is part of the handshaking protocol of the bootloader itself.

The other required estimate is the time needed to update the firmware, which depends on the size of the firmware and the serial protocol --notably the baud rate. The boot loader uses a frame with 8 data bits, 1 stop bit and no parity; thus, including the start bit, the serial frame is 10 bytes. At 9600 Baud, you will transmit 960 bytes per second. The data must be in UU-encoded format and this format expands the binary image data by approximately a factor 1.37 (4 ASCII characters for every three bytes, plus a length byte for every 45 bytes and a checksum for every 20 lines, or 900 bytes). However, since the upload utility also has to issue instructions to prepare and erase sectors (before writing them), to sail on the safe side, a factor of 1.6 or 1.7 is more suitable. I recommend that you measure the required upload time at the Baud rate that you wish to use, and add a few seconds for the handshaking protocol for the bootloader and the occasional lost packet.

Further reading & references
AN10302 - Using the Philips LPC2000 Flash utility with the Keil MCB2100 and IAR LPC210x Kickstart evaluation boards
A description of the wiring for using RTS/DTR to switch to the bootloader.
LPC2000 Flash ISP Utility
The "LPC2000 Flash ISP Utility".
lpc21isp
An Open Source alternative Flash uploader utility for, amongst others, the LPC2100 series, which runs under Windows and Linux; by Martin Maurer.


LINKS

http://www.standardics.nxp.com/support/documents/microcontrollers/pdf/an10356.pdf

http://www.standardics.nxp.com/support/documents/microcontrollers/pdf/an10302.pdf



*/
