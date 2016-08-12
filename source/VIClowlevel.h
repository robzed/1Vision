
#ifndef VICLOWLEVEL_H
#define VICLOWLEVEL_H

#include "robot_basic_types.h"

//
// functions to manipulate IRQ
//
unsigned enableIRQ(void);
unsigned disableIRQ(void);
unsigned restoreIRQ(unsigned oldCPSR);

//
// functions to manipulate FIQ
//
//unsigned enableFIQ(void);
//unsigned disableFIQ(void);
//unsigned restoreFIQ(unsigned oldCPSR);

//
// Set up the FIQ registers
//
//void setup_FIQ_registers(byte* buffer_address, byte* buffer_end_address);
//byte* get_final_pointer();

#endif // VICLOWLEVEL_H


