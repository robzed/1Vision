/* Enable and disable functions "stripped" from a sample by R O Software.
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact. */

#include "VIClowlevel.h"
#include "lpc210x_gnuarm.h"

#define IRQ_MASK 0x00000080

static inline unsigned asm_get_cpsr(void)
{
  unsigned long retval;
  asm volatile (" mrs  %0, cpsr" : "=r" (retval) : /* no inputs */  );
  return retval;
}

static inline void asm_set_cpsr(unsigned val)
{
  asm volatile (" msr  cpsr, %0" : /* no outputs */ : "r" (val)  );
}

unsigned enableIRQ(void)
{
  unsigned _cpsr;

  _cpsr = asm_get_cpsr();
  asm_set_cpsr(_cpsr & ~IRQ_MASK);
  return _cpsr;
}

unsigned disableIRQ(void)
{
  unsigned _cpsr;

  _cpsr = asm_get_cpsr();
  asm_set_cpsr(_cpsr | IRQ_MASK);
  return _cpsr;
}

unsigned restoreIRQ(unsigned oldCPSR)
{
  unsigned _cpsr;

  _cpsr = asm_get_cpsr();
  asm_set_cpsr((_cpsr & ~IRQ_MASK) | (oldCPSR & IRQ_MASK));
  return _cpsr;
}
/* end of R O code */

//
// Let's do the same for the FIQ
//

#define FIQ_MASK 0x00000040

//unsigned enableFIQ(void)
//{
//	unsigned _cpsr;
//	
//	_cpsr = asm_get_cpsr();
//	asm_set_cpsr(_cpsr & ~FIQ_MASK);
//	return _cpsr;
//}
//
//unsigned disableFIQ(void)
//{
//	unsigned _cpsr;
//	
//	_cpsr = asm_get_cpsr();
//	asm_set_cpsr(_cpsr | FIQ_MASK);
//	return _cpsr;
//}
//
//unsigned restoreFIQ(unsigned oldCPSR)
//{
//	unsigned _cpsr;
//	
//	_cpsr = asm_get_cpsr();
//	asm_set_cpsr((_cpsr & ~FIQ_MASK) | (oldCPSR & FIQ_MASK));
//	return _cpsr;
//}



#define MODE_MASK 0x1F			// bit used for mode

#define MODE_USR 0x10            // User Mode
#define MODE_FIQ 0x11            // FIQ Mode
#define MODE_IRQ 0x12            // IRQ Mode
#define MODE_SVC 0x13            // Supervisor Mode
#define MODE_ABT 0x17            // Abort Mode
#define MODE_UND 0x1B            // Undefined Mode
#define MODE_SYS 0x1F            // System Mode


static inline void asm_set_r9(unsigned val)
{
	asm volatile (" mov  r9, %0" : /* no outputs */ : "r" (val)  );
}

static inline void asm_set_r10(unsigned val)
{
	asm volatile (" mov  r10, %0" : /* no outputs */ : "r" (val)  );
}

static inline void asm_set_r11(unsigned val)
{
	asm volatile (" mov  r11, %0" : /* no outputs */ : "r" (val)  );
}

static inline void asm_set_r12(unsigned val)
{
	asm volatile (" mov  r12, %0" : /* no outputs */ : "r" (val)  );
}

static inline void asm_set_r13(unsigned val)
{
	asm volatile (" mov  r13, %0" : /* no outputs */ : "r" (val)  );
}

//
//
//

static inline unsigned asm_get_r10()
{
	unsigned long retval;
	asm volatile (" mov  %0, r10" : "=r" (retval) : /* no inputs */  );	
	return retval;
}

//#define SCB_EXTINT_ADDRESS 0xE01FC140
//#define GPIO_IOPIN_ADDRESS 0xE0028000
#define SCB_EXTINT_ADDRESS (&(SCB_EXTINT))
#define GPIO_IOPIN_ADDRESS (&(GPIO_IOPIN))
#define IRQ0_CLEAR_BITMASK (1)

//void setup_FIQ_registers(byte* buffer_address, byte* buffer_end_address)
//{
//	disableFIQ();		// ensure disabled. Bad things might happen otherwise.
//
//	// get us into FIQ mode so we can access the FIQ banked reqisters r8 - r14
//	unsigned _cpsr = asm_get_cpsr();
//	asm_set_cpsr((_cpsr & ~MODE_MASK) | (MODE_FIQ));
//	
//	asm_set_r9(reinterpret_cast<unsigned int>(GPIO_IOPIN_ADDRESS));
//	asm_set_r10(reinterpret_cast<unsigned int>(buffer_address));
//	asm_set_r11(reinterpret_cast<unsigned int>(buffer_end_address));
//	asm_set_r12(IRQ0_CLEAR_BITMASK);						// interrupt bit 0 to clear
//	asm_set_r13(reinterpret_cast<unsigned int>(SCB_EXTINT_ADDRESS));	// overrite stack here!
//			
//	// put us back into system mode
//	_cpsr = asm_get_cpsr();
//	asm_set_cpsr((_cpsr & ~MODE_MASK) | (MODE_SYS));
//
//}


//byte* get_final_pointer()
//{
//	byte* address;
//	
//	disableFIQ();
//
//	// get us into FIQ mode so we can access the FIQ banked reqisters r8 - r14
//	unsigned _cpsr = asm_get_cpsr();
//	asm_set_cpsr((_cpsr & ~MODE_MASK) | (MODE_FIQ));
//	
//	address = reinterpret_cast<byte*>(asm_get_r10());
//	
//	// put us back into system mode
//	_cpsr = asm_get_cpsr();
//	asm_set_cpsr((_cpsr & ~MODE_MASK) | (MODE_SYS));
//	
//	return address;
//}
//
