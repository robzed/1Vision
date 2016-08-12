/*
 *  language_support.cpp
 *  robot_core
 *
 *  Created by Rob Probin on 18/02/2007.
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
 */

#include "robot_hardware_defs.h"
#include "language_support.h"
#include "pc_uart.h"
#include "robot_basic_types.h"


#define DEBUGGING_ENABLED 0			// more allocator debugging support see dwrite... statements
#include "debug_support.h"

// select which allocator you want (just one!)
#define FULL_REUSE_MEMORY_ALLOCATOR 0	// turn on the full reuse memory allocator?
#define SIMPLE_ALLOCATOR 0
#define ROBS_OTHER_MEMORY_ALLOCATOR 1
#define SLOT_BASED_FIRST_FIT_ALGORITHM 0

// other defines that help
#define PRINT_NEW_DELETE_ALLOCATIONS 0

#define ALLOW_SIMPLE_ALLOCATOR_TO_DEALLOC_ONCE 1		// if the block to free is the last block allocated we can deallocate...

#define LINK_LIST_ADVANCED_DEBUG 0

// +-------------------------------+-------------------------+-----------------------
// | TITLE:		   
// | AUTHOR(s):    Rob Probin
// | DATE STARTED: 6 April 2007
// +
// | DESCRIPTION:  Stop totally ...
// +----------------------------------------------------------------ROUTINE HEADER----

void halt()
{
	halt("");
}

//
// Halt with a message (for debug)
//
void halt(const char* error_message)
{
	volatile int i,j;
	
	// repeat message
	while(1) {
		write_line(error_message);
		write_line("*HALT*");
		
		for (i=0;i<1000;i++)
			for (j=0;j<10000;j++) /* nop */ ;
	}
	
}


// return the estimated current user stack ... if in user mode
// there is actually slightly more than this because the bottom of the stack is aligned 
// 256 from the bss_end. 
unsigned long user_stack_free()
{
	unsigned long sp_value;
	asm volatile (" mov  %0, sp" : "=r" (sp_value) : /* no inputs */  );	

	return sp_value-STACK_BOTTOM;
}

#if !INCLUDE_DYNAMIC_MEMORY

void init_dynamic_memory()
{
}

#else // this section is for INCLUDE_DYNAMIC_MEMORY

#if FULL_REUSE_MEMORY_ALLOCATOR

// Very simple (and brain dead) memory allocation to support new and delete. No
// plans on using this except for command line debugging and classes.
//
// NOTES
// 1. This version always returns a valid pointer, even for malloc(0) - so that new(0) is
// compliant to standards. Pointers will be unique.
// 2. Start pointer is four byte aligned memory always.
// 3. We don't support all memory primartives of Std C lib (e.g. realloc and calloc)
// 4. Minimum allocated size is 4 bytes (to maintain alignment)
// 5. Failure to allocate will return 0. This means new needs to check.
// 6. Memory pool comes from the RAM unallocated by the compiler in the micro.
// 8. May well suffer with memory fragmentation. If this becomes a problem, use
// the buddy algorithm (although this loses some memory to try to avoid 
// fragmentation - "slack space").
// 9. No background clean up required.
// 10. Tries to be memory efficent rather than fast.
// 11. Overhead per allocation = 4 bytes
// 12. free assumes 
//
// Other references:
// http://en.wikipedia.org/wiki/Dynamic_memory_allocation
// http://www.osdcom.info/content/view/31/39/
// 

const void* mem_start = ((FREE_STORE_START+3) & ~0x03) + 4;	// ensure 4 byte aligned and leave space for first link

const unsigned char mem_free = 0;
const unsigned char mem_allocated = 1;

enum { mem_free, mem_allocated };

static void create_link(void *ptr, unsigned char type, size_t size, size_t previous_offset, bool last);
{
}

static bool get_link(void *ptr, unsigned char* type, size_t* size, size_t* previous_offset)
{
}

void init_dynamic_memory()
{
	create_link(mem_start, mem_free, (FREE_STORE_END - i_mem_start) - 4, 0, true);
}

// each link contains:
// 1. type of link (free, allocated)
// 2. size of this block
//
// Possible bit structure
//  Since we have 
//    (a) an int32 for memory management data storage (ideal)
//    (b) a maximum (on this CPU) of 64K bytes data to access as the free_store
//    (c) all blocks are rounded up to 4 bytes (int32) for alignment purposes
//	
// Therefore:
//
// 14 bits - this block size (gives 64K forward)
// 14 bits - previous int32 offset (gives 64K back) (=0 no previous block)
// 1 bit - last block
// 1 bit - allocated block
// 2 bits - spare
//
#define MAX_ALLOCATION_SIZE 0xffffff			// since we have only 64K this shouldn't be a problem

void *malloc( size_t size )
{
	// only allow up to a certain size to be allocated
	if(size > MAX_ALLOCATION_SIZE) { return 0; }
	
	// search for a free block
	void* mem_ptr = mem_start;
	void* candidate_ptr = 0;
	void* candidate_size = MAX_ALLOCATION_SIZE+1;
	unsigned char type;
	size_t blksize;
	bool last;
	do {
		last = get_link(mem_ptr, &type, &blksize);
		if(type == mem_free)
		{
			if(blksize > size && blksize < candidate_size)
			{
				candidate_ptr = mem_ptr;
				candidate_size = blksize;
			}
		}
	} while(!last);
	
	if(candidate_ptr)
	{
		// need to split block and create new links
		create_link(candidate_ptr, mem_allocated, ((size+3)&~0x03), false);
		create_link(candidate_ptr+((size+3)&~0x03)+4, mem_free, ,last);
	}
	
	return candidate_ptr; // which might be zero
}


void free(void* ptr)
{
	if(ptr)	// check for NULL pointer
	{
		// check for stupid values ... and halt
		if(ptr < FREE_STORE_START || ptr > FREE_STORE_END)
		{
			halt("free");		// something is wrong with the code
		}
	
		// get this block data
		unsigned char type;
		size_t blksize;
		bool last;
		last = get_link(ptr, &type, &blksize);
		create_link(ptr, mem_free, blksize, last);	// free up that block

		// if next or previous are free we should merge these in...

		// next blocks are easy ... just scan forward from this position...
		
		// last blocks are a PITA ... because they require scanning from the beginning...
		
		// unless we can use the unused bits for something clever? e.g. a block count?
		
		// actually we have 14 bits free (because largest theoretical allocation on this chip is 65535
		
		// 
	}
}

void memstat(memstat_data_t* returned_memory_statistics)
{
}

size_t total_free()
{
}

size_t largest_free()
{
}

#elif SIMPLE_ALLOCATOR

// Simple no-reuse allocator
//
// DESCRIPTION: This allocates memory and never reuses previously freed memory blocks.
// This means we will run out, but since this is an embedded system that is likely 
// to get reset very often and assuming this leaves us with a VERY simple memory
// allocator.
//
// NOTES:
//
// 1. This version always returns a valid pointer, even for malloc(0) - so that new(0) is
// compliant to standards. Pointers will be unique.
// 2. Start pointer is four byte aligned memory always.
// 3. We don't support all memory primartives of Std C lib (e.g. realloc and calloc)
// 4. Minimum allocated size is 4 bytes (to maintain alignment and ensure pointers are unique)
// 5. Failure to allocate will return 0. This means new needs to check.
// 6. Memory pool comes from the RAM unallocated by the compiler in the micro.
// 7. free() 'checks' for a null pointer ... to support delete(0) operation.
// 8. <<removed note>>
// 9. No background clean up required.
// 10. <<removed note>>
// 11. Overhead per allocation = 0 bytes.
// 12. <<removed note>>
// 13. We assume new does the right thing if we can't get memory and malloc returns NULL. 
//
// Other references:
// http://en.wikipedia.org/wiki/Dynamic_memory_allocation
// http://www.osdcom.info/content/view/31/39/
// 
// See the last link for the source of the idea.
//

// general constants
byte * const mem_start = reinterpret_cast<byte*>(FREE_STORE_START);
const size_t total_free_store_memory = FREE_STORE_END - FREE_STORE_START + 1;

// control variables
static byte* current_allocated_memory = mem_start;	// set at start of block
static unsigned int blocks_allocated = 0;

#if ALLOW_SIMPLE_ALLOCATOR_TO_DEALLOC_ONCE
static byte* last_allocated_block=0;				// Added to allow deallocation of previous block
#endif

void init_dynamic_memory()
{
	// nothing to do here...
}

static inline size_t align_int32(size_t i)
{
	return ((i+3)&~0x03);
}

void memstat(memstat_data_t* returned_memory_statistics)
{
	returned_memory_statistics->total_mem = total_free_store_memory;
	returned_memory_statistics->space_available = total_free();
	returned_memory_statistics->num_blocks_allocated = blocks_allocated;
	returned_memory_statistics->largest_unallocated = largest_free();
	returned_memory_statistics->number_of_gaps = 1;	// always one free
	
	returned_memory_statistics->ram_start = RAM_START;
	returned_memory_statistics->ram_end = RAM_END;
	returned_memory_statistics->free_store_start = FREE_STORE_START;
	returned_memory_statistics->free_store_end = FREE_STORE_END;
}

//
// For the simple no reuse allocator the total memory available is always the remaining 
// block (which is the same as the largest unallocated block).
//
size_t total_free()
{
	return reinterpret_cast<byte*>(FREE_STORE_END) - current_allocated_memory + 1;
}

//
// For the simple no reuse allocator the largest unallocate total is always the remaining 
// block (which is the same as the total memory available block).
//
size_t largest_free()
{
	return reinterpret_cast<byte*>(FREE_STORE_END) - current_allocated_memory + 1;
}

void *malloc(size_t size)
{
	if(size==0) size=1;			// support requirement for unique pointers and requirement for allocation of 0 to succeed.

	size = align_int32(size);	// ensure 4 byte boundary for allocations

	if(largest_free() < size) return 0;
	byte *memory_ptr = current_allocated_memory;
	current_allocated_memory += size;

	blocks_allocated++;

#if ALLOW_SIMPLE_ALLOCATOR_TO_DEALLOC_ONCE
	last_allocated_block = memory_ptr;
#endif

	return memory_ptr;
}


void free(void* ptr)
{
	// mostly do nothing here (passing in zero is fine!)

#if ALLOW_SIMPLE_ALLOCATOR_TO_DEALLOC_ONCE
	if(ptr==0) return;									// 0=null pointer, don't do anything - also  can't allow this comparison since last_allocated_block=0 has special meaning (last block not known)
	if(ptr == last_allocated_block)
	{
		blocks_allocated--;									// one less block has been allocated
		current_allocated_memory = last_allocated_block;	// we can move the memory pointer back to the previous position
		last_allocated_block = 0;							// don't know previous last block
	}

#endif

}

#elif ROBS_OTHER_MEMORY_ALLOCATOR


// 14 bits - this block size (gives 64K forward)
// 14 bits - previous int32 offset (gives 64K back) (=0 no previous block)
// 1 bit - last block
// 1 bit - allocated block
// 2 bits - spare

// general constants
byte * const mem_start = reinterpret_cast<byte*>((FREE_STORE_START+3)&~0x03);	// ensure 4 byte aligned
const size_t total_free_store_memory = FREE_STORE_END - FREE_STORE_START + 1;

#define USE_DEBUGGABLE_HEADER_VERSION 1

#if USE_DEBUGGABLE_HEADER_VERSION

struct mem_data_header
{
	unsigned short next_offset_plus_last;				// offset from next header start. Size allocated/free = this-(sizeof(mem_data_header))
	unsigned short previous_offset_plus_allocated;	// 0=no previous block, offset from header start
													// can't do away with last (including it in next_block_offset) because last block might have a size ... unless we have a false header at the end.
};
const unsigned short next_mask = 0xfffc;
const unsigned short previous_mask = 0xfffc;
const unsigned short last_mask = 0x0001;
const unsigned short allocated_mask = 0x0001;

#if LINK_LIST_ADVANCED_DEBUG
void print_raw_header(byte* header_address)
{
	write_string(" next=");
	write_hex(header_address[1]); write_string(" ");
	write_hex(header_address[0] & next_mask); write_string(" previous=");
	write_hex(header_address[3]); write_string(" ");
	write_hex(header_address[2] & previous_mask); write_string(" last=");

	write_hex(header_address[0]&last_mask); write_string(" allocated=");
	write_hex(header_address[2]&allocated_mask);	
}
#endif


//typedef byte mem_data_header[4];			// replacement without alignment problems


//
// A bunch of routines to query the header information
//
bool is_allocated(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	return header->previous_offset_plus_allocated & allocated_mask;
}

bool is_free(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	return !(header->previous_offset_plus_allocated & allocated_mask);
}

byte* get_previous(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	// the previous address is the offset, multiplied into int32s then subtracted from the 
	// from the header. This gives the previous user block memory address.
	return header_address - (header->previous_offset_plus_allocated & previous_mask);
}

byte* get_next(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	// the next address is the offset, multiplied into int32s then added to the 
	// header. 
	return header_address + (header->next_offset_plus_last & next_mask);
}

size_t get_size(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	// the size is the next offset multiplied into int32s with the 
	// size of the data header taken off
	return (header->next_offset_plus_last & next_mask) - sizeof(mem_data_header);
}

bool is_last(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	return header->next_offset_plus_last & last_mask;	
}

bool is_first(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	// the previous address is the offset, multiplied into int32s then subtracted from the 
	// from the header. This gives the previous user block memory address.
	return (header->previous_offset_plus_allocated & previous_mask) == 0;
}


//
// A bunch of routines to set the header information
//
void make_allocated(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	header->previous_offset_plus_allocated |= allocated_mask;
}

void make_free(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	header->previous_offset_plus_allocated &= ~allocated_mask;
}

void set_previous(byte* header_address, byte* previous_block_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	header->previous_offset_plus_allocated &= ~previous_mask;
	header->previous_offset_plus_allocated |= previous_mask & (header_address - previous_block_address);
}

void set_next(byte* header_address, byte* next_block_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer

	header->next_offset_plus_last &= ~next_mask;
	header->next_offset_plus_last |= next_mask & (next_block_address - header_address);
}

void set_size(byte* header_address, size_t size)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	

	header->next_offset_plus_last &= ~next_mask;
	header->next_offset_plus_last |= next_mask & (size+sizeof(mem_data_header));
}

void set_last(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	header->next_offset_plus_last |= last_mask;
}

void set_not_last(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	header->next_offset_plus_last &= ~last_mask;
}

void set_first(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	header->previous_offset_plus_allocated &= ~previous_mask;
}

#else

struct mem_data_header
{
	//unsigned int this_block_size:14;
	unsigned int next_block_offset:14;		// offset from next header start. Size allocated/free = this-(sizeof(mem_data_header))
	unsigned int previous_block_offset:14;	// 0=no previous block, offset from header start
	bool last_block:1;			// can't do away with this (including it in next_block_offset) because last block might have a size ... unless we have a false header at the end.
	bool allocated:1;
	// spare:2
};

//typedef byte mem_data_header[4];			// replacement without alignment problems


//
// A bunch of routines to query the header information
//
bool is_allocated(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	return header->allocated;
}

bool is_free(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	return !header->allocated;
}

byte* get_previous(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	// the previous address is the offset, multiplied into int32s then subtracted from the 
	// from the header. This gives the previous user block memory address.
	return header_address - (header->previous_block_offset*4);
}

byte* get_next(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	// the next address is the offset, multiplied into int32s then added to the 
	// header. 
	return header_address + (header->next_block_offset*4);
}

size_t get_size(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	// the size is the next offset multiplied into int32s with the 
	// size of the data header taken off
	return (header->next_block_offset*4) - sizeof(mem_data_header);
}

bool is_last(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	return header->last_block;	
}

bool is_first(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	// the previous address is the offset, multiplied into int32s then subtracted from the 
	// from the header. This gives the previous user block memory address.
	return header->previous_block_offset==0;
}


//
// A bunch of routines to set the header information
//
void make_allocated(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	header->allocated = true;
}

void make_free(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	header->allocated = false;
}

void set_previous(byte* header_address, byte* previous_block_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	header->previous_block_offset = (header_address - previous_block_address) >> 2;
}

void set_next(byte* header_address, byte* next_block_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	header->next_block_offset = (next_block_address - header_address) >> 2;
}

void set_size(byte* header_address, size_t size)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	header->next_block_offset = (size+sizeof(mem_data_header))>>2;	
	//header->next_block_offset = size>>2+sizeof(mem_data_header);
}

void set_last(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	header->last_block = true;
}

void set_not_last(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;
	
	header->last_block = false;
}

void set_first(byte* header_address)
{
	mem_data_header* header = (mem_data_header*) header_address;	// and make the right sort of pointer
	
	header->previous_block_offset = 0;
}



#if LINK_LIST_ADVANCED_DEBUG
void print_raw_header(byte* header_address)
{
	write_hex(header_address[0]); write_string(" ");
	write_hex(header_address[1]); write_string(" ");
	write_hex(header_address[2]); write_string(" ");
	write_hex(header_address[3]); 
}
#endif

#endif
//
// A routine to return the user memory address
//
byte* get_user_memory_block_address(byte* this_block)
{
	//byte address = (byte*)this_block;
	//address += sizeof(mem_data_header);
	//return address;
	if(this_block)		// don't do it for zero returned blocks!!!
	{
		this_block += sizeof(mem_data_header);
	}
	return this_block;
}

//
// A routine to return the header address
//
byte* get_header_address(byte* user_address)
{
	return user_address - sizeof(mem_data_header);
}

//
//
//
static inline byte* align_int32(byte* p)
{
	unsigned long i = (unsigned long) p;
	i = ((i+3)&~0x03);
	
	return (byte*)i;
}

byte* memory_root_pointer;			// relies on init to setup this and first free header here


void init_dynamic_memory()
{
	dwrite_line("CALL:init_dynamic_memory()");
	memory_root_pointer = align_int32(mem_start);		// ensure aligned
	int mem_alignment_waste = memory_root_pointer - mem_start;
	set_last(memory_root_pointer);
	set_first(memory_root_pointer);
	make_free(memory_root_pointer);
	set_size(memory_root_pointer, total_free_store_memory-(mem_alignment_waste+sizeof(mem_data_header)));
	
	dwrite_string("mem_start="); dwrite_int((int)mem_start); dwrite_string(" = 0x"); dwrite_hex((int)mem_start); dwrite_cr();
	dwrite_line("memory_root_pointer=", (int)memory_root_pointer);
	dwrite_line("total_free_store_memory=", (int)total_free_store_memory);
	dwrite_line("sizeof(mem_data_header)=", (int)sizeof(mem_data_header));
	dwrite_line("mem_alignment_waste=", (int)mem_alignment_waste);
	dwrite_line("calculated mem size=", (int)total_free_store_memory-(mem_alignment_waste+sizeof(mem_data_header)));
	dwrite_line("set mem size=", (int)get_size(memory_root_pointer));
	dwrite_line("RETURN:init_dynamic_memory() ");
}

size_t blocks_allocated()
{
	dwrite_line("CALL:blocks_allocated()");
	byte* current = memory_root_pointer;
	
	int count = 0;
	
	// always at least one
	bool is_not_last;
	do
	{
		dwrite_line("Looking at block ", (int)current);
		if(is_allocated(current))
		{
			count++;
			dwrite_line("Block is allocated, count =", count);
		}
		
		is_not_last = !is_last(current);
		current = get_next(current);
	} while(is_not_last);
	
	dwrite_line("RETURN:blocks_allocated() with ", count);
	
	return count;
}


size_t blocks_free()
{
	dwrite_line("CALL:blocks_free()");
	byte* current = memory_root_pointer;
	
	int count = 0;
	
	// always at least one
	bool is_not_last;
	do
	{
		dwrite_line("Looking at block ", (int)current);
		if(is_free(current))
		{
			count++;
			dwrite_line("Block is free, count =", count);
		}
		
		is_not_last = !is_last(current);
		current = get_next(current);
	} while(is_not_last);
	
	dwrite_line("RETURN:blocks_free() with ", count);
	
	return count;
}


#if LINK_LIST_ADVANCED_DEBUG
void memory_list()
{
	byte* current = memory_root_pointer;
	write_line("-------------------------------");
	bool is_not_last;
	do
	{
		write_line("-----> Block at ", (int)get_user_memory_block_address(current));
		write_string("Header at "); write_int((int)current); write_string("  Raw header="); print_raw_header(current); write_cr();
		if(is_free(current)) { 
			write_line("  Free"); 
		}
		if(is_allocated(current)) { 
			write_line("  Allocated"); 
		}
		if(is_last(current)) { 
			write_line("  Last"); 
		}
		write_string("  Size = "); write_int(get_size(current)); write_string(" (0x"); write_hex(get_size(current)); write_string(")"); write_cr();
		write_line("  Previous = ", (int)get_previous(current));
		if(is_first(current)) { 
			write_line("    (First)"); 
		}
		is_not_last = !is_last(current);
		if(is_not_last) {
			write_line("  Next = ", (int)get_next(current)); 
		}
		current = get_next(current);
	} while(is_not_last);	
	write_line("-------------------------------");
}
#endif


void memstat(memstat_data_t* returned_memory_statistics)
{
	returned_memory_statistics->total_mem = total_free_store_memory;
	returned_memory_statistics->space_available = total_free();
	returned_memory_statistics->num_blocks_allocated = blocks_allocated();
	returned_memory_statistics->largest_unallocated = largest_free();
	returned_memory_statistics->number_of_gaps = blocks_free();
	
	returned_memory_statistics->ram_start = RAM_START;
	returned_memory_statistics->ram_end = RAM_END;
	returned_memory_statistics->free_store_start = FREE_STORE_START;
	returned_memory_statistics->free_store_end = FREE_STORE_END;
}

void write_memstat()
{
	write_line("total=", total_free_store_memory);
	write_line("available=", total_free());
	write_line("blocks alloc=", blocks_allocated()); 
	write_line("largest unalloc=", largest_free());
	write_line("gap=", blocks_free());
	write_string("ram start=0x"); write_hex(RAM_START); write_cr();
	write_string("ram end=0x"); write_hex(RAM_END); write_cr();
	write_string("free store start=0x"); write_hex(FREE_STORE_START); write_cr();
	write_string("free store end=0x"); write_hex(FREE_STORE_END); write_cr();
}

size_t total_free()
{
	dwrite_line("CALL:total_free()");
	byte* current = memory_root_pointer;
	
	size_t total= 0;
	
	// always at least one
	bool is_not_last;
	do
	{
		dwrite_line("Looking at block ", (int)current);
		if(is_free(current))
		{
			size_t this_block_size = get_size(current);
			dwrite_line("Block is free and has size of ", this_block_size);
			total += this_block_size;
			dwrite_line("Total = ", total);
		}
		
		is_not_last = !is_last(current);
		current = get_next(current);
	} while(is_not_last);
	
	dwrite_line("RETURN:total_free() size with ", total);
	
	return total;
}

size_t largest_free()
{
	dwrite_line("CALL:largest_free()");
	byte* current = memory_root_pointer;
	
	size_t best_size= 0;
	
	// always at least one
	bool is_not_last;
	do
	{
		dwrite_line("Looking at block ", (int)current);
		if(is_free(current))
		{
			size_t this_block_size = get_size(current);
			
			dwrite_line("Block is free and has size of ", this_block_size);
			if(this_block_size > best_size)
			{
				dwrite_line("Best candidate so far");
				best_size = this_block_size;
			}
		}
		
		is_not_last = !is_last(current);
		current = get_next(current);
	} while(is_not_last);
	
	dwrite_line("RETURN:largest_free() block with ", best_size);
	
	return best_size;
}


//
// Actual worker functions
//

//
//
//
byte* find_free(size_t size)
{
	dwrite_line("CALL:find_free() block of size", size);
	byte* current = memory_root_pointer;
	
	byte* best = 0;
	size_t best_size= 9999999;
	
	// always at least one
	bool is_not_last;
	do
	{
		dwrite_line("Looking at block ", (int)current);
		if(is_free(current))
		{
			size_t this_block_size = get_size(current);
			
			dwrite_line("Block is free and has size of ", this_block_size);
			if(this_block_size >= size && this_block_size < best_size)
			{
				dwrite_line("Best candidate so far");
				best = current;
				best_size = this_block_size;
			}
		}
		
		is_not_last = !is_last(current);
		current = get_next(current);
	} while(is_not_last);

	dwrite_line("RETURN:find_free() block with ", (int)best);

	return best;
}



//
//
//
static inline size_t align_int32(size_t i)
{
	return ((i+3)&~0x03);
}

void *malloc( size_t size )
{
#if LINK_LIST_ADVANCED_DEBUG
	memory_list();
#endif
	dwrite_line("CALL:malloc() block of size", size);
	dwrite_line("Bytes requested=", size);
	// this next line is not needed since headers support unique pointers for zero allocation
	//if(size==0) size=1;			// support requirement for unique pointers and requirement for allocation of 0 to succeed.
	size = align_int32(size);	// ensure 4 byte boundary for allocations
	dwrite_line("Bytes to be allocated=",size);
	
	byte* available_block = find_free(size);
	if(available_block != 0)
	{
		// fill in this header
		make_allocated(available_block);
		dwrite_line("Allocated ", (int)available_block);

		// previous header will be uneffected
		
		// we will also need to create a false header if the size are 4 more than sizeof(mem_data_header)
		// to avoid wasting space
		// +4 removed, since malloc(0) requires just the header now, so even these
		// empty blocks are useful.
		size_t actual_size = get_size(available_block);
		size_t spare_space = actual_size-size;
		if( spare_space >= (sizeof(mem_data_header)))
		{
			dwrite_line("Can use spare space as extra block, spare bytes = ", spare_space);
			
			// we have enough space to create a header to use up the extra space for something useful
			byte* intermediate_block = available_block+size+sizeof(mem_data_header);
			
			// find the next block in the chain
			byte* next_block = get_next(available_block);
			bool next_block_exists = !is_last(available_block);
			
			dwrite_line("Next block is ", (int)next_block);
			dwrite_line("Next block exists = ", next_block_exists);
			
			// create this header
			set_next(intermediate_block, next_block);
			set_previous(intermediate_block, available_block);
			make_free(intermediate_block);
			dwrite_line("Create new header at ", (int)intermediate_block);
			dwrite_line("  next = ", (int)next_block);
			dwrite_line("  previous = ", (int)available_block);
			dwrite_line("  free");
			if(next_block_exists)
			{
				set_not_last(intermediate_block);
				dwrite_line("  not last block");
			}
			else
			{
				set_last(intermediate_block);
				dwrite_line("  last block");
			}

			// patch this new block in to the next and previous
			if(next_block_exists)
			{
				set_previous(next_block, intermediate_block);
				dwrite_line("next block previous patched to ", (int)intermediate_block);
			}
			else
			{
				set_not_last(available_block);			// previous last block isn't any more
				dwrite_line("available block no longer last block");
			}
			set_next(available_block, intermediate_block);
			dwrite_line("available next patched to ", (int)intermediate_block);
		}
	}

	dwrite_line("RET:malloc() ", (int)available_block);	
#if LINK_LIST_ADVANCED_DEBUG
	memory_list();
#endif
	return get_user_memory_block_address(available_block);
}


void free(void* ptr)
{
#if LINK_LIST_ADVANCED_DEBUG
	memory_list();
#endif
	dwrite_line("CALL: free() ", (int)ptr);
	byte* header = get_header_address((byte*)ptr);
	
	if(is_free(header)) { halt("free(FreeBlock)"); }
	
	// actually free this block
	make_free(header);
	
	// ok. Now, we have a situation where the block is free.
	// but what we want to do is combine this with any free block before or after.
	// Doing this everytime means we won't have a string of free blocks
	// i.e. the most free blocks between allocated blocks will be one.
	// This means free blocks will always be as large as possible. 
	// Because we have previous and next pointers (a doubly linked list) this 
	// is a simple operation and means free is very quick.
	
	byte* previous = get_previous(header);
	if(previous != header)	// if there is a previous header
	{
		dwrite_line("Analysing previous");
		if(is_free(previous))
		{
			dwrite_line("Previous is free");

			// set the size of the previous block to both block sizes
			set_size(previous, get_size(header)+get_size(previous)+sizeof(mem_data_header));
			// set size also sets the size of the previous

			// if this one is the last, then the previous will be the last now
			if(is_last(header))
			{
				set_last(previous);
				dwrite_line("Previous now last");
			}
			else
			{
				//need to patch next to previous... otherwise it will point at phantom header
				dwrite_line("patch next to previous");
				byte* next = get_next(header);
				set_previous(next, previous);
			}
			
			header = previous;			// make the next bit of code work
		}
	}
	
	if(!is_last(header))
	{
		byte* next = get_next(header);
		dwrite_line("Analysing next");
		if(is_free(next))
		{
			dwrite_line("Next is free");

			// set the size of the previous block to both block sizes
			set_size(header, get_size(header)+get_size(next)+sizeof(mem_data_header));
			// set size also sets the size of the previous
			
			// if this next one is the last, then the header will be the last now
			if(is_last(next))
			{
				set_last(header);
				dwrite_line("current header now last");
			}
			else
			{
				//again, we need to patch 'next' header's 'previous' pointer... otherwise it will point at phantom header
				//if we are not the last...
				dwrite_line("patch nextnext to header");
				byte* nextnext = get_next(header);
				set_previous(nextnext, header);
			}
			
		}
	}

	
	dwrite_line("RET: free() ");

#if LINK_LIST_ADVANCED_DEBUG
	memory_list();
#endif
}

#elif SLOT_BASED_FIRST_FIT_ALGORITHM

//
// Originally from http://www.osdcom.info/content/view/31/39/
//
// Actually made into working algorithm and modified for use here by Rob
//

#define MAX_MEMORY 1024 * 1024 * MEMORY_SIZE_IN_MEGABYTES
// using a fixed size array isn’t the best idea,
// the best implementation would be to manage the data with a chained list
const int LIST_ITEMS = 100;

typedef struct {
    size_t size;
    size_t first_byte_ptr;
} list_item;

list_item free_list[LIST_ITEMS];			// need to initialise first with free mem and rest empty (=0)
list_item allocated[LIST_ITEMS];

byte blocks_allocated = 0;

void init_dynamic_memory()
{
	
}

void *malloc(size_t size) {
    int i;
    void *current;
	
    if( blocks_allocated >= LIST_ITEMS ) return null; // no slots left
	
	// let's find an entry (the first one) that has enough space
	// (It would be better to find the closest fit, but ignore that now)
    for( i = 0 ; i < LIST_ITEMS ; i++ )
        if( free_list[i].size >= size ) break;
	
    if( i == LIST_ITEMS ) return null; // no suitable block found
	
	// allocate this block
    current = (void *)free_list[i].first_byte_ptr;
	
	// put the rest in that entry in the free list
    free_list[i].first_byte_ptr += size;
    free_list[i].size -= size;
	
	// Rob: This won't work - we will only get 100 allocations total. Really need a loop here to search for empty alloc slots.
	// record that as an allocated block in the list
    allocated[blocks_allocated].first_byte_ptr = current;
    allocated[blocks_allocated].size = size;
	
	// increment the blocks allocated
    blocks_allocated++;				// Rob: never gets decremented - this is a problem
	
    return current;
}

void free(void *ptr) {
    int i;
	
	// find the index of this allocated block to free
    for( i = 0 ; i < LIST_ITEMS ; i++ )
        if( allocated[i].first_byte_ptr == ptr ) break;
    
	if(i==LIST_ITEMS) { halt("free:bad ptr"); }
	
	// shove the previous allocated block back in the free list whereever there is space
	// (would be better to combine blocks here rather than letting them become more fragmented)
    for( k = 0 ; k < LIST_ITEMS ; k++ )
        if( free_list[k].size == 0 ) break;
	if(k==LIST_ITEMS) { halt("free:no slots"); }

	// Rob: this is going to be a problem - after 100 frees surely you have all free blocks
	// taken by allocated blocks? Re-combining would get rid of this. 
	// Maybe it never gets as bad as this ... since you will be using up these as well. 
	// but it could be that with a mixture of large and small allocations - even in
	// a safe order you end up with lots of fragmentation very quickly?
	// Only if the remaining items in the free list just happen to end up at zero
	// is this not a problem.
    free_list[k].size = allocated[i].size;
    free_list[k].first_byte_ptr = allocated[k].first_byte_ptr;
	
    allocated[i].first_byte_ptr = 0;
    allocated[i].size = 0;
	
    return;
}



#endif


//
// C++ memory operators
//
void* operator new(size_t sz)
{
	void* p = malloc(sz);
#if PRINT_NEW_DELETE_ALLOCATIONS
	write_string("new called requesting "); write_int(sz); write_string(" bytes, object address = 0x"); write_hex(reinterpret_cast<unsigned int>(p)); write_cr();
#endif
	if(p==0) {
		halt("new alloc failed");
	}
	return p;
}

void operator delete(void *p)
{
#if PRINT_NEW_DELETE_ALLOCATIONS
	write_string("delete called with object address = 0x"); write_hex(reinterpret_cast<unsigned int>(p)); write_cr();
#endif
	if(p) free(p);
}

void* operator new[](size_t sz)
{
	void* p = malloc(sz);
#if PRINT_NEW_DELETE_ALLOCATIONS
	write_string("new[] called requesting "); write_int(sz); write_string(" bytes, object address = 0x"); write_hex(reinterpret_cast<unsigned int>(p)); write_cr();
#endif
	if(p==0) {
		halt("new[] alloc failed");
	}
	return p;
}

void operator delete[](void *p)
{
#if PRINT_NEW_DELETE_ALLOCATIONS
	write_string("delete[] called with object address = 0x"); write_hex(reinterpret_cast<unsigned int>(p)); write_cr();
#endif
	if(p) free(p);
}


#if 0
void* operator new(size_t sz)
{
	return MALLOC(sz);
}

void operator delete(void *p)
{
	FREE(p);
}

void* operator new[](size_t sz)
{
	return MALLOC(sz);
}

void operator delete[](void *p)
{
	FREE(p);
}

extern "C" void __assert(const char * , const char * , int ) {
	
}

// from http://www.osdev.org/wiki/C_PlusPlus#new_and_delete
//overload the operator "new"
void * operator new (uint_t size)
{
    return kmalloc(size);
}

//overload the operator "new[]"
void * operator new[] (uint_t size)
{
    return kmalloc(size);
}

//overload the operator "delete"
void operator delete (void * p)
{
    kfree(p);
}

//overload the operator "delete[]"
void operator delete[] (void * p)
{
    kfree(p);
}
//Note that new should actually use kcalloc (allocate and zero) otherwise the variables will be filled //with garbage which you will then need to clear manually

#endif
#endif



//
// C++ Support
// 
extern "C" void __cxa_pure_virtual(void)
{
	// call to a pure virtual function happened ... wow, should never happen ... stop
    halt("cxa_pure()");
	while(1)
		;
}
// From http://www.osdev.org/wiki/C_PlusPlus :
// Pure virtual functions have to be overridden in every class, that should be instantiated. 
// If during runtime your kernel detects that a call to a pure virtual function couldn't be 
// made, it calls the following function:
//    ...
// These functions should actually never be called, because without hacks (or through 
// undefined behaviour of your kernel) it is not possible to instantiate a class that 
// doesn't define all pure virtual functions. But nonetheless you have to define this 
// function or your linker will complain about unresolved symbols.
//
// From eCos:
// g++ generates references to a function __cxa_pure_virtual()
// whenever the code contains pure virtual functions. This is a
// placeholder used while constructing an object, and the function
// should never actually get called. There is a default
// implementation in libsupc++ but that has dependencies on I/O
// and so on, unwanted in a minimal eCos environment. Instead we
// want a minimal __cxa_pure_virtual().

// From http://www.osdev.org/wiki/C_PlusPlus :
// GCC sometimes emits code that calls operator delete even when you haven't used delete yourself. 
// It seems to sometimes emit a "normal" version of a destructor and a separate version for delete. 
/// So you might need to define operator delete even before you have kmalloc:

#if !INCLUDE_DYNAMIC_MEMORY
void operator delete(void *)
{
	// should never get here ... we don't use new
	halt("Funny Delete called");
	while(1)
		;
}
#endif
//  This won't be called until you use new/delete, but it might be needed for linking. This problem 
// seems to appear when classes with pure virtual functions are used. (Rob: Actually not pure ... )

// More info...
// Why are there multiple destructors?
// see http://lists.gnu.org/archive/html/bug-gnu-utils/2004-07/msg00042.html
// 
// Copied text:
//
/*
 Hi Andy,
 
 A better mailling list for gcc questions is address@bogus.example.com .
 
 But here's an answer.
 
 gcc emits two or three versions of the object code for each
 destructor.
 
 They are:
 
 _ZN7DerivedD0Ev  Derived::~Derived [in-charge deleting]()
 _ZN7DerivedD1Ev  Derived::~Derived [in-charge]()
 _ZN7DerivedD2Ev  Derived::~Derived [not-in-charge]()
 
 When the program destroys an object, the compiler generates code
 to call one of the in-charge destructors for the object's complete
 type.  The in-charge destructor calls not-in-charge destructors
 for each of the non-virtual base classes, then calls not-in-charge
 destructors for each virtual base.
 
 Consider the classic diamond-shaped hierarchy:
 
 class A { };
 class B1 : virtual public A { ... };
 class B2 : virtual public A { ... };
 class C : virtual public B1, virtual public B2 { ... };
 
 When the program destroys an object of type A:
 
 A::~A [in-charge]
 
 When the program destroys an object of type B1:
 
 B1::~B1 [in-charge]
 A::~A [not-in-charge]
 
 When the program destroys an object of type C:
 
 C::~C [in-charge]
 B2::B2 [not-in-charge]
 B1::B1 [not-in-charge]
 A::A [not-in-charge]
 
 See, the destructor for the complete object is the only destructor
 that can destroy the virtual base classes.  If you destroy an object
 of type B1, then you need B1::~B1 [in-charge], which calls one of
 the flavors of A::~A.  If you destroy an object of type C,
 then C::~C [in-charge] calls B1::~B1 [not-in-charge], and
 B1::~B1 [not-in-charge] does *not* call any of A::~A.  It's the
 responsibility of the complete object type, and *only* the complete
 object type, to construct/destroy virtual bases.
 
 That's what the in-charge/not-in-charge distinction is all about.
 
 I don't know what the "in-charge" versus "in-charge deleting"
 distinction is about.
 
 Here is more info on the C++ multi-vendor ABI standard.
 
 http://www.codesourcery.com/cxx-abi/
 http://www.codesourcery.com/cxx-abi/abi.html
 
 This was originally developed for Itanium, but g++ uses it for
 all C++ architectures.
 
 > I need to know which instance of DerivedD2Ev gets linked into the final
 > executable (don't ask why...), so I modified the assembler files and
 > continued the build. However my modifications (inserted instructions)
 > never make it into the final build.
 
 _ZN7DerivedD2Ev will be called by the destructor of any class
 which has 'Derived' directly as a base class, or directly or
 indirectly as a virtual base class.
 
 class MoreDerived : public Derived         // will  call _ZN7DerivedD2Ev
 class MoreMoreDerived : public MoreDerived // won't call _ZN7DerivedD2Ev
 class V1 : public virtual Derived          // will  call _ZN7DerivedD2Ev
 class V2 : public V1                       // will  call _ZN7DerivedD2Ev
 
 Again, each destructor calls the destructors for its immediate
 non-virtual base classes.  And then the top-level destructor
 calls the destructor for all the virtual base classes anywhere
 in the inheritance hierarchy.
 
 > Interestingly, there is no instance of it in Derived.S, just in
 > ReallyDerived.S and in another file which references it.
 
 Yup.  The compiler synthesizes these functions where they are used.
 Derived::~Derived [not-in-charge]() is not used by Derived;
 it's used by ReallyDerived.
 
 If you have a lot of classes that have Derived as an immediate
 base class, or directly or indirectly as a virtual base, then you'll
 get a lot of Derived::~Derived [not-in-charge]() in a lot of
 different .o files.  Depending on your platform, these might be
 coalesced by the linker into a single function, or there might be
 several copies in the object code.
 
 That's what happens when you let the compiler synthesize methods.
 It synthesizes them at point-of-use.
 
 If you write class Derived like this:
 
 // Derived.h
 Derived::Derived
 {
	 virtual ~Derived ();
 }
 
 // Derived.c
 Derived::~Derived () { ; }
 
 Then there will be one copy of Derived::~Derived [not-in-charge],
 and it will be in Derived.o.  That will make it easier for you
 to mess with the assembly code.
 
 Hope this helps,
 
 Michael C
 */

//
//


#if 0		// delete array unit test
class test_class {
public:
	int x;
	int y;
	test_class();
	~test_class();
};

test_class::test_class()
:x(0),y(0) 
{
	x = 3;
}

test_class::~test_class()
{
	x = x+y;
}

void test_delete_array()
{
	//
	// Simple types are simple memory allocations
	//
	char * x = new char [5];
	
	x[0] = 10;
	x[1] = 20;
	x[2] = 30;
	
	delete[] x;
	
	//
	// Classes are slightly different. These need to store
	// the number of classes so that we can call the constructors
	// and destructors of each
	//
	test_class* y = new test_class[7];
	
	delete[] y;
}

#endif

void print_stack_space()
{   
	extern unsigned int _stack;
	extern unsigned int _stack_bottom;
	
	byte* const stack_top = reinterpret_cast<byte*>(&_stack);
	byte* const stack_bottom = reinterpret_cast<byte*>(&_stack_bottom);
	byte* current_stack;
	asm volatile (" mov  %0, sp" : "=r" (current_stack) : /* no inputs */  );
	
	write_line("Stack used=", (stack_top-current_stack)-(128+16));	// 128+16 is for other stacks
	write_line("Stack available=", current_stack-stack_bottom);
}



