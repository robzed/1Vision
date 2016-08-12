/*
 *  language_support.h
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

#ifndef LANGUAGE_SUPPORT_H
#define LANGUAGE_SUPPORT_H


typedef long unsigned int size_t;
void halt();			// stop totally...
void halt(const char* error_message);
unsigned long user_stack_free();	// must be in user mode to use this

#define INCLUDE_DYNAMIC_MEMORY 1	// turn on dynamic memory management free store

void init_dynamic_memory();		// always present to avoid optional compilation problems

#if INCLUDE_DYNAMIC_MEMORY

//
// Normally you should use new() and delete() rather than these.
//
void *malloc(size_t size);
void free(void* ptr);

//
// Memory statistics return data structure
//
struct memstat_data_t {
	size_t total_mem;
	size_t space_available;
	unsigned int num_blocks_allocated;
	size_t largest_unallocated;
	unsigned int number_of_gaps;		// gives an idea of fragmentation
	
	// basic memory info
	unsigned int ram_start;
	unsigned int ram_end;
	unsigned int free_store_start;
	unsigned int free_store_end;
};

// 
// Functions to peek inside the memory subsystem
//
void memstat(memstat_data_t* returned_memory_statistics);	// return a bunch of data about the memory and allocator
void write_memstat(); // as above but printed
size_t total_free();		// fast check of total space availble (no overhead considerations)
size_t largest_free();		// fast check of largest available (with overhead removed)

#endif // INCLUDE_DYNAMIC_MEMORY


void print_stack_space();


inline int abs(int value)
{
	if(value<0)
	{
		value = -value;
	}
	return value;
}


#endif // LANGUAGE_SUPPORT_H


