/*
 * Copyright 2006-2007  Anthony Rowe and Adam Goode
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

//
// Lots of modifications from original by Rob Probin. 
// All changes Copyright (C) 2008 Rob Probin and
// this file Licensed as above (Apache License v2.0).
//

/******************************************************************************
 *
 *  Initial CMUcam3 (cc3) data types and functions.
 *
 *****************************************************************************/
//#include <stdbool.h>
//#include <stdlib.h>

#include "camera_board_lowlevel.h"
#include "camera_board_hardware_defs.h"
#include "camera_board_util.h"
#include "LPC2100.h"
//#include "serial.h"
//#include "devices.h"
#include "camera_board_interrupts.h"


// Globals used by CMUCam functions
cc3_frame_t cc3_g_pixbuf_frame;        // global that keeps clip, stride


//static _cc3_camera_state_t _cc3_g_current_camera_state;


static inline void _cc3_pixbuf_skip_pixels (uint32_t size);

static inline uint8_t _cc3_pixbuf_read_subpixel (void);

//static inline void _cc3_pixbuf_read_pixel (uint8_t * pixel,
//                                           uint8_t * saved,
//                                           uint8_t off0,
//                                           uint8_t off1, uint8_t off2);

static inline void _cc3_pixbuf_skip_subpixel (void);


// Move to the next byte in the FIFO
static inline void _cc3_fifo_read_inc (void);

//static uint8_t _cc3_second_green;
//static bool _cc3_second_green_valid;

//static void _cc3_update_frame_bounds (cc3_frame_t *);


//
// This routine starts the grab of the frame into the fifo
//
bool load_frame_into_fifo()
{

  if (REG (GPIO_IOPIN) & _CC3_CAM_POWER_DOWN) {
    return false;
  }

  //REG(GPIO_IOCLR)=CAM_IE;
  //while(frame_done!=1);
 // cc3_pixbuf_rewind();
  _cc3_pixbuf_write_rewind ();

  while (!(REG (GPIO_IOPIN) & _CC3_CAM_VSYNC)); //while(CAM_VSYNC);
  while (REG (GPIO_IOPIN) & _CC3_CAM_VSYNC);    //while(!CAM_VSYNC);


  REG (GPIO_IOSET) = _CC3_BUF_WEE;


  // wait for vsync to finish
  while (!(REG (GPIO_IOPIN) & _CC3_CAM_VSYNC)); //while(CAM_VSYNC);

  enable_vsync_interrupt ();

  // Delay to wait for the image buffer to fill up before you start reading it
  for (unsigned int i = 0; i < 3; i++) {
    while (!(REG (GPIO_IOPIN) & _CC3_CAM_HREF));
    while (REG (GPIO_IOPIN) & _CC3_CAM_HREF);
  }

  cc3_pixbuf_rewind();

  //while (REG (GPIO_IOPIN) & _CC3_CAM_VSYNC);
  //REG (GPIO_IOCLR) = _CC3_BUF_WEE;
//      delay();
//  REG(GPIO_IOCLR)=_CC3_BUF_WEE;  //BUF_WEE=0return 1;
	return true;
}

void _cc3_fifo_read_inc (void)
{
  REG (GPIO_IOSET) = _CC3_BUF_RCK;
  REG (GPIO_IOCLR) = _CC3_BUF_RCK;
}

void _cc3_pixbuf_skip_pixels (uint32_t size)
{
  uint32_t i;

  for (i = 0; i < size; i++) {
    _cc3_pixbuf_skip_subpixel ();
    _cc3_pixbuf_skip_subpixel ();
    _cc3_pixbuf_skip_subpixel ();
    _cc3_pixbuf_skip_subpixel ();
  }
}


uint8_t _cc3_pixbuf_read_subpixel (void)
{
  uint8_t result = REG (GPIO_IOPIN) >> 24;
  _cc3_fifo_read_inc ();

  return result;
}

void _cc3_pixbuf_skip_subpixel (void)
{
  _cc3_fifo_read_inc ();
}

//void _cc3_pixbuf_read_pixel (uint8_t * pixel,
//                             uint8_t * saved,
//                             uint8_t off0, uint8_t off1, uint8_t off2)
//{
//  if (cc3_g_pixbuf_frame.x_step == 1) {
//    if (_cc3_second_green_valid) {
//      // use the second green
//      _cc3_second_green_valid = false;
//      *(pixel + off0) = *(saved + off0);
//      *(pixel + off1) = _cc3_second_green;
//      *(pixel + off2) = *(saved + off2);
//
//      return;
//    }
//
//    // otherwise, load a new thing
//    *(pixel + off1) = _cc3_pixbuf_read_subpixel ();   // G
//    *(pixel + off0) = _cc3_pixbuf_read_subpixel ();   // R
//    _cc3_second_green = _cc3_pixbuf_read_subpixel (); // G
//    *(pixel + off2) = _cc3_pixbuf_read_subpixel ();   // B
//
//    _cc3_second_green_valid = true;
//  } else {
//    _cc3_pixbuf_skip_subpixel ();
//    *(pixel + off0) = _cc3_pixbuf_read_subpixel ();
//    *(pixel + off1) = _cc3_pixbuf_read_subpixel ();
//    *(pixel + off2) = _cc3_pixbuf_read_subpixel ();
//  }
//}

/**
 * cc3_pixbuf_rewind():
 * Rewinds the fifo.
 * Calling this and then changing parameters such as the
 * region of interest, channel of interest, virtual frame, and
 * subsampling will allow rapid reprocessing of a new frame.
 */
void cc3_pixbuf_rewind ()
{
  REG (GPIO_IOCLR) = _CC3_BUF_RRST;
  REG (GPIO_IOCLR) = _CC3_BUF_RCK;
  REG (GPIO_IOSET) = _CC3_BUF_RCK;
  REG (GPIO_IOCLR) = _CC3_BUF_RCK;
  REG (GPIO_IOSET) = _CC3_BUF_RRST;
  // skip first line because it is blue
  _cc3_pixbuf_skip_pixels (cc3_g_pixbuf_frame.raw_width / 2);
#warning "is the previous line valid? it seems as if it's skipping a non-number of bytes"
  //_cc3_pixbuf_skip_pixels (cc3_g_pixbuf_frame.raw_width / 2);
  //_cc3_second_green_valid = false;
  //cc3_g_pixbuf_frame.y_loc = 0;
}


void read_raw_pixels_from_camera(uint8_t* destination, unsigned int subpixels)
{
	//
	// Loop unrolled for speed (to reduce loop overhead)
	//
	// Should really time vs. non-unrolled version
	//
	unsigned int octets = subpixels >> 3;			// divide by 8
	
	for(unsigned int i=0; i<octets; i++)
	{
		*destination = _cc3_pixbuf_read_subpixel(); destination++;
		*destination = _cc3_pixbuf_read_subpixel(); destination++;
		*destination = _cc3_pixbuf_read_subpixel(); destination++;
		*destination = _cc3_pixbuf_read_subpixel(); destination++;

		*destination = _cc3_pixbuf_read_subpixel(); destination++;
		*destination = _cc3_pixbuf_read_subpixel(); destination++;
		*destination = _cc3_pixbuf_read_subpixel(); destination++;
		*destination = _cc3_pixbuf_read_subpixel(); destination++;
	}
	
	unsigned int remainder = subpixels & 0x07;
	for(unsigned int i=0; i< remainder; i++)
	{
		*destination = _cc3_pixbuf_read_subpixel();
		destination++;
	}

}

#define REMOVE_BLUE_GREEN_SUBPIXELS 1

void get_processed_pixels_from_camera(uint8_t* destination, int offset)
{
	//
	// Loop unrolled for speed (to reduce loop overhead)
	//
	// Should really time vs. non-unrolled version
	//
	for(unsigned int i=0; i<(320*240/8); i++)
	{
		uint8_t byte = 0; uint16_t red, green;
//		uint8_t byte = 0; uint8_t red, green;			// size doesn't matter because all promoted to unsigned ints before add/compare!
		// blue is missed, so it's GRG...B
		green = _cc3_pixbuf_read_subpixel();
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		_cc3_pixbuf_skip_subpixel ();		// skip B (alternating line from interlace)
#endif
		if(red > green+offset)
		{
			byte |= 0x80;
		}
		green = _cc3_pixbuf_read_subpixel();
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		_cc3_pixbuf_skip_subpixel ();		// skip B (alternating line from interlace)
#endif
		if(red > green+offset)
		{
			byte |= 0x40;
		}
		green = _cc3_pixbuf_read_subpixel();
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		_cc3_pixbuf_skip_subpixel ();		// skip B (alternating line from interlace)
#endif
		if(red > green+offset)
		{
			byte |= 0x20;
		}
		green = _cc3_pixbuf_read_subpixel();
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		_cc3_pixbuf_skip_subpixel ();		// skip B (alternating line from interlace)
#endif
		if(red > green+offset)
		{
			byte |= 0x10;
		}
		green = _cc3_pixbuf_read_subpixel();
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		_cc3_pixbuf_skip_subpixel ();		// skip B (alternating line from interlace)
#endif
		if(red > green+offset)
		{
			byte |= 0x08;
		}
		green = _cc3_pixbuf_read_subpixel();
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		_cc3_pixbuf_skip_subpixel ();		// skip B (alternating line from interlace)
#endif
		if(red > green+offset)
		{
			byte |= 0x04;
		}
		green = _cc3_pixbuf_read_subpixel();
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		_cc3_pixbuf_skip_subpixel ();		// skip B (alternating line from interlace)
#endif
		if(red > green+offset)
		{
			byte |= 0x02;
		}
		green = _cc3_pixbuf_read_subpixel();
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		_cc3_pixbuf_skip_subpixel ();		// skip B (alternating line from interlace)
#endif
		if(red > green+offset)
		{
			byte |= 0x01;
		}
		
		*destination = byte; destination++;
	}
	
}

void get_processed_pixels_from_camera_BLUE(uint8_t* destination, int offset)
{
	//
	// Loop unrolled for speed (to reduce loop overhead)
	//
	// Should really time vs. non-unrolled version
	//
	for(unsigned int i=0; i<(320*240/8); i++)
	{
		uint8_t byte = 0; uint16_t red, blue;
		//		uint8_t byte = 0; uint8_t red, green;			// size doesn't matter because all promoted to unsigned ints before add/compare!
		// blue is missed, so it's GRG...B
		_cc3_pixbuf_skip_subpixel();		// G
		red = _cc3_pixbuf_read_subpixel();	// red
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		blue = _cc3_pixbuf_read_subpixel ();		// read B (alternating line from interlace)
#endif
		if(red > blue+offset)
		{
			byte |= 0x80;
		}
		_cc3_pixbuf_skip_subpixel();		// G
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		blue = _cc3_pixbuf_read_subpixel ();		// read B (alternating line from interlace)
#endif
		if(red > blue+offset)
		{
			byte |= 0x40;
		}
		_cc3_pixbuf_skip_subpixel();		// G
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		blue = _cc3_pixbuf_read_subpixel ();		// read B (alternating line from interlace)
#endif
		if(red > blue+offset)
		{
			byte |= 0x20;
		}
		_cc3_pixbuf_skip_subpixel();		// G
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		blue = _cc3_pixbuf_read_subpixel ();		// read B (alternating line from interlace)
#endif
		if(red > blue+offset)
		{
			byte |= 0x10;
		}
		_cc3_pixbuf_skip_subpixel();		// G
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		blue = _cc3_pixbuf_read_subpixel ();		// read B (alternating line from interlace)
#endif
		if(red > blue+offset)
		{
			byte |= 0x08;
		}
		_cc3_pixbuf_skip_subpixel();		// G
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		blue = _cc3_pixbuf_read_subpixel ();		// read B (alternating line from interlace)
#endif
		if(red > blue+offset)
		{
			byte |= 0x04;
		}
		_cc3_pixbuf_skip_subpixel();		// G
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		blue = _cc3_pixbuf_read_subpixel ();		// read B (alternating line from interlace)
#endif
		if(red > blue+offset)
		{
			byte |= 0x02;
		}
		_cc3_pixbuf_skip_subpixel();		// G
		red = _cc3_pixbuf_read_subpixel();
#ifdef REMOVE_BLUE_GREEN_SUBPIXELS
		_cc3_pixbuf_skip_subpixel ();		// skip G (alternating line from interlace)
		blue = _cc3_pixbuf_read_subpixel ();		// read B (alternating line from interlace)
#endif
		if(red > blue+offset)
		{
			byte |= 0x01;
		}
		
		*destination = byte; destination++;
	}
	
}

/**
 * cc3_pixbuf_read_rows():
 * Using the cc3_frame_t reads rows taking into account the virtual window and subsampling.
 * This function copies a specified number of rows from the camera FIFO into a block
 * of cc3_pixel_t memory.
 * This should be the lowest level call that the user directly interacts with.
 * Returns number of rows read. (May be zero if error.)
 */
//int cc3_pixbuf_read_rows (void * mem, uint32_t rows)
//{
//
//  int16_t j;
//  uint16_t r;
//
//  uint8_t off0, off1, off2;
//
//  int width = cc3_g_pixbuf_frame.width;
//
//  unsigned int row_limit = (cc3_g_pixbuf_frame.y1 - cc3_g_pixbuf_frame.y_loc)
//    / cc3_g_pixbuf_frame.y_step;
//
//  if (row_limit < rows) {
//    rows = row_limit;
//  }
//
//  if (_cc3_g_current_camera_state.colorspace == CC3_COLORSPACE_RGB) {
//    off0 = 0;
//    off1 = 1;
//    off2 = 2;
//  }
//  else if (_cc3_g_current_camera_state.colorspace == CC3_COLORSPACE_YCRCB) {
//    off0 = 1;
//    off1 = 0;
//    off2 = 2;
//  }
//  else {
//    off0 = 0;
//    off1 = 1;
//    off2 = 2;
//  }
//
//  // First read into frame
//  _cc3_seek_top ();
//
//  for (r = 0; r < rows; r++) {
//    int x = cc3_g_pixbuf_frame.x0;
//
//    // First read into line
//    _cc3_seek_left ();
//
//    switch (cc3_g_pixbuf_frame.coi) {
//    case CC3_CHANNEL_ALL:
//      _cc3_second_green_valid = false;
//      for (j = 0; j < width; j++) {
//        uint8_t *p = ((uint8_t *) mem) +
//          (r * width + j * 3);
//        _cc3_pixbuf_read_pixel (p, p - 3, off0, off1, off2);
//
//	// advance by x_step
//	x += cc3_g_pixbuf_frame.x_step;
//	_cc3_pixbuf_skip_pixels ((cc3_g_pixbuf_frame.x_step - 1) / 2);
//      }
//
//      break;
//
//    case CC3_CHANNEL_RED:
//      for (j = 0; j < width; j++) {
//	uint8_t *p = ((uint8_t *) mem) + (r * width + j);
//
//	if ((j & 0x1) == 0 || cc3_g_pixbuf_frame.x_step > 1) {
//	  // read
//	  _cc3_pixbuf_skip_subpixel ();
//	  *p = _cc3_pixbuf_read_subpixel ();
//	  _cc3_pixbuf_skip_subpixel ();
//	  _cc3_pixbuf_skip_subpixel ();
//	} else {
//	  *p = *(p - 1);
//	}
//
//	x += cc3_g_pixbuf_frame.x_step;
//	_cc3_pixbuf_skip_pixels ((cc3_g_pixbuf_frame.x_step - 1) / 2);
//      }
//      break;
//
//    case CC3_CHANNEL_GREEN:
//      for (j = 0; j < width; j++) {
//	uint8_t *p = ((uint8_t *) mem) + (r * width + j);
//
//	if ((j & 0x1) == 0 || cc3_g_pixbuf_frame.x_step > 1) {
//	  // read
//	  *p = _cc3_pixbuf_read_subpixel ();
//	  _cc3_pixbuf_skip_subpixel ();
//	  _cc3_second_green = _cc3_pixbuf_read_subpixel ();
//	  _cc3_pixbuf_skip_subpixel ();
//	} else {
//	  *p = _cc3_second_green;
//	}
//
//	x += cc3_g_pixbuf_frame.x_step;
//	_cc3_pixbuf_skip_pixels ((cc3_g_pixbuf_frame.x_step - 1) / 2);
//      }
//      break;
//
//    case CC3_CHANNEL_BLUE:
//      for (j = 0; j < width; j++) {
//	uint8_t *p = ((uint8_t *) mem) + (r * width + j);
//
//	if ((j & 0x1) == 0 || cc3_g_pixbuf_frame.x_step > 1) {
//	  // read
//	  _cc3_pixbuf_skip_subpixel ();
//	  _cc3_pixbuf_skip_subpixel ();
//	  _cc3_pixbuf_skip_subpixel ();
//	  *p = _cc3_pixbuf_read_subpixel ();
//	} else {
//	  *p = *(p - 1);
//	}
//
//	x += cc3_g_pixbuf_frame.x_step;
//	_cc3_pixbuf_skip_pixels ((cc3_g_pixbuf_frame.x_step - 1) / 2);
//      }
//      break;
//    }
//    _cc3_pixbuf_skip_pixels ((cc3_g_pixbuf_frame.raw_width - x) / 2);
//
//
//    // advance by y_step
//    _cc3_pixbuf_skip_pixels ((cc3_g_pixbuf_frame.y_step - 1) * cc3_g_pixbuf_frame.raw_width / 2);
//    cc3_g_pixbuf_frame.y_loc += cc3_g_pixbuf_frame.y_step;
//  }
//  return rows;
//}
//



/**
 * cc3_camera_init():
 * First Enable Camera & FIFO Power, next Reset Camera, then call cc3_set functions for default state
 *
 */
bool cc3_camera_init ()
{
  bool result = true;

  // commented out by Rob
  //REG (PCB_PINSEL0) = (REG (PCB_PINSEL0) & 0xFFFF0000) | UART0_PCB_PINSEL_CFG | UART1_PCB_PINSEL_CFG;  //| 0x50;
  REG (GPIO_IODIR) = _CC3_DEFAULT_PORT_DIR;

  // originally commented out
  //REG(GPIO_IOSET)=CAM_BUF_ENABLE;
  //REG (GPIO_IOCLR) = CAM_BUF_ENABLE;  // Change for AL440B
  REG (GPIO_IOCLR) = _CC3_BUF_RESET;
  _cc3_camera_reset ();
  _cc3_fifo_reset ();


  return result;
}



//
//static void _cc3_set_cam_ddr_sccb_idle (void)
//{
//  REG (GPIO_IODIR) = _CC3_SCCB_PORT_DDR_IDLE;
//  _cc3_delay_sccb ();
//}
//
//static void _cc3_set_cam_ddr_sccb_write (void)
//{
//  REG (GPIO_IODIR) = _CC3_SCCB_PORT_DDR_WRITE;
//  _cc3_delay_sccb ();
//}
//
//
//static void _cc3_set_cam_ddr (volatile unsigned long val)
//{
//  //DDR(SCCB_PORT,val);
//  REG (GPIO_IODIR) = val;
//  _cc3_delay_sccb ();
//}
//
//
//
//static bool _cc3_sccb_send (unsigned int num, unsigned int *buffer)
//{
//  bool ack;
//  unsigned int i, k;
//  unsigned int data;
//
//  // Send Start Bit
//  //SCCB_SDA=0;  // needed because values can be reset by read-modify cycle
//  REG (GPIO_IOCLR) = 0x00800000;
//  _cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);        // SDA=0 SCL=1
//  //SCCB_SCL=0;  // needed because values can be reset by read-modify cycle
//  REG (GPIO_IOCLR) = 0x00400000;
//  _cc3_set_cam_ddr_sccb_write ();        // SDA=0 SCL=0
//
//  // Send the Byte
//  for (k = 0; k != num; k++) {
//    data = buffer[k];           // To avoid shifting array problems
//    for (i = 0; !(i & 8); i++)  // Write data
//    {
//      if (data & 0x80) {
//        _cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);  // SDA=1 SCL=0
//        _cc3_set_cam_ddr_sccb_idle ();   // SDA=1 SCL=1
//      }
//      else {
//        _cc3_set_cam_ddr_sccb_write ();  // SDA=0 SCL=0
//        _cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);  // SDA=0 SCL=1
//
//      }
//      while (!(REG (GPIO_IOPIN) & 0x00400000));
//      //while(!SCCB_SCL);
//
//
//      if (data & 0x08) {
//        _cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);  // SDA=1 SCL=0
//
//      }
//      else {
//        _cc3_set_cam_ddr_sccb_write ();  // SDA=0 SCL=0
//      }
//
//      data <<= 1;
//    }                           // END OF 8 BIT FOR LOOP
//
//    // Check ACK  <*************************************
//    _cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SDA);      // SDA=1 SCL=0
//
//    _cc3_set_cam_ddr_sccb_idle ();       // SDA=1 SCL=1
//    ack = false;
//
//    //if(SCCB_SDA)                     // sample SDA
//    if (REG (GPIO_IOPIN) & 0x00800000) {
//      ack = true;
//      break;
//    }
//
//    _cc3_set_cam_ddr_sccb_write ();      // SDA=0 SCL=0
//
//  }
//
//  // Send Stop Bit
//  _cc3_set_cam_ddr (_CC3_SCCB_PORT_DDR_READ_SCL);        // SDA=0 SCL=1
//  _cc3_set_cam_ddr_sccb_idle (); // SDA=1 SCL=1
//
//  return ack;
//
//}
//
///**
// * cc3_set_raw_register():
// * This will take an address and a value from the OmniVision manual
// * and set it on the camera.  This should be used for advanced low level
// * manipulation of the camera modes.  Currently, this will not set the
// * corresponding cc3 internal data structure that keeps record of the camera
// * mode.  Use with CAUTION.
// *
// * For basic manipulation of camera parameters see other cc3_set_xxxx functions.
// */
//bool cc3_camera_set_raw_register (uint8_t address, uint8_t value)
//{
//  unsigned int data[3];
//  int to;
//  data[0] = _cc3_g_current_camera_state.camera_type;
//  data[1] = address;
//  data[2] = value;
//  to = 0;
//  while (_cc3_sccb_send (3, data)) {
//    to++;
//    if (to > 3)
//      return false;
//  }
//  _cc3_delay_us_4 (1);
//  return true;
//}


void cc3_camera_set_power_state (bool state)
{
  if (state) {
    REG (GPIO_IOCLR) = _CC3_CAM_POWER_DOWN;
  } else {
    REG (GPIO_IOSET) = _CC3_CAM_POWER_DOWN;
  }
  //_cc3_set_register_state();   // restore registers and clear pixbuf_frame
}





