

/* Copyright (c) 2011, Peter Barrett
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#include "Platform.h"
#include "USBAPI.h"
#include "USBDesc.h"
#include "HID.h"

#if defined(USBCON)
#ifdef HID_ENABLED

// dustin's rig, removed - Mouse_/Keyboard_/RawHID: neither was ever part of the HID
// report descriptor here (ADD_MOUSE/ADD_KEYBOARD/RAWHID_ENABLED were never defined),
// but the two global objects still forced their vtables, press/release logic and the
// 128-byte _asciimap table into flash (~0.9KB) on every build.
Joystick_ Joystick;

//================================================================================
#define ADD_JOYSTICK // dustin's rig, fixed - accidentally dropped in the Mouse/Keyboard/RawHID
// cleanup above; the whole descriptor block below is still wrapped in #ifdef ADD_JOYSTICK,
// so without this define the joystick HID report descriptor compiled out to nothing (empty
// report descriptor -> Windows/games see the CDC port but never enumerate the HID gamepad).

extern const u8 _hidReportDescriptor[] PROGMEM;
const u8 _hidReportDescriptor[] =
{
#ifdef ADD_JOYSTICK
  0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
  0x09, 0x04,	// USAGE (Joystick)
  0xA1, 0x01,	// COLLECTION (Application)
  0x85, 0x04,	// REPORT_ID (04)
  0x09, 0x01, // USAGE (Pointer)
  //0x05, 0x01,							/*   USAGE_PAGE (Generic Desktop) */
  0xA1, 0x00, // COLLECTION (Physical)

  0x09, 0x30,          // USAGE (x)
  //0x16, X_AXIS_LOG_MIN & 0xFF, (X_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM // milos, old
  //0x27, X_AXIS_LOG_MAX & 0xFF, (X_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM // milos, old
  0x17, X_AXIS_LOG_MIN & 0xFF, (X_AXIS_LOG_MIN >> 8) & 0xFF, (X_AXIS_LOG_MIN >> 16) & 0xFF, (X_AXIS_LOG_MIN >> 24) & 0xFF, // LOGICAL_MINIMUM // milos, new 32bit
  0x27, X_AXIS_LOG_MAX & 0xFF, (X_AXIS_LOG_MAX >> 8) & 0xFF, (X_AXIS_LOG_MAX >> 16) & 0xFF, (X_AXIS_LOG_MAX >> 24) & 0xFF, // LOGICAL_MAXIMUM // milos, new 32bit
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, X_AXIS_PHYS_MAX & 0xFF, (X_AXIS_PHYS_MAX >> 8) & 0xFF, (X_AXIS_PHYS_MAX >> 16) & 0xFF, (X_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff) // milos, new 32bits
  0x75, X_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,            // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  0x09, 0x31,         // USAGE (y)
  0x16, Y_AXIS_LOG_MIN & 0xFF, (Y_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
  0x27, Y_AXIS_LOG_MAX & 0xFF, (Y_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, Y_AXIS_PHYS_MAX & 0xFF, (Y_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(X_AXIS_PHYS_MAX >> 16) & 0xFF,(X_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
  0x75, Y_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,           // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  0x09, 0x32,         // USAGE (z)
  0x16, Z_AXIS_LOG_MIN & 0xFF, (Z_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
  0x27, Z_AXIS_LOG_MAX & 0xFF, (Z_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, Z_AXIS_PHYS_MAX & 0xFF, (Z_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(Z_AXIS_PHYS_MAX >> 16) & 0xFF,(Z_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
  0x75, Z_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,           // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  0x09, 0x33,         // USAGE (rx)
  0x16, RX_AXIS_LOG_MIN & 0xFF, (RX_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
  0x27, RX_AXIS_LOG_MAX & 0xFF, (RX_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, RX_AXIS_PHYS_MAX & 0xFF, (RX_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(RX_AXIS_PHYS_MAX >> 16) & 0xFF,(RX_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
  0x75, RX_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,           // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  0x09, 0x34,         // USAGE (ry)
  0x16, RY_AXIS_LOG_MIN & 0xFF, (RY_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
  0x27, RY_AXIS_LOG_MAX & 0xFF, (RY_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
  0x35, 0x00,         // PHYSICAL_MINIMUM (00)
  0x47, RY_AXIS_PHYS_MAX & 0xFF, (RY_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(RY_AXIS_PHYS_MAX >> 16) & 0xFF,(RY_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
  0x75, RY_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
  0x95, 0x01,           // REPORT_COUNT (1)
  0x81, 0x02,         // INPUT (Data,Var,Abs)

  /*0x09, 0x35,         // USAGE (rz)
    0x16, RZ_AXIS_LOG_MIN & 0xFF, (RZ_AXIS_LOG_MIN >> 8) & 0xFF, // LOGICAL_MINIMUM
    0x27, RZ_AXIS_LOG_MAX & 0xFF, (RZ_AXIS_LOG_MAX >> 8) & 0xFF, 0, 0, // LOGICAL_MAXIMUM
    0x35, 0x00,         // PHYSICAL_MINIMUM (00)
    0x47, RZ_AXIS_PHYS_MAX & 0xFF, (RZ_AXIS_PHYS_MAX >> 8) & 0xFF, 0, 0,//(RZ_AXIS_PHYS_MAX >> 16) & 0xFF,(RZ_AXIS_PHYS_MAX >> 24) & 0xFF, // PHYSICAL_MAXIMUM (0xffff)
    0x75, RZ_AXIS_NB_BITS,   // REPORT_SIZE (AXIS_NB_BITS)
    0x95, 0x01,           // REPORT_COUNT (1)
    0x81, 0x02,         // INPUT (Data,Var,Abs)*/

  //0xc0, // END_COLLECTION

  0x09, 0x39,                     // USAGE (HAT SWITCH)
  0x15, 0x01,                     // LOGICAL_MINIMUM (1)
  0x25, 0x08,                     // LOGICAL_MAXIMUM (8)
  0x35, 0x00,                     // PHYSICAL_MINIMUM (0)
  0x46, 0x3B, 0x01,               // PHYSICAL_MAXIMUM (315)
  0x65, 0x14,                     // UNIT (Eng Rot:Angular Pos)
  0x55, 0x00,                     // UNIT_EXPONENT (0)
  0x75, 0x04,                     // REPORT_SIZE (4)
  0x95, 0x01,                     // REPORT_COUNT (1)
  0x81, 0x02,                     // Input (Data,Var,Abs)

  0x05, 0x09,                     // USAGE_PAGE (Button)
  0x15, 0x00,                     // LOGICAL_MINIMUM (0)
  0x25, 0x01,                     // LOGICAL_MAXIMUM (1)
  0x55, 0x00,                     // UNIT_EXPONENT (0)
  0x65, 0x00,                     // UNIT (None)
  0x19, 0x01,					            // USAGE_MINIMUM (button 1)
  0x29, NB_BUTTONS,               // USAGE_MAXIMUM (button NB_BUTTONS)
  0x75, 0x01,                     // REPORT_SIZE (1)
  0x95, NB_BUTTONS,			          // REPORT_COUNT (NB_BUTTONS)
  0x81, 0x02,                     // Input (Data,Var,Abs)

  // FOR CONFIG PROFILE
  0x85, 0xf1,                    //   REPORT_ID (f1)
  0x09, 0x01,                    //   USAGE (Vendor Usage 1)
  0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
  0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
  0x95, 0x3F, //0x20,            //   REPORT_COUNT (63)
  0x75, 0x08,                    //   REPORT_SIZE (8)
  0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)	//8

  0x85, 0xf2,                    //   REPORT_ID (f2)
  0x09, 0x01,                    //   USAGE (Vendor Usage 3)
  0x95, 0x3F, //0x20,            //   REPORT_COUNT (63)
  0x75, 0x08,                    //   REPORT_SIZE (8)
  0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)	8
  0xc0, // END_COLLECTION


  //FFB part (PID) starts from here
  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x92,	// USAGE (PID State Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x02,	// REPORT_ID (02)
  0x09, 0x9F,	// USAGE (Device Paused)
  0x09, 0xA0,	// USAGE (Actuators Enabled)
  0x09, 0xA4,	// USAGE (Safety Switch)
  0x09, 0xA5,	// USAGE (Actuator Override Switch)
  0x09, 0xA6,	// USAGE (Actuator Power)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MINIMUM (01)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
  0x75, 0x01,	// REPORT_SIZE (01)
  0x95, 0x05,	// REPORT_COUNT (05)
  0x81, 0x02,	// INPUT (Data,Var,Abs)
  0x95, 0x03,	// REPORT_COUNT (03)
  0x81, 0x03,	// INPUT (Constant,Var,Abs)
  0x09, 0x94,	// USAGE (Effect Playing)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MAXIMUM (01)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
  0x75, 0x01,	// REPORT_SIZE (01)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x81, 0x02,	// INPUT (Data,Var,Abs)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x07,	// REPORT_SIZE (07)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x81, 0x02,	// INPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x09, 0x21,	// USAGE (Set Effect Output Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x01,	// REPORT_ID (01)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x25,	// USAGE (Effect type)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x09, 0x26,	// USAGE (ET Constant Force)
  0x09, 0x27,	// USAGE (ET Ramp)
  0x09, 0x30,	// USAGE (ET Square)
  0x09, 0x31,	// USAGE (ET Sine)
  0x09, 0x32,	// USAGE (ET Triangle)
  0x09, 0x33,	// USAGE (ET Sawtooth Up)
  0x09, 0x34,	// USAGE (ET Sawtooth Down)
  0x09, 0x40,	// USAGE (ET Spring)
  0x09, 0x41,	// USAGE (ET Damper)
  0x09, 0x42,	// USAGE (ET Inertia)
  0x09, 0x43,	// USAGE (ET Friction)
  //0x09, 0x28,	// USAGE (ET Custom Force Data) //milos, removed custom force effect block
  //0x25, 0x0C,	// LOGICAL_MAXIMUM (12)
  0x25, 0x0B,  // LOGICAL_MAXIMUM (11) //milos, 1 less effect
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  //0x45, 0x0C,	// PHYSICAL_MAXIMUM (12)
  0x45, 0x0B,  // PHYSICAL_MAXIMUM (11) //milos, 1 less effect
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x00,	// OUTPUT (Data)
  0xC0,	// END COLLECTION ()
  0x09, 0x50,	// USAGE (Duration)
  0x09, 0x54,	// USAGE (Trigger Repeat Interval)
  //0x09, 0x51,	// USAGE (Sample Period) //milos, commented
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x27, 0xFF, 0xFF, 0x00, 0x00,	// LOGICAL_MAXIMUM (65535)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
  0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
  0x55, 0xFD,  // UNIT_EXPONENT (-3)
  0x75, 0x10,	// REPORT_SIZE (16)
  //0x95, 0x03,	// REPORT_COUNT (03)
  0x95, 0x02,  // REPORT_COUNT (02) //milos
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0x66, 0x00, 0x00,	// UNIT (None)
  0x09, 0x52,	// USAGE (Gain)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  //0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x46, 0x10, 0x27,	// PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  //0x75, 0x08,	// REPORT_SIZE (08)
  0x75, 0x10,  // REPORT_SIZE (16) //milos
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x53,	// USAGE (Trigger Button)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x08,	// LOGICAL_MAXIMUM (08)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x08,	// PHYSICAL_MAXIMUM (08)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x55,	// USAGE (Axes Enable)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
  0x09, 0x30,	// USAGE (X)
#ifdef NB_FF_AXIS>1
  0x09, 0x31,	// USAGE (Y)
#endif
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MAXIMUM (01)
  0x75, 0x01,	// REPORT_SIZE (01)
  0x95, NB_FF_AXIS,	// REPORT_COUNT (NB_FF_AXIS)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x56,	// USAGE (Direction Enable)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x95, 0x07 - NB_FF_AXIS,	// REPORT_COUNT (05 (2 axes) or 06 (1 axes)) seems to be for padding
  0x91, 0x03,	// OUTPUT (Constant,Var,Abs)
  0x09, 0x57,	// USAGE (Direction)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x0B, 0x01, 0x00, 0x0A, 0x00, // USAGE (Ordinals:Instance 1)
  0x0B, 0x02, 0x00, 0x0A, 0x00, // USAGE (Ordinals:Instance 2)
  0x66, 0x14, 0x00,	// UNIT (Eng Rot:Angular Pos)
  0x55, 0xFE,  // UNIT_EXPONENT (-2) //milos, 16bit
  //0x55, 0x00, // UNIT_EXPONENT (0) //milos, 8bit
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos, 16bit
  //0x26, 0xFF, 0x00,  // LOGICAL_MAXIMUM (255) //milos, 8bit
  0x35, 0x00,	// PHYSICAL_MINIMUM (0)
  0x47, 0x9F, 0x8C, 0x00, 0x00,	// PHYSICAL_MAXIMUM (35999) //milos, 16bit
  //0x46, 0x67, 0x01,  // PHYSICAL_MAXIMUM (359) //milos, 8bit
  0x75, 0x10,  // REPORT_SIZE (16) //milos, 16bit
  //0x75, 0x08,  // REPORT_SIZE (08) //milos, 8bit
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0x66, 0x00, 0x00,	// UNIT (None)
  0xC0,	// END COLLECTION ()
  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0xA7,	// USAGE (Start Delay) //milos, uncommented
  0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
  0x55, 0xFD,  // UNIT_EXPONENT (-3)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x27, 0xFF, 0xFF, 0x00, 0x00,	// LOGICAL_MAXIMUM (65535)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs) //milos, uncommented
  0x66, 0x00, 0x00,	// UNIT (None)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0xC0,	// END COLLECTION ()

  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x5A,	// USAGE (Set Envelope Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x02,	// REPORT_ID (02)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x5B,	// USAGE (Attack Level)
  0x09, 0x5D,	// USAGE (Fade Level)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x00,  // LOGICAL_MAXIMUM (255)
  //0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x08, // REPORT_SIZE (08)
  0x95, 0x02,	// REPORT_COUNT (02)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x5C,	// USAGE (Attack Time)
  0x09, 0x5E,	// USAGE (Fade Time)
  0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
  0x55, 0xFD,  // UNIT_EXPONENT (-3)
  //0x26, 0xFF, 0xFF,	// LOGICAL_MAXIMUM (65535)
  //0x46, 0xFF, 0xFF,	// PHYSICAL_MAXIMUM (65535)
  0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos
  0x46, 0xFF, 0x7F, // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x02, // REPORT_COUNT (02) //milos, added
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x66, 0x00, 0x00,	// UNIT (None)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0xC0,	// END COLLECTION ()

  0x09, 0x5F,	// USAGE (Set Condition Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x03,	// REPORT_ID (03)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x23,	// USAGE (Parameter Block Offset)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MAXIMUM (01)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
  0x75, 0x04,	// REPORT_SIZE (04)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x58,	// USAGE (Type Specific Block Offset)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x0B, 0x01, 0x00, 0x0A, 0x00,	// USAGE (Instance 1)
  0x0B, 0x02, 0x00, 0x0A, 0x00,	// USAGE (Instance 2)
  0x75, 0x02,	// REPORT_SIZE (02)
  0x95, 0x02,	// REPORT_COUNT (02)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()
  0x09, 0x60, // USAGE (CP Offset)
  //0x15, 0x80,	// LOGICAL_MINIMUM (-128)
  0x16, 0x00, 0x80,  // LOGICAL_MINIMUM (-32768) //milos
  //0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
  0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos
  //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000)
  0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  //0x75, 0x08,	// REPORT_SIZE (08)
  0x75, 0x10, // REPORT_SIZE (16) //milos
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x61, // USAGE (Positive Coefficient)
  //0x36, 0xF0, 0xD8,	// PHYSICAL_MINIMUM (-10000)
  0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  // dustin's rig, added - Negative Coefficient is now a real wire field (was declared in the
  // struct but never sent - only positiveCoefficient was ever used, in both directions).
  // Same LOGICAL/PHYSICAL_MIN/MAX still active from Positive Coefficient above (unchanged).
  0x09, 0x62,	// USAGE (Negative Coefficient)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  //0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x09, 0x63,	// USAGE (Positive Saturation) //milos, uncommented
  0x75, 0x10,	// REPORT_SIZE (16) //milos
  0x95, 0x01,	// REPORT_COUNT (01) //milos, uncommented
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  // dustin's rig, added - Negative Saturation, same LOGICAL/PHYSICAL_MIN/MAX still active
  // from Positive Saturation above (0..32767, unchanged)
  0x09, 0x64,	// USAGE (Negative Saturation)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x65,	// USAGE (Dead Band ) //milos, uncommented
  0x15, 0x00,  // LOGICAL_MINIMUM (00) //milos
  // dustin's rig, widened from 8-bit (0..255) to 16-bit (0..32767) - every other field in this
  // report (CP Offset, Coefficients, Saturation) is 16-bit; Dead Band being the odd one out at
  // 8 bits is a real divergence from common reference implementations (e.g. vJoy sends Dead
  // Band as 16-bit too) that a PID consumer assuming a uniform field width across the whole
  // Condition block could choke on, silently failing Spring/Damper/Inertia/Friction entirely
  // while simpler effects (Constant Force etc, no Condition report needed) keep working fine -
  // matching a "device is seen, but no FFB" symptom from some games.
  0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x10, // REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01) //milos, uncommented
  0x91, 0x02,	// OUTPUT (Data,Var,Abs) //milos, uncommented
  0xC0,	// END COLLECTION ()

  0x09, 0x6E,	// USAGE (Set Periodic Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x04,	// REPORT_ID (04)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x70,	// USAGE (Magnitude)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  //0x26, 0xFF, 0x00,   // LOGICAL_MAXIMUM (255)
  0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F, // PHYSICAL_MAXIMUM (32767) //milos
  //0x75, 0x08,	// REPORT_SIZE (08)
  0x75, 0x10,  // REPORT_SIZE (16) //milos
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x6F,	// USAGE (Offset)
  //0x15, 0x80,	// LOGICAL_MINIMUM (-128)
  0x16, 0x00, 0x80,  // LOGICAL_MINIMUM (-32768) //milos
  //0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
  0x26, 0xFF, 0x7F,   // LOGICAL_MAXIMUM (32737) //milos
  //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000)
  0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x71,	// USAGE (Phase)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  //0x27, 0xFF, 0xFF, 0x00, 0x00,  // LOGICAL_MAXIMUM (65535) //milos
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  //0x47, 0x9F, 0x8C, 0x00, 0x00,	// PHYSICAL_MAXIMUM (35999) //milos
  0x46, 0x67, 0x01,  // PHYSICAL_MAXIMUM (359) //milos
  0x66, 0x14, 0x00,  // UNIT (Eng Rot:Angular Pos)
  //0x55, 0xFE, // UNIT_EXPONENT (-2)
  0x55, 0x00, // UNIT_EXPONENT (0) //milos
  0x75, 0x08, // REPORT_SIZE (08) //milos
  0x95, 0x01, // REPORT_COUNT (01) //milos
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x72,	// USAGE (Period)
  //0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767)
  0x27, 0xFF, 0xFF, 0x00, 0x00,  // LOGICAL_MAXIMUM (65535) //milos
  //0x46, 0xFF, 0x7F,	// PHYSICAL_MAXIMUM (32767)
  0x47, 0xFF, 0xFF, 0x00, 0x00, // PHYSICAL_MAXIMUM (65535) //milos
  0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
  0x55, 0xFD,  // UNIT_EXPONENT (-3)
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x66, 0x00, 0x00,	// UNIT (None)
  0x55, 0x00,	// UNIT_EXPONENT (00)
  0xC0,	// END COLLECTION ()

  0x09, 0x73,	// USAGE (Set Constant Force Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x05,	// REPORT_ID (05)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x70,	// USAGE (Magnitude)
  //0x16, 0x01, 0xFF,	// LOGICAL_MINIMUM (-255)
  0x16, 0x01, 0x80,  // LOGICAL_MINIMUM (-32767) //milos, my original
  //0x16, 0xF0, 0xD8,  // LOGICAL_MINIMUM (-10000) //milos, testing
  //0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos, my original
  //0x26, 0x10, 0x27, // LOGICAL_MAXIMUM (10000) //milos, testing
  //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000) //milos, testing
  0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos, my original
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000) //milos, testing
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos, my original
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x09, 0x74,	// USAGE (Set Ramp Force Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x06,	// REPORT_ID (06)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x75,	// USAGE (Ramp Start)
  0x09, 0x76,	// USAGE (Ramp End)
  0x15, 0x81,	// LOGICAL_MINIMUM (-127)
  0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
  //0x36, 0xF0, 0xD8,  // PHYSICAL_MINIMUM (-10000)
  0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos
  //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x02,	// REPORT_COUNT (02)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  //milos, commented since it was not used
  /*0x09, 0x68,	// USAGE (Custom Force Data Report)
    0xA1, 0x02,	// COLLECTION (Logical)
    0x85, 0x07,	// REPORT_ID (07)
    0x09, 0x22,	// USAGE (Effect Block Index)
    0x15, 0x01,	// LOGICAL_MINIMUM (01)
    0x25, 0x28,	// LOGICAL_MAXIMUM (40)
    0x35, 0x01,	// PHYSICAL_MINIMUM (01)
    0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x09, 0x6C,	// USAGE (Custom Force Data Offset)
    //0x15, 0x00,	// LOGICAL_MINIMUM (00)
    0x16, 0x00, 0x80,  // LOGICAL_MINIMUM (-32768) //milos
    //0x26, 0x10, 0x27,	// LOGICAL_MAXIMUM (10000)
    0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767) //milos
    //0x35, 0x00,	// PHYSICAL_MINIMUM (00)
    0x36, 0x00, 0x80,  // PHYSICAL_MINIMUM (-32768) //milos
    //0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
    0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
    0x75, 0x10,	// REPORT_SIZE (16)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x09, 0x69,	// USAGE (Custom Force Data)
    0x15, 0x81,	// LOGICAL_MINIMUM (-127)
    0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
    //0x35, 0x00,	// PHYSICAL_MINIMUM (00)
    0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos
    //0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
    0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x0C,	// REPORT_COUNT (12)
    0x92, 0x02, 0x01,	// OUTPUT (Data,Var,Abs,Buf)
    0xC0,	// END COLLECTION ()*/

  //milos, commented since it was not used
  /*0x09, 0x66,	// USAGE (Download Force Sample)
    0xA1, 0x02,	// COLLECTION (Logical)
    0x85, 0x08,	// REPORT_ID (08)
    0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
    0x09, 0x30,	// USAGE (X)
    0x09, 0x31,	// USAGE (Y)
    0x15, 0x81,	// LOGICAL_MINIMUM (-127)
    0x25, 0x7F,	// LOGICAL_MAXIMUM (127)
    //0x35, 0x00, // PHYSICAL_MINIMUM (00)
    0x36, 0x01, 0x80,  // PHYSICAL_MINIMUM (-32767) //milos
    //0x46, 0xFF, 0x00, // PHYSICAL_MAXIMUM (255)
    0x46, 0xFF, 0x7F,  // PHYSICAL_MAXIMUM (32767) //milos
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x02,	// REPORT_COUNT (02)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0xC0,	// END COLLECTION ()*/

  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x77,	// USAGE (Effect Operation Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x0A,	// REPORT_ID (10)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0x09, 0x78,	// USAGE (78)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x09, 0x79,	// USAGE (Op Effect Start)
  0x09, 0x7A,	// USAGE (Op Effect Start Solo)
  0x09, 0x7B,	// USAGE (Op Effect Stop)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x03,	// LOGICAL_MAXIMUM (03)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x00,	// OUTPUT (Data,Ary,Abs)
  0xC0,	// END COLLECTION ()
  0x09, 0x7C,	// USAGE (Loop Count)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x09, 0x90,	// USAGE (PID Block Free Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x0B,	// REPORT_ID (11)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x15, 0x01, // LOGICAL_MINIMUM (01)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  0x09, 0x96,	// USAGE (PID Device Control)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x0C,	// REPORT_ID (12)
  0x09, 0x97,	// USAGE (DC Enable Actuators)
  0x09, 0x98,	// USAGE (DC Disable Actuators)
  0x09, 0x99,	// USAGE (DC Stop All Effects)
  0x09, 0x9A,	// USAGE (DC Device Reset)
  0x09, 0x9B,	// USAGE (DC Device Pause)
  0x09, 0x9C,	// USAGE (DC Device Continue)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x25, 0x06,	// LOGICAL_MAXIMUM (06)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x00,	// OUTPUT (Data)
  0xC0,	// END COLLECTION ()

  0x09, 0x7D,	// USAGE (Device Gain Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x0D,	// REPORT_ID (14)
  0x09, 0x7E,	// USAGE (Device Gain)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x00,  // LOGICAL_MAXIMUM (255)
  //0x26, 0xFF, 0x7F, // LOGICAL_MAXIMUM (32767) //milos, back to 8bit
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x46, 0x10, 0x27,  // PHYSICAL_MAXIMUM (10000)
  //0x46, 0xFF, 0x7F, // PHYSICAL_MAXIMUM (32767) //milos
  0x75, 0x08,	// REPORT_SIZE (08)
  //0x75, 0x10,  // REPORT_SIZE (16) //milos, back to 8bit
  0x95, 0x01,	// REPORT_COUNT (01)
  0x91, 0x02,	// OUTPUT (Data,Var,Abs)
  0xC0,	// END COLLECTION ()

  //milos, commented since it was not used
  /*0x09, 0x6B,	// USAGE (Set Custom Force Report)
    0xA1, 0x02,	// COLLECTION (Logical)
    0x85, 0x0E,	// REPORT_ID (14)
    0x09, 0x22,	// USAGE (Effect Block Index)
    0x15, 0x01,	// LOGICAL_MINIMUM (01)
    0x25, 0x28,	// LOGICAL_MAXIMUM (28)
    0x35, 0x01,	// PHYSICAL_MINIMUM (01)
    0x45, 0x28,	// PHYSICAL_MAXIMUM (28)
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x09, 0x6D,	// USAGE (Sample Count)
    0x15, 0x00,	// LOGICAL_MINIMUM (00)
    0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
    0x35, 0x00,	// PHYSICAL_MINIMUM (00)
    0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
    0x75, 0x08,	// REPORT_SIZE (08)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x09, 0x51,	// USAGE (Sample Period)
    0x66, 0x01, 0x10,  // UNIT (SI Lin:Time)
    0x55, 0xFD,  // UNIT_EXPONENT (-3)
    0x15, 0x00,	// LOGICAL_MINIMUM (00)
    0x26, 0xFF, 0x7F,	// LOGICAL_MAXIMUM (32767)
    0x35, 0x00,	// PHYSICAL_MINIMUM (00)
    0x46, 0xFF, 0x7F,	// PHYSICAL_MAXIMUM (32767)
    0x75, 0x10,	// REPORT_SIZE (16)
    0x95, 0x01,	// REPORT_COUNT (01)
    0x91, 0x02,	// OUTPUT (Data,Var,Abs)
    0x55, 0x00,	// UNIT_EXPONENT (00)
    0x66, 0x00, 0x00,	// UNIT (None)
    0xC0,	// END COLLECTION ()*/

  0x09, 0xAB,	// USAGE (Create New Effect Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x05,	// REPORT_ID (05)
  0x09, 0x25,	// USAGE (Effect Type)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x09, 0x26, // USAGE (ET Constant Force)
  0x09, 0x27, // USAGE (ET Ramp)
  0x09, 0x30, // USAGE (ET Square)
  0x09, 0x31, // USAGE (ET Sine)
  0x09, 0x32, // USAGE (ET Triangle)
  0x09, 0x33, // USAGE (ET Sawtooth Up)
  0x09, 0x34, // USAGE (ET Sawtooth Down)
  0x09, 0x40, // USAGE (ET Spring)
  0x09, 0x41, // USAGE (ET Damper)
  0x09, 0x42, // USAGE (ET Inertia)
  0x09, 0x43, // USAGE (ET Friction)
  //0x09, 0x28,	// USAGE (ET Custom Force data) //milos, removed custom force effect
  0x15, 0x01,  // LOGICAL_MINIMUM (01)
  0x25, 0x0B,	// LOGICAL_MAXIMUM (11) //milos, was 0x0C (12)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x0B,	// PHYSICAL_MAXIMUM (11) //milos, was 0x0C (12)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x00,	// FEATURE (Data)
  0xC0,	// END COLLECTION ()
  0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
  0x09, 0x3B,	// USAGE (Byte Count)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x26, 0xFF, 0x01,	// LOGICAL_MAXIMUM (511)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x46, 0xFF, 0x01,	// PHYSICAL_MAXIMUM (511)
  0x75, 0x0A,	// REPORT_SIZE (10)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x75, 0x06,	// REPORT_SIZE (06)
  0xB1, 0x01,	// FEATURE (Constant,Ary,Abs)
  0xC0,	// END COLLECTION ()

  0x05, 0x0F,	// USAGE_PAGE (Physical Interface)
  0x09, 0x89,	// USAGE (PID Block Load Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x06,	// REPORT_ID (06)
  0x09, 0x22,	// USAGE (Effect Block Index)
  0x25, 0x28,	// LOGICAL_MAXIMUM (40)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x28,	// PHYSICAL_MAXIMUM (40)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x09, 0x8B,	// USAGE (Block Load Status)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x09, 0x8C,	// USAGE (Block Load Success)
  0x09, 0x8D,	// USAGE (Block Load Full)
  0x09, 0x8E,	// USAGE (Block Load Error)
  0x25, 0x03,	// LOGICAL_MAXIMUM (03)
  0x15, 0x01,	// LOGICAL_MINIMUM (01)
  0x35, 0x01,	// PHYSICAL_MINIMUM (01)
  0x45, 0x03,	// PHYSICAL_MAXIMUM (03)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x00,	// FEATURE (Data)
  0xC0,	// END COLLECTION ()
  0x09, 0xAC,	// USAGE (RAM Pool Available)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x27, 0xFF, 0xFF, 0x00, 0x00,	// LOGICAL_MAXIMUM (65535)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x00,	// FEATURE (Data)
  0xC0,	// END COLLECTION ()

  0x09, 0x7F,	// USAGE (PID Pool Report)
  0xA1, 0x02,	// COLLECTION (Logical)
  0x85, 0x07,	// REPORT_ID (07)
  0x09, 0x80,	// USAGE (RAM Pool Size)
  0x75, 0x10,	// REPORT_SIZE (16)
  0x95, 0x01,	// REPORT_COUNT (01)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x27, 0xFF, 0xFF, 0x00, 0x00,  // LOGICAL_MAXIMUM (65535)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x47, 0xFF, 0xFF, 0x00, 0x00,	// PHYSICAL_MAXIMUM (65535)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x09, 0x83,	// USAGE (Simultaneous Effects Max)
  0x26, 0xFF, 0x00,	// LOGICAL_MAXIMUM (255)
  0x46, 0xFF, 0x00,	// PHYSICAL_MAXIMUM (255)
  0x75, 0x08,	// REPORT_SIZE (08)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x09, 0xA9,	// USAGE (Device Managed Pool)
  0x09, 0xAA,	// USAGE (Shared Parameter Blocks)
  0x75, 0x01,	// REPORT_SIZE (01)
  0x95, 0x02,	// REPORT_COUNT (02)
  0x15, 0x00,	// LOGICAL_MINIMUM (00)
  0x25, 0x01,	// LOGICAL_MAXIMUM (01)
  0x35, 0x00,	// PHYSICAL_MINIMUM (00)
  0x45, 0x01,	// PHYSICAL_MAXIMUM (01)
  0xB1, 0x02,	// FEATURE (Data,Var,Abs)
  0x75, 0x06,	// REPORT_SIZE (06)
  0x95, 0x01,	// REPORT_COUNT (01)
  0xB1, 0x03,	// FEATURE ( Cnst,Var,Abs)
  0xC0,	// END COLLECTION ()

  0xC0,	// END COLLECTION ()
#endif
};

extern const HIDDescriptor _hidInterface PROGMEM;
const HIDDescriptor _hidInterface =
{
  D_INTERFACE(HID_INTERFACE, HID_ENPOINT_COUNT, 3, 0, 0), //0xEF,0x02,0x01),
  D_HIDREPORT(sizeof(_hidReportDescriptor)),
  D_ENDPOINT(USB_ENDPOINT_IN(HID_ENDPOINT_INT), USB_ENDPOINT_TYPE_INTERRUPT, 0x40, 0x01),
#if HID_ENPOINT_COUNT>1
  D_ENDPOINT(USB_ENDPOINT_OUT(HID_ENDPOINT_OUT), USB_ENDPOINT_TYPE_INTERRUPT, 0x40, 0x01)
#endif
};

//================================================================================
//================================================================================
//	Driver

u8 _hid_protocol = 1;
u8 _hid_idle = 1;

#define WEAK __attribute__ ((weak))

int WEAK HID_GetInterface(u8* interfaceNum)
{
  interfaceNum[0] += 1;	// uses 1
  return USB_SendControl(TRANSFER_PGM, &_hidInterface, sizeof(_hidInterface));
}

int WEAK HID_GetDescriptor(int i)
{
  return USB_SendControl(TRANSFER_PGM, _hidReportDescriptor, sizeof(_hidReportDescriptor));
}

void WEAK HID_SendReport(u8 id, const void* data, int len)
{
  USB_Send(HID_TX, &id, 1);
  USB_Send(HID_TX | TRANSFER_RELEASE, data, len);
}

u8 WEAK HID_ReportAvailable()
{
  return (USB_Available(HID_RX));
}

s16 WEAK HID_ReceiveReport(void* data, int len)
{
  return (USB_Recv(HID_RX, data, len));
}

b8 WEAK HID_Setup(Setup& setup)
{
  u8 r = setup.bRequest;
  u8 requestType = setup.bmRequestType;
  if (REQUEST_DEVICETOHOST_CLASS_INTERFACE == requestType)
  {
    if (HID_GET_REPORT == r)
    {
      return true;
    }
    if (HID_GET_PROTOCOL == r)
    {
      //Send8(_hid_protocol);	// TODO
      return true;
    }
  }

  if (REQUEST_HOSTTODEVICE_CLASS_INTERFACE == requestType)
  {
    switch (r)
    {
      case HID_SET_PROTOCOL:
        _hid_protocol = setup.wValueL;
        return true;

      case HID_SET_IDLE:
        _hid_idle = setup.wValueL;
        return true;

      case HID_SET_REPORT:
        return true;
    }
  }
  return false;
}

//================================================================================
//================================================================================
// Joystick
//  Usage: Joystick.move(x, y, throttle, buttons)
//  x & y forward/left = 0, centre = 127, back/right = 255
//  throttle max = 0, min = 255
//  8 buttons packed into 1 byte

Joystick_::Joystick_()
{
}

// milos ver4, 16+16+12+12+12 bits version + 28 buttons
void Joystick_::send_16_16_12_12_12_28(uint16_t x, uint16_t y, uint16_t z, uint16_t rx, uint16_t ry, uint32_t buttons)
{
  // milos, total of 12 bytes, 2B for x, 2B for y, 3B for z and rx, 5B for ry and buttons
  u8 j[12];
  j[0] = x;
  j[1] = x >> 8;
  j[2] = y;
  j[3] = y >> 8;
  j[4] = z;
  j[5] = (z >> 8) & 0xf | ((rx & 0xf) << 4);
  j[6] = rx >> 4;
  j[7] = ry;
  j[8] = (ry >> 8) & 0xf | ((buttons & 0xf) << 4);
  j[9] = buttons >> 4;
  j[10] = buttons >> 12;
  j[11] = buttons >> 20;

  HID_SendReport(4, j, 12);
}

#endif

#endif /* if defined(USBCON) */
