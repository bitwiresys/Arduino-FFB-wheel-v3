/*
  Force Feedback Joystick
  Basic debugging utilities.

  This code is for Microsoft Sidewinder Force Feedback Pro joystick
  with some room for additional extra controls.

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  MIT License.

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#ifndef _DEBUG_H_
#define _DEBUG_H_

// dustin's rig, removed - gDebugMode/DoDebug and the old debug-target constants:
// nothing ever read them (all Log* bodies compile out unless DEBUG_FFB is defined).

// If below are defined, code for respective debug target is included into build
#define DEBUG_ENABLE_UART
//#define DEBUG_ENABLE_USB

#define DEBUG_BUFFER_SIZE 512 // milos, you may want to reduce this if your run out of memory

// Debugging utilities

// The basic debug data sending method used by all other methods.
// Implement this to send debug data to desired destination.
void LogSendByte(u8 data);

// Send out the given null terminated text
void LogText(const char *text);
void LogTextLf(const char *text);	// Adds linefeed
void LogTextP(const char *text);	// From program memory
void LogTextLfP(const char *text);	// From program memory, adds linefeed

// Send out the given binary data
void LogBinary(const void *data, uint16_t len);
void LogBinaryLf(const void *data, uint16_t len);	// Adds linefeed


#define DEBUG_SERIAL		Serial
#define CONFIG_SERIAL		Serial

//#define DEBUG_FFB // milos, uncomment this to enable FFB debugging via UART
//#define DEBUG_CALIBRATION

#ifdef DEBUG_CALIBRATION
#define cal_print(str)			DEBUG_SERIAL.print(str)
#define cal_println(str)		DEBUG_SERIAL.println(str)
#else
#define cal_print(str)
#define cal_println(str)
#endif

#endif // _DEBUG_H_
