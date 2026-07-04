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

#include <arduino.h>
#include "debug.h"

// dustin's rig, removed - gDebugMode and the DEBUG_TO_* constants (nothing read them);
// all Log* bodies below compile to empty stubs unless DEBUG_FFB is defined in debug.h.

#ifdef DEBUG_ENABLE_USB
// Internal buffer for sending debug data to USB COM-port
volatile char debug_buffer[DEBUG_BUFFER_SIZE];
volatile uint16_t debug_buffer_used = 0;
#endif

void LogSendData(u8 *data, uint16_t len)
{
#ifdef DEBUG_FFB
  uint16_t i = 0;
  for (i = 0; i < len; i++)
  {
#ifdef DEBUG_DATA_AS_HEX
    static const char *hexmap = "0123456789ABCDEF";
    LogSendByte( ' ' );
    LogSendByte( hexmap[(data[i] >> 4)]);
    LogSendByte( hexmap[(data[i] & 0x0F)]);
#else
    LogSendByte(data[i]);
#endif
  }
#endif
}


void LogText(const char *text)
{
#ifdef DEBUG_FFB
  while (1)
  {
    char c = *text++;
    if (c == 0)
      break;
    if (c == '\n')
      LogSendByte('\r');	// CR
    LogSendByte(c);
  }
#endif
}

void LogTextLf(const char *text)
{
#ifdef DEBUG_FFB
  LogText(text);
  LogSendByte('\r');	// CR
  LogSendByte('\n');	// LF
#endif
}


void LogTextP(const char *text)
{
#ifdef DEBUG_FFB
  while (1)
  {
    char c = pgm_read_byte(text++);
    if (c == 0)
      break;
    if (c == '\n')
      LogSendByte('\r');	// CR
    LogSendByte(c);
  }
#endif
}

void LogTextLfP(const char *text)
{
#ifdef DEBUG_FFB
  LogTextP(text);
  LogSendByte('\r');	// CR
  LogSendByte('\n');	// LF
#endif
}


void LogBinary(const void *data, uint16_t len)
{
#ifdef DEBUG_FFB
  u8 temp = (u8) (len & 0xFF);
  if (temp > 0)
    LogSendData((u8*) data, temp);
#endif
}

void LogBinaryLf(const void *data, uint16_t len)
{
#ifdef DEBUG_FFB
  u8 temp = (u8) (len & 0xFF);
  if (temp > 0)
    LogSendData((u8*) data, temp);
  LogSendByte('\r');	// CR
  LogSendByte('\n');	// LF
#endif
}



// -------------------------------
// Send debug data to/from buffer
// to the chosen bus (USB or MIDI)

void LogSendByte(u8 data)
{
#ifdef DEBUG_FFB
  DEBUG_SERIAL.write(data);
#endif
}


