/* DisableHardwareSerial.h

Copyright (c) 2015 Michael Jonker.  All right reserved.

This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free
Software Foundation; either version 2.1 of the License, or (at your option)
any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


// To disable the HardwareSerial class, this file is to be included ahead of the Arduino.h header file. This can be achieved with a
// ...extra_flags=-include DisableHardwareSerial.h "-I{runtime.platform.path}" compile directive in the boards.txt file.
// Note: prior to the Arduino IDE 1.6.6, the disabling of HardwareSerial class was done by the HardwareSerialRS485.h header. With IDE 1.6.6
// the inclusion of Arduino.h has been moved to the very first line of the sketch, making this option impossible.
//
// for more information see https://github.com/MichaelJonker/HardwareSerialRS485/wiki/installation-and-deployment


// disables the inclusion of HardwareSerial.h
#define HardwareSerial_h

// this is still needed by main.cpp (serialEventRun is defined by HardwareSerialRS485.h)
extern void serialEventRun(void) __attribute__((weak));

// we also make up for the missing Stream.h in "USBAPI.h"
#include "Stream.h"

// to include HardwareSerialRS485.h unconditionally, uncomment the follwoing line
// you might also need to add an extra include directive in a extra_flag of the boards.txt file to add the folder library/HardwareSerialRS485/src
// #include "HardwareSerialRS485.h"

