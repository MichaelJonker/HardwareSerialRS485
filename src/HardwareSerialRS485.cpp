/* HardwareSerialRS485.cpp copyright notice

Replacement of the classical HardwareSerial version for Wiring by Nicholas Zambetti, Mark Sproul, Alarus

Copyright (c) 2015 Michael Jonker.  All right reserved.

 http://en.wikipedia.org/wiki/Beerware
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42++):
 * Michael Jonker <ERPID = {52.36040, 4.87001, NAP+5m, 1954.349629}> wrote this file.
 * As long as you retain this notice you can do whatever you want with this stuff within the limits of the GNU Lesser General Public License.
 * If we meet some day, and you think this stuff is worth it, you can buy me a beer in return.

This code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details. You can obtain a copy of this license
from the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define Implement_HardwareSerialRS485 
#include "HardwareSerialRS485.h"
// All static data and code of the HardwareSerialRS485 related classes are instantiated by compiling and linking this file.
// However, the actual code is located in HardwareSerialRS485.h, HardwareSerialRS485_Enabled.h, HardwareSerialRS485_Helper.h, etc
// This is a personal style choice for ease of maintenance (with minor overhead).

// Todo look at possibility of #pragma interface / #pragma implementation



/* testing options:
# options to compile outside Arduino environment (for debugging and code optimisation)

cd $Arduino

arduino-1.0.5/hardware/tools/avr/bin/avr-g++ -c -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -MMD -DARDUINO=105\
  -mmcu=atmega328p -DF_CPU=16000000L -MMD -DUSB_VID=null -DUSB_PID=null -DARDUINO=105 \
  -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard \
  -ImyProjects/libraries/HardwareSerialRS485 \
  -S myProjects/libraries/HardwareSerialRS485/HardwareSerialRS485.cpp

    
  -mmcu=atmega32u4 -DF_CPU=16000000L -DUSB_VID=0x2341 -DUSB_PID=0x8039 -Iarduino-1.0.5/hardware/arduino/cores/arduino/robot -Iarduino-1.0.5/hardware/arduino/variants/standard/robot_motor \
  -mmcu=atmega328p -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino       -Iarduino-1.0.5/hardware/arduino/variants/standard \
Board:
Mega(ATmega1280)                                          -mmcu=atmega1280 -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/mega             
BT w/ ATmega168                                           -mmcu=atmega168  -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/eightanaloginputs
Mini w/ ATmega168                                         -mmcu=atmega168  -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/eightanaloginputs
Nano w/ ATmega168                                         -mmcu=atmega168  -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/eightanaloginputs
Diecimila or Duemilanove w/ ATmega328lanova w/ ATmega168  -mmcu=atmega168  -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
NG or older w/ ATmega168                                  -mmcu=atmega168  -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
Pro or Pro Mini (5V,16 MHz) w/ ATmega168                  -mmcu=atmega168  -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
Lilipad w/ ATmega168                                      -mmcu=atmega168  -DF_CPU=8000000L  -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
Pro or Pro Mini (3.3V,8 MHz) w/ ATmega168                 -mmcu=atmega168  -DF_CPU=8000000L  -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
Mega 2560 or Mega ADK                                     -mmcu=atmega2560 -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/mega             
BT w/ ATmega328                                           -mmcu=atmega328p -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/eightanaloginputs
Mini w/ ATmega328                                         -mmcu=atmega328p -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/eightanaloginputs
Nano w/ ATmega328                                         -mmcu=atmega328p -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/eightanaloginputs
Duemilanove w/ ATmega328                                  -mmcu=atmega328p -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
Pro or Pro Mini (5V,16 MHz) w/ ATmega328                  -mmcu=atmega328p -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
Ethernet                                                  -mmcu=atmega328p -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
UNO                                                       -mmcu=atmega328p -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
Fio                                                       -mmcu=atmega328p -DF_CPU=8000000L  -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/eightanaloginputs
Lilipad w/ ATmega328                                      -mmcu=atmega328p -DF_CPU=8000000L  -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
Pro or Pro Mini (3.3V,8 MHz) w/ ATmega328                 -mmcu=atmega328p -DF_CPU=8000000L  -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         
Leonardo                                                  -mmcu=atmega32u4 -DF_CPU=16000000L -DUSB_VID=0x2341 -DUSB_PID=0x8036 -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/leonardo         
Micro                                                     -mmcu=atmega32u4 -DF_CPU=16000000L -DUSB_VID=0x2341 -DUSB_PID=0x8037 -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/micro            
Robot Control                                             -mmcu=atmega32u4 -DF_CPU=16000000L -DUSB_VID=0x2341 -DUSB_PID=0x8038 -Iarduino-1.0.5/hardware/arduino/cores/robot   -Iarduino-1.0.5/hardware/arduino/variants/robot_control    
Robot Motor                                               -mmcu=atmega32u4 -DF_CPU=16000000L -DUSB_VID=0x2341 -DUSB_PID=0x8039 -Iarduino-1.0.5/hardware/arduino/cores/robot   -Iarduino-1.0.5/hardware/arduino/variants/robot_motor      
Esplora                                                   -mmcu=atmega32u4 -DF_CPU=16000000L -DUSB_VID=0x2341 -DUSB_PID=0x803C -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/leonardo         
Lilipad USB                                               -mmcu=atmega32u4 -DF_CPU=8000000L  -DUSB_VID=0x1B4F -DUSB_PID=0x9208 -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/leonardo         
NG or older w/ ATmega8                                    -mmcu=atmega8    -DF_CPU=16000000L -DUSB_VID=null   -DUSB_PID=null   -Iarduino-1.0.5/hardware/arduino/cores/arduino -Iarduino-1.0.5/hardware/arduino/variants/standard         

# see http://gcc.gnu.org/wiki/avr-gcc
# see http://gcc.gnu.org/onlinedocs/gcc-4.3.6/gcc/Optimize-Options.html#Optimize-Options
arduino-1.0.5/hardware/tools/avr/bin/avr-g++ -v --help
arduino-1.0.5/hardware/tools/avr/bin/avr-gcc -v --help

g++ -E HardwareSerial.cpp > HardwareSerial.p   #preprocess only
g++ -S HardwareSerial.cpp                      #assemble only
g++ -c HardwareSerial.cpp                      #compile only
g++ -O0 -g  HardwareSerial.cpp                 #compile and link, no optimisation

g++ -mmcu=atmega328p -S HardwareSerial.cpp
g++ -mmcu=atmega328p -c HardwareSerial.cpp
g++ -mmcu=atmega328p    HardwareSerial.cpp
*/

