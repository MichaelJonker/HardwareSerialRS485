/* HardwareSerial_RS485.h copyright notice

Replacement of the classical HardwareSerial version for Wiring by Nicholas Zambetti, Mark Sproul, Alarus.

Copyright (c) 2014 Michael Jonker.  All right reserved.

 http://en.wikipedia.org/wiki/Beerware
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Michael Jonker <EPID: 52.36040, 4.87001, 5m, AD19540508.621> wrote this file.
 * As long as you retain this notice you can do whatever you want with this stuff.
 * If we meet some day, and you think this stuff is worth it, you can buy me a beer in return.

This code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details. You can obtain a copy of this license
from the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#if !defined(HardwareSerial_RS485_h)
#define      HardwareSerial_RS485_h


#if !defined(HardwareSerial_use_classical_version)
#if !defined(HardwareSerial_h)
#define HardwareSerial_h /* block loading the classsical version of HardwareSerial */
#else
#error "HardwareSerial.h already loaded. The file HardwareSerial_RS485.h must be included before the #include "Arduino.h" (whether explicit or implicit)."
#endif

#include <HardwareSerial_RS485Helper.h>
#include <HardwareSerial_RS485Enabled.h>

/* define Hardware Serial classes for all objects.
notes:
if defined_SerialClass<n> has been defined externally, this definition will be used. If not, a default will be chosen.
if the externally defined defined_SerialClass<n> is 'empty', then no Serial class will be created.
//todo include a user accessible configuration file, allowing to externalize the definition of the SerialClass(es) an to keep this header file 'clean'.
*/
#if defined(USBCON)
#include "HardwareSerial_USB.h"
extern HardwareSerial_USB Serial_;  
#endif
#if defined(UCSR0A)
    #if !defined(defined_SerialClass0)
        #define  defined_SerialClass0  HardwareSerial<USART0, 6, 6, ' ', RS485serial< >, MessageFilterRT >
    #endif
    //#if defined_SerialClass0  //todo find a way to implement a test like #if defined_SerialClass0 != null
        typedef defined_SerialClass0 SerialClass0;
        extern SerialClass0 Serial0;
    //#endif
#endif
#if defined(UCSR1A)
    #if !defined(defined_SerialClass1)
        #define  defined_SerialClass1  HardwareSerial<USART1>
    #endif
    //#if defined_SerialClass1
        typedef defined_SerialClass1 SerialClass1;
        extern SerialClass1 Serial1;
    //#endif
#endif
#if defined(UCSR2A)
    #if !defined(defined_SerialClass2)
        #define  defined_SerialClass2  HardwareSerial<USART2>
    #endif
    //#if defined_SerialClass2
        typedef defined_SerialClass2 SerialClass2;
        extern SerialClass2 Serial2;
    //#endif
#endif
#if defined(UCSR3A)
    #if !defined(defined_SerialClass3)
        #define  defined_SerialClass3  HardwareSerial<USART3>
    #endif
    //#if defined_SerialClass3
        typedef defined_SerialClass3 SerialClass3;
        extern SerialClass3 Serial3;
    //#endif
#endif
// other examples:
//#define defined_SerialClass  HardwareSerial<USART0, 5, 6, ' ', RS485serial<> > // RS485 only, no message filter
//#define defined_SerialClass  HardwareSerial<USART0, 5, 6, ' ', RS485serial< TRxControl<'C', 2, 3> >, MessageFilterRT >
//#define defined_SerialClass  HardwareSerial<USART0, 5, 6, ' ', RS485_dummy>

#if defined(USBCON)
#define Serial Serial_
#else
#define Serial Serial0
#endif

#if defined(InHardwareSerial_RS485_CPP)
// link the hardware interrupt vectors to the interrupt methods of the HWSerial class
#if defined(defined_SerialClass0)
SerialClass0 Serial0;
link_USART_Vectors(USART0, Serial0);
void serialEvent0() __attribute__((weak));
void serialEvent0() {}
#endif
#if defined(defined_SerialClass1)
SerialClass1 Serial1;
link_USART_Vectors(USART1, Serial1);
void serialEvent1() __attribute__((weak));
void serialEvent1() {}
#endif
#if defined(defined_SerialClass2)
SerialClass2 Serial2;
link_USART_Vectors(USART2, Serial2);
void serialEvent2() __attribute__((weak));
void serialEvent2() {}
#endif
#if defined(defined_SerialClass3)
SerialClass3 Serial3;
link_USART_Vectors(USART3, Serial3);
void serialEvent3() __attribute__((weak));
void serialEvent3() {}
#endif

// define the USART classes
extern void  serialEventRun(void) __attribute__((weak));
void serialEventRun(void)
{
#if defined(defined_SerialClass0)
  if (Serial0.available()) serialEvent();
#endif
#if defined(defined_SerialClass1)
  if (Serial1.available()) serialEvent1();
#endif
#if defined(defined_SerialClass2)
  if (Serial2.available()) serialEvent2();
#endif
#if defined(defined_SerialClass3)
  if (Serial3.available()) serialEvent3();
#endif
}


#endif // defined(InHardwareSerial_RS485_CPP)

#endif // !defined(HardwareSerial_use_classical_version)

#endif // !defined(HardwareSerial_RS485_h)