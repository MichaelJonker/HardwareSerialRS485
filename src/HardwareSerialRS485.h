/* HardwareSerialRS485.h copyright notice

Replaces the classical HardwareSerial version for Wiring by Nicholas Zambetti, Mark Sproul, Alarus, and others.

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

#if !defined(HardwareSerialRS485_h)
#define      HardwareSerialRS485_h

#if !defined(ARDUINO_ARCH_AVR)
  // only AVR implementation is available.
  #error “The __FILE__ library only supports boards with an AVR.”
#endif


#if !defined(HardwareSerial_use_classical_version)
#if !defined(HardwareSerial_h)
    #define HardwareSerial_h /* block loading the classsical version of HardwareSerial */
#else
    #error "HardwareSerial.h already loaded. The file HardwareSerialRS485.h must be included before the #include ""Arduino.h"" (whether explicit or implicit)."
#endif

#include "utility/Messagefilter.h"
#include "utility/HardwareSerialRS485_Helper.h"
#include "utility/HardwareSerialRS485_Enabled.h"

// The definitions RS485configuration_HardwareSerialRS485_<n> for the HardwareSerial classes are provided by the file "HardwareSerialRS485_configuration.h"
// For a HardwareSerialRS485_<n> class to be created, both the RS485configuration_HardwareSerialRS485_<n> should be defined, and a corresponding (i.e. target hardware dependent) USART<n> class should exists.
#include "HardwareSerialRS485_configuration.h"


#if 1 /* define HardwareSerialRS485_0...3 and the corresponding objects Serial0...3 */
// nb: '#if 1' allows to collaps this code block in a decent editor (e.g. notepad++)

#if defined(UCSR0A)
    #if defined(RS485configuration_HardwareSerialRS485_0)
        typedef RS485configuration_HardwareSerialRS485_0 HardwareSerialRS485_0;
        extern HardwareSerialRS485_0 Serial0;
    #endif
#endif

#if defined(UCSR1A)
    #if defined(RS485configuration_HardwareSerialRS485_1)
        typedef RS485configuration_HardwareSerialRS485_1 HardwareSerialRS485_1;
        extern HardwareSerialRS485_1 Serial1;
    #endif
#endif

#if defined(UCSR2A)
    #if defined(RS485configuration_HardwareSerialRS485_2)
        typedef RS485configuration_HardwareSerialRS485_2 HardwareSerialRS485_2;
        extern HardwareSerialRS485_2 Serial2;
    #endif
#endif

#if defined(UCSR3A)
    #if defined(RS485configuration_HardwareSerialRS485_3)
        typedef RS485configuration_HardwareSerialRS485_3 HardwareSerialRS485_3;
        extern HardwareSerialRS485_3 Serial3;
    #endif
#endif

#if defined(USBCON)
//  We have a native USB interface, import the SerialUSBSwitch class as a service...
    #include "utility/SerialUSBSwitch.h"
    #include "USBAPI.h"
#else
//  for compatibility with the classical environment we define Serial
    #define Serial Serial0
#endif





#endif /* define HardwareSerialRS485_0...3 and the corresponding objects Serial0...3 */

#if defined(Implement_HardwareSerialRS485) // ++========================================================================================================
// link the hardware interrupt vectors to the interrupt methods of the HWSerial class
#if defined(UCSR0A)
    #if defined(RS485configuration_HardwareSerialRS485_0)
        HardwareSerialRS485_0 Serial0;
        template<> HardwareSerialRS485_0& HardwareSerialRS485_0::ourSerialObject = Serial0;
        link_USART_Vectors(USART0, Serial0);
        void serialEvent0() __attribute__((weak));
        void serialEvent0() {}
    #endif
#endif
#if defined(UCSR1A)
    #if defined(RS485configuration_HardwareSerialRS485_1)
        HardwareSerialRS485_1 Serial1;
        template<> HardwareSerialRS485_1& HardwareSerialRS485_1::ourSerialObject = Serial1;
        link_USART_Vectors(USART1, Serial1);
        void serialEvent1() __attribute__((weak));
        void serialEvent1() {}
    #endif
#endif
#if defined(UCSR2A)
    #if defined(RS485configuration_HardwareSerialRS485_2)
        HardwareSerialRS485_2 Serial2;
        template<> HardwareSerialRS485_2& HardwareSerialRS485_2::ourSerialObject = Serial2;
        link_USART_Vectors(USART2, Serial2);
        void serialEvent2() __attribute__((weak));
        void serialEvent2() {}
    #endif
#endif
#if defined(UCSR3A)
    #if defined(RS485configuration_HardwareSerialRS485_3)
        HardwareSerialRS485_3 Serial3;
        template<> HardwareSerialRS485_3& HardwareSerialRS485_3::ourSerialObject = Serial3;
        link_USART_Vectors(USART3, Serial3);
        void serialEvent3() __attribute__((weak));
        void serialEvent3() {}
    #endif
#endif

// build up serialEventRun
// IMHO serialEventRun is a useless function, I put it here for compatability with HardwareSerial
extern void serialEventRun(void) __attribute__((weak));
void serialEventRun(void)
{
#if defined(UCSR0A)
    #if defined(RS485configuration_HardwareSerialRS485_0)
        if (Serial0.available()) serialEvent0();
    #endif
#endif
#if defined(UCSR1A)
    #if defined(RS485configuration_HardwareSerialRS485_1)
        if (Serial1.available()) serialEvent1();
    #endif
#endif
#if defined(UCSR2A)
    #if defined(RS485configuration_HardwareSerialRS485_2)
        if (Serial2.available()) serialEvent2();
    #endif
#endif
#if defined(UCSR3A)
    #if defined(RS485configuration_HardwareSerialRS485_3)
        if (Serial3.available()) serialEvent3();
    #endif
#endif
}

#endif // defined(Implement_HardwareSerialRS485) ++ ====================================================================================================


#endif // !defined(HardwareSerial_use_classical_version)

#endif // !defined(HardwareSerialRS485_h)