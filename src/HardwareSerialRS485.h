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

#if !defined(HardwareSerial_h)  // if HardwareSerial was not disabled yet...
#define HardwareSerial_h        // make sure that it gets disabled
#endif

#if defined(Serial_8N1) // we use a different 'marker' to detect the presence of HardwareSerial.h
    #error "HardwareSerial.h already loaded. You have not selected an RS485 specific board, or your boards.txt file is incomplete (see documentation https://github.com/MichaelJonker/HardwareSerialRS485/wiki/installation-and-deployment)."
#endif

#include "utility/Messagefilter.h"
#include "utility/HardwareSerialRS485_Helper.h"
#include "utility/HardwareSerialRS485_Enabled.h"

// The definitions RS485configuration_HardwareSerialRS485_<n> for the HardwareSerial classes are provided by the file "HardwareSerialRS485_configuration.h"
// For a HardwareSerialRS485_<n> class to be created, both the RS485configuration_HardwareSerialRS485_<n> should be defined, and a corresponding (i.e. target hardware dependent) USART<n> class should exists.
#include "utility/HardwareSerialRS485_configuration.h"


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
    #endif
#endif
#if defined(UCSR1A)
    #if defined(RS485configuration_HardwareSerialRS485_1)
        HardwareSerialRS485_1 Serial1;
        template<> HardwareSerialRS485_1& HardwareSerialRS485_1::ourSerialObject = Serial1;
        link_USART_Vectors(USART1, Serial1);
    #endif
#endif
#if defined(UCSR2A)
    #if defined(RS485configuration_HardwareSerialRS485_2)
        HardwareSerialRS485_2 Serial2;
        template<> HardwareSerialRS485_2& HardwareSerialRS485_2::ourSerialObject = Serial2;
        link_USART_Vectors(USART2, Serial2);
    #endif
#endif
#if defined(UCSR3A)
    #if defined(RS485configuration_HardwareSerialRS485_3)
        HardwareSerialRS485_3 Serial3;
        template<> HardwareSerialRS485_3& HardwareSerialRS485_3::ourSerialObject = Serial3;
        link_USART_Vectors(USART3, Serial3);
    #endif
#endif

// build up serialEventRun
// IMHO serialEventRun is a useless function, I put it here for compatability with HardwareSerial
//todo extern void serialEventRun(void) __attribute__((weak));
void serialEventRun(void)
{
#if defined(UCSR0A)
    #if defined(RS485configuration_HardwareSerialRS485_0)
        if (Serial0.available()) Serial0_Event();
    #endif
#endif
#if defined(UCSR1A)
    #if defined(RS485configuration_HardwareSerialRS485_1)
        if (Serial1.available()) Serial1_Event();
    #endif
#endif
#if defined(UCSR2A)
    #if defined(RS485configuration_HardwareSerialRS485_2)
        if (Serial2.available()) Serial2_Event();
    #endif
#endif
#if defined(UCSR3A)
    #if defined(RS485configuration_HardwareSerialRS485_3)
        if (Serial3.available()) Serial3_Event();
    #endif
#endif
}

#endif // defined(Implement_HardwareSerialRS485) ++ ====================================================================================================


#endif // !defined(HardwareSerialRS485_h)