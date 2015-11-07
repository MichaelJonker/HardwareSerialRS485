/* HardwareSerialRS485_configuration.h copyright notice

Defines the configuration of the HardwareSerialRS485 classes and objects.

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

#if !defined(HardwareSerialRS485_configuration_h)
#define      HardwareSerialRS485_configuration_h


/* define HardwareSerialRS485 classes for all possible USARTs. */

/* Important, this file may exists in more than one location:
   A functional default/template/example file is located in folder libraries/HardwareSerialRS485/src
   A copied and adapted file may be located in a sub folder of the hardware folder in the user project directory.

   To tailor the HardwareSerialRS485 configuration, the user is advised to copy the two files:
     - libraries/HardwareSerialRS485/src/HardwareSerialRS485_configuration.h  and
     - libraries/HardwareSerialRS485/src/boards.txt
   to a folder hardware/HardwareSerialRS485/avr (which is to be created in the user project folder, i.e. the folder
   where all sketches and user libraries are located).
   These two file can then be further tailored to the need of the user. For further explanations see the comments in this file and in boards.txt file.
 */



// These definitions specify the configuration of the HardwareSerialRS485 classes and may (should) be tailored by the user as needed.
// Only those definitions of RS485configuration_HardwareSerialRS485_<n> will be used for which a corresponding (i.e. target hardware dependent) UCSR<n>A register is defined.

// The configuration of the HardwareSerialRS485 classes is controlled by a number of preprocessor macros, all starting with the prefix RS485configuration_.
// These preprocessor macros can be adapted in this header file or (preferred) in the boards.txt file. In case no definition is given by the boards.txt file,
// then default values are taken from this file.


#if !defined(RS485configuration_TRxBufferParameters)
//  defines, in three comma separated fields, the Tx (transmit) and Rx (receive) buffer sizes and the Rx-buffer overrun indicator.
//  - the buffer sizes are specified in powers of 2, i.e. a value 6 specifies a buffersize of (1<<6) or 64 bytes;
//  - the buffer overrun indicator specifies the character to be entered in the Rx buffer when the buffer is full and incoming characters will have to be dropped.
//    a space character indicates no overrun indicator.
    #define RS485configuration_TRxBufferParameters 6, 6, ' '    // default definition:  Buffer size (Tx & Rx): 64, no Rx-buffer overrun signaling
//  #define RS485configuration_TRxBufferParameters 5, 7, '$'    // alternative example: Buffer size: 32 (Tx) and 128 (Rx), Rx overrun marked by a '$' sign.
#endif

#if !defined(RS485configuration_TRxControl)
//  defines the class that implements the control of the RS485 enable lines.
//  This is likely the only definition that should be adapted by the user to reflect how the RS485 enable lines are connected in the hardware implementation.
//  Unless in special cases where a user defined class is needed, the user should use the TRxControl template class.
//  This template class is specialized using three parameters that give the port (by character), the TxE port number and the RxE* port number.
//  Example: if the hardware connects output pin B2 to the RxE input and output pin B3 to the RxE* input, the three template parameters 'B', 2, 3 should be given.
//  Note that for efficiency this class only handles hardware configurations where the two enable pins of the RS485 transceiver are controled from the same port.
//  In case this restriction can not not match you hardware reality, you have to addapt the TRxControl class.
//  The following definition is given as an example (and serves as default).
    #define RS485configuration_TRxControl TRxControl< 'B', 2, 3 >
    #if defined(Implement_HardwareSerialRS485)      // only print the warning once
    #if !defined(RS485configuration_RS485helper)    // and not if the user defined his own RS485helper
    #warning "No definition of RS485configuration_TRxControl was given, a possibly defunct default is now used."
    #endif
    #endif
#endif

#if !defined(RS485configuration_RS485helper)
//  defines the helper class to provide RS485 capabilities. Defining RS485configuration_RS485helper without a value, creates a class without RS485 capabilities.
    #define RS485configuration_RS485helper RS485serial< RS485configuration_TRxControl >
//  #define RS485configuration_RS485helper  // alternative: a definition without value removes RS485 capabilities from the HardwareSerialRS485 class
//  #define RS485configuration_RS485helper RS485serial< RS485configuration_TRxControl, Transaction_dummy > // alternative: RS485 with transaction capabilities disabled.
#endif

#if !defined(RS485configuration_MessageFilterParameters)
//  defines the messageFilter parameter class used by the message filter.
//  note this defintion will also be used by the message reader.
    #define RS485configuration_MessageFilterParameters MFP< '{', '}' >      // default value:       message filtering based on '{' and '}' message delimiting characters
//  #define RS485configuration_MessageFilterParameters MFP< 0x01, 0x04 >    // alternative example: message filtering based on control characters SOH, Start of header (^A) and EOT end of transmission (^D)
#endif


#if !defined(RS485configuration_MessageFilter)
//  defines the message filter handling class. The default defintion is a template expansion of the MessageFilter class based on the MessageFilterParameters
    #define RS485configuration_MessageFilter MessageFilter< RS485configuration_MessageFilterParameters >  // message filtering based on predefined messageFilter parameters
//  #define RS485configuration_MessageFilter                                                              // alternative: no filtering 
#endif

#if !defined(RS485configuration_HardwareSerialRS485_0)
//  defines the HardwareSerialRS485 class associated with USART0. The definitions is based on other macro definitions described above.
    #define RS485configuration_HardwareSerialRS485_0 HardwareSerialRS485< USART0, RS485configuration_TRxBufferParameters, RS485configuration_RS485helper, RS485configuration_MessageFilter >
//  #define RS485configuration_HardwareSerialRS485_0 HardwareSerialRS485< USART0 >                          // alternative: define HardwareSerialRS485_0 with all extra capabilities stripped 
#endif

#if !defined(RS485configuration_HardwareSerialRS485_1)
//  defines the HardwareSerialRS485 class associated with USART0. The definitions is based on other macro definitions described above.
    #define RS485configuration_HardwareSerialRS485_1 HardwareSerialRS485< USART1, RS485configuration_TRxBufferParameters, RS485configuration_RS485helper, RS485configuration_MessageFilter >
#endif

#if !defined(RS485configuration_HardwareSerialRS485_2)
// No default is defined for RS485configuration_HardwareSerialRS485_2, i.e. the class will not be created.
// If needed, this defintion can be added to the boards.txt file or you may tailor your private copy of this file.
#endif

#if !defined(RS485configuration_HardwareSerialRS485_3)
// No default is defined for RS485configuration_HardwareSerialRS485_3, i.e. the class will not be created.
// If needed, this defintion can be added to the boards.txt file or you may tailor your private copy of this file.
#endif



#endif // !defined(HardwareSerialRS485_configuration_h)