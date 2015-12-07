/* USARTdef.h copyright notice

Defines classes with USART constants.

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

#if !defined(USARTdef_h)
#define      USARTdef_h

/* Description
   The USART definitions have grown in an somehow chaotic mess.
   This header file will put some order in it and defined all USART registers, constants and vectors in USART specific classes
   that can be used as template parameter for the HardwareSerialRS485_Enabled template class.
*/

/* USART class // =================================================================================
// All USART registers and parameters are defined in a USART specific class.
//
// The UCSRA register has 3 writeable bits:
// - TXCn:  this bit gets cleared by writing a one to it.
// - U2Xn:  this doubles the baud rate at the cost of halving the bit sampling.
// - MPCMn: Multi-processor Communication Mode
// Conclusions 1: As we do not use MPCM, we can just write the U2X bit-value, i.e. it is safe to write zero to TXCn and MPCMn
// Conclusions 2: If we want to clear TXCn we can write the register to itself

Note on TXC.
The TXC bit is set when the TX shift register is emptied and no data is available in the data register (DR) to be loaded next.
TXC is automatically cleared when the interrupt is executed. In case the TXC interrupts are disabled, it can be cleared explicitly
(by writing 1 in the TXC bit).
Implications:
 - In case we do not use the TXC interrupts, the TXC bit can be exploited as an !TX_active status bit (i.e. explicitly clear the bit at start
   of each transfer).
 - In case we use the TXC interrupts, the TXC bit becomes not exploitable as an !TX_active status bit, as it is cleared upon calling the interrupt.
   TXC is even not exploitable in combination with the UDRE (as e.g. UDRE may show glitches if the buffer is not filled fast enough).
Conclusion, as we use TXC interrupts, we maintain the TX_active status in the software register: static volatile unsigned char swReg.
*/

#if 1 // define USART_constants(__NREG__, __NBIT__)
// note: the #if 1 allows you to collapse these USART_constants code in respectable code editors

#if defined(__AVR_ATmega8__) // define special bit for setting configuration
  #define __AVR_ATmega8_config__ 0x80 /* select UCSRC register (shared with UBRRH) -- that is how it was commented in HardwareSerial.cpp ... -- */
#else
  #define __AVR_ATmega8_config__ 0
#endif

#define USART_constants(__NREG__, __NBIT__) \
  static inline volatile unsigned char& ucsra()  {return UCSR##__NREG__##A; }; \
  const static unsigned char testreg   = MPCM##__NREG__;     /* bit 0 */  \
  const static unsigned char mpcp   = MPCM##__NBIT__;     /* bit 0 */  \
  const static unsigned char u2x    = U2X##__NBIT__;      /* bit 1 */  \
  const static unsigned char upe    = UPE##__NBIT__;      /* bit 2 */  \
  const static unsigned char dor    = DOR##__NBIT__;      /* bit 3 */  \
  const static unsigned char fe     = FE##__NBIT__;       /* bit 4 */  \
  const static unsigned char udre   = UDRE##__NBIT__;     /* bit 5 ~ */  \
  const static unsigned char txc    = TXC##__NBIT__;      /* bit 6 */  \
  const static unsigned char rxc    = RXC##__NBIT__;      /* bit 7 ~ */  \
  static inline volatile unsigned char& ucsrb()  {return UCSR##__NREG__##B; }; \
  const static unsigned char txb8   = TXB8##__NBIT__;     /* bit 0 ~ */  \
  const static unsigned char rxb8   = RXB8##__NBIT__;     /* bit 1 ~ */  \
  const static unsigned char ucsz2  = UCSZ##__NBIT__##2;  /* bit 2 */  \
  const static unsigned char txen   = TXEN##__NBIT__;     /* bit 3 */  \
  const static unsigned char rxen   = RXEN##__NBIT__;     /* bit 4 */  \
  const static unsigned char udrie  = UDRIE##__NBIT__;    /* bit 5 */  \
  const static unsigned char txcie  = TXCIE##__NBIT__;    /* bit 6 */  \
  const static unsigned char rxcie  = RXCIE##__NBIT__;    /* bit 7 */  \
  static inline volatile unsigned char& ucsrc()  {return UCSR##__NREG__##C; }; \
  const static unsigned char ucpol  = UCPOL##__NBIT__;    /* bit 0 ~ */  \
  const static unsigned char ucsz0  = UCSZ##__NBIT__##0;  /* bit 1 */  \
  const static unsigned char ucsz1  = UCSZ##__NBIT__##1;  /* bit 2 */  \
  const static unsigned char usbs   = USBS##__NBIT__;     /* bit 3 */  \
  const static unsigned char upm0   = UPM##__NBIT__##0;   /* bit 4 */  \
  const static unsigned char upm1   = UPM##__NBIT__##1;   /* bit 5 */  \
  const static unsigned char umsel0 = UMSEL##__NBIT__##0; /* bit 6 */  \
  const static unsigned char umsel1 = UMSEL##__NBIT__##1; /* bit 7 */  \
  static inline volatile unsigned char& ubrrl()  {return UBRR##__NREG__##L; }; \
  static inline volatile unsigned char& ubrrh()  {return UBRR##__NREG__##H; }; \
  static inline volatile unsigned char& udr()    {return UDR##__NREG__; }; \
  \
  static volatile unsigned char swReg; /* see note on TXC above */ \
  const static unsigned char swReg_BUSY = 0;  /* bit 3 */  \
//end define USART_constants(__NREG__, __NBIT__)

#endif // define USART_constants(__NREG__, __NBIT__)

/* define known USARTn
   check presence of
   USART0_UDRE_vect -- USART User Data Regsiter Empty
   USART0_TX_vect   -- USART Tx Complete
   USART0_RX_vect   -- USART Rx Complete
*/
#if defined(UCSR0A) or defined(UCSRA)
  #if   defined(USART0_UDRE_vect)     /* normalize & test USART0_UDRE_vect */
  #elif defined(USART_UDRE_vect)
       #define  USART0_UDRE_vect              USART_UDRE_vect
  #else
      #error "USART0_UDRE_vect not defined"
  #endif
  #if   defined(USART0_RX_vect)       /* normalize & test USART0_RX_vect */
  #elif defined(USART_RX_vect)
       #define  USART0_RX_vect                USART_RX_vect
  #else
      #error "USART0_RX_vect not defined"
  #endif
  #if   defined(USART0_TX_vect)       /* normalize & test USART0_TX_vect */
  #elif defined(USART_TX_vect)
       #define  USART0_TX_vect                USART_TX_vect
  #else
      #error "USART0_TX_vect not defined"
  #endif


  class USART0 { protected: USART_constants(
  #if defined(UCSR0A)
    0
  #endif
  ,
  #if defined(RXEN0)
    0
  #endif
  );};
#endif
#if defined(UCSR1A)
  #if    defined(USART1_UDRE_vect)
  #else
      #error "USART1_UDRE_vect not defined"
  #endif
  #if    defined(USART1_RX_vect)
  #elif  defined(USART1_RXC_vect)   /* iom162.h */
        #define USART1_RX_vect USART1_RXC_vect
  #else
      #error "USART1_RX_vect not defined"
  #endif
  #if    defined(USART1_TX_vect)
  #elif  defined(USART1_TXC_vect)   /* iom162.h */
        #define USART1_TX_vect USART1_TXC_vect
  #else
      #error "USART1_TX_vect not defined"
  #endif

  class USART1 { protected: USART_constants(1, 1);};
#endif
#if defined(UCSR2A)
  #if    defined(USART2_UDRE_vect)
  #else
      #error "USART2_UDRE_vect not defined"
  #endif
  #if    defined(USART2_RX_vect)
  #else
      #error "USART2_RX_vect not defined"
  #endif
  #if    defined(USART2_TX_vect)
  #else
      #error "USART2_TX_vect not defined"
  #endif

  class USART2 { protected: USART_constants(2, 2);};

#endif
#if defined(UCSR3A)
  #if    defined(USART3_UDRE_vect)
  #else
      #error "USART3_UDRE_vect not defined"
  #endif
  #if    defined(USART3_RX_vect)
  #else
      #error "USART3_RX_vect not defined"
  #endif
  #if    defined(USART3_TX_vect)
  #else
      #error "USART3_TX_vect not defined"
  #endif

  class USART3 { protected: USART_constants(3, 3);};

#endif

// end USART class // =============================================================================

// macro to define linking of interrupts
#define link_USART_Vectors(T_USART, T_SERIAL) \
ISR(T_USART##_UDRE_vect) { T_SERIAL.dreInt(); } \
ISR(T_USART##_TX_vect)   { T_SERIAL.txcInt(); } \
ISR(T_USART##_RX_vect)   { T_SERIAL.rxcInt(); } \
void T_SERIAL##_Event() __attribute__((weak)); \
void T_SERIAL##_Event() {} \
volatile unsigned char T_USART::swReg;

/* todo: any use for these thoughts? or is it better to define our friends elsewhere.
//To resolve friendship ...
//HardwareSerial needs to be friends with the interrupt vectors. This is difficult to achieve as in hardware Serial we do not know where those vectors are.
//For the USART class it is easier to become friends as the information of the vectors can be made known to the class (through an extension of the USART_CONSTANTS() macro.
// we get:

class USART
{
  USART_CONSTANTS();

  // we declare the interrupt handlers...
  void rxcInt();
  void txcInt();
  void dreInt();

  // and add the interrupt vector our friends
  friend void USART_RX_vect();
  friend void USART_TX_vect();
  friend void USART_UDRE_vect();
};

// these are the interrupt vectors, which may call the USART interrupt handlers. question could their declaration be made from within the class... as ISR(xx) specifies an absolute address
ISR(USART_RX_vect)   { USART::rxcInt(); }
ISR(USART_TX_vect)   { USART::txcInt(); }
ISR(USART_UDRE_vect) { USART::dreInt(); }

extern "c" void USART_RX_vect(void) --Attribute__ ((signal, used, externally_visible))
extern "c" void __vector_21(void) --Attribute__ ((signal, used, externally_visible))

// Now the interrupt handler of the USART class must be linked to the HardwareSerial object.
// For the HardwareSerial class it is east to make the USART interrupt handlers their friend, as the USART class is a template parameter of the HardwareSerial class
class HardwareSerial
{
  friend void T__USART::rxcInt();
  friend void T__USART::txcInt();
  friend void T__USART::dreInt();
}


// part of the linking (macro?)
void USART::rxcInt() { Serial.rxcInt(); }
void USART::txcInt() { Serial.txcInt(); }
void USART::dreInt() { Serial.dreInt(); }

alternative, the USART class may have a (static) member pointing to an Interrupt interface. (Can I generalize the interrupt interface? to simplify the above)

Other idea:
Try to make rxcInt(), txcInt(), dreInt() static member of HardwareSerial. They call only members of T__RS485 and T__USART
T__RS485 should be declared as a static member (and hence instantiated), instead of being sub-classed.

Price to pay of HardwareSerial not subclassing T__RS485is that members of T__RS485 are now hidden from external users. The most important
are the TRxControl members. Possible solution is to provide a method to expose the TRxControl object, or better, the T__RS485 object (as this is passed as template parameter)
Note: Originally T__RS485 was non static because of virtual methods to be implemented by HardwareSerial.

//cat $Arduino/arduino-1.0.5/hardware/tools/avr/avr/include/avr/interrupt.h

#define ISR(vector, ...)            \
    extern "C" void vector (void) __attribute__ ((signal, used, externally_visible)) __VA_ARGS__; \
    void vector (void)

ISR(vector)
{ code }
gives:

extern "c" void vector(void) --Attribute__ ((signal, used, externally_visible))
void vector (void)
{ code }


*/

#endif // !defined(USARTdef_h)