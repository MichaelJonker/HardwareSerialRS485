/* USARTdef.h copyright notice

Defines classes with USART constants.

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

#if !defined(USARTdef_h)
#define      USARTdef_h

/* Description
   The USART definitions have grown in an somehow chaotic mess.
   This header file will put some order in it and defined all USART registers, constants and vectors in USART specific classes
   that can be used as template parameter for the HardwareSerial_RS485Enabled template class.
*/

#if 1 // normalize USART defines
// note: the #if 1 allows you to collapse the code to "normalize USART defines" in respectable code editors.
// This section ensures that missing definitions for USART0 and USART1 classes are compensated
// Note: this part has not been tested for configurations that are 20141212 not part of the Arduino configurations.

#if !defined(UCSR0A) and defined(UCSRA)
    #define  UCSR0A              UCSRA
    #define  UCSR0B              UCSRB
    #define  UCSR0C              UCSRC
    #define  UBRR0H              UBRRH
    #define  UDR0                UDR
#endif

#if !defined(RXEN0)  and defined(RXEN)
    #define  MPCM0               MPCM
    #define  U2X0                U2X
    #define  UPE0                UPE
    #define  DOR0                DOR
    #define  FE0                 FE
    #define  TXC0                TXC
    #define  TXEN0               TXEN
    #define  RXEN0               RXEN
    #define  UDRIE0              UDRIE
    #define  TXCIE0              TXCIE
    #define  RXCIE0              RXCIE
#endif

/* normalize USART0_UDRE_vect */
#if !defined(USART0_UDRE_vect) and defined(USART_UDRE_vect)
    #define  USART0_UDRE_vect              USART_UDRE_vect
#endif

/* normalize USART0_RX_vect and USART0_tX_vect*/
#if !defined(USART0_RX_vect)
                             #if   defined(USART_RX_vect)
    #define  USART0_RX_vect                USART_RX_vect
    #define  USART0_TX_vect                USART_TX_vect
                             #elif defined(USART0_RXC_vect)
    #define  USART0_RX_vect                USART0_RXC_vect
    #define  USART0_TX_vect                USART0_TXC_vect
                             #elif defined(USART_RXC_vect)
    #define  USART0_RX_vect                USART_RXC_vect
    #define  USART0_TX_vect                USART_TXC_vect
                             #endif
#endif

/* normalize USART1_RX_vect, USART1_TX_vect (iom162.h) */
#if !defined(USART1_RX_vect) and defined(USART1_RXC_vect)
    #define  USART1_RX_vect              USART1_RXC_vect
    #define  USART1_TX_vect              USART1_TXC_vect
#endif

#endif // normalize USART defines

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
Conclusion, as we use TXC interrupts, we maintain the TX_active status in the software register: static unsigned char swReg.
*/

#if 1 // define USART_constants(__N)
// note: the #if 1 allows you to collapse these USART_constants code in respectable code editors

#if defined(__AVR_ATmega8__) // define special bit for setting configuration
  #define __AVR_ATmega8_config__ 0x80 /* select UCSRC register (shared with UBRRH) -- that is how it was commented in HardwareSerial.cpp ... -- */
#else
  #define __AVR_ATmega8_config__ 0
#endif

#define USART_constants(__N) \
  static inline volatile unsigned char& ucsra()  {return UCSR##__N##A; }; \
  const static unsigned char mpcp  = MPCM##__N;  /* bit n */  \
  const static unsigned char u2x   = U2X##__N;   /* bit n */  \
  const static unsigned char upe   = UPE##__N;   /* bit n */  \
  const static unsigned char dor   = DOR##__N;   /* bit n */  \
  const static unsigned char fe    = FE##__N;    /* bit n */  \
  const static unsigned char txc   = TXC##__N;   /* bit n */  \
  static inline volatile unsigned char& ucsrb()  {return UCSR##__N##B; }; \
  const static unsigned char txen  = TXEN##__N;  /* bit 3 */  \
  const static unsigned char rxen  = RXEN##__N;  /* bit 4 */  \
  const static unsigned char udrie = UDRIE##__N; /* bit 5 */  \
  const static unsigned char txcie = TXCIE##__N; /* bit 6 */  \
  const static unsigned char rxcie = RXCIE##__N; /* bit 7 */  \
  static inline volatile unsigned char& ucsrc()  {return UCSR##__N##C; }; \
  static inline volatile unsigned char& ubbrl()  {return UBRR##__N##L; }; \
  static inline volatile unsigned char& ubbrh()  {return UBRR##__N##H; }; \
  static inline volatile unsigned char& udr()    {return UDR##__N; }; \
  \
  static unsigned char swReg; /* see note on TXC above */ \
  const static unsigned char swReg_BUSY = 0;  /* bit 3 */  \
//end define USART_constants(__N)

#endif // define USART_constants(__N)

/* define known USARTn (after consistency checks)
   check presence of
   USART0_UDRE_vect -- USART User Data Regsiter Empty
   USART0_TX_vect   -- USART Tx Complete
   USART0_RX_vect   -- USART Rx Complete
*/
#if defined(UCSR0A)
  #if   !defined(USART0_UDRE_vect)
      #error "USART0_UDRE_vect not defined"
  #elif !defined(USART0_TX_vect)
      #error "USART0_TX_vect not defined"
  #elif !defined(USART0_RX_vect)
      #error "USART0_RX_vect not defined"
  #endif

  class USART0 { protected: USART_constants(0);};
#endif
#if defined(UCSR1A)
  #if   !defined(USART1_UDRE_vect)
      #error "USART1_UDRE_vect not defined"
  #elif !defined(USART1_TX_vect)
      #error "USART1_TX_vect not defined"
  #elif !defined(USART1_RX_vect)
      #error "USART1_RX_vect not defined"
  #endif

  class USART1 { protected: USART_constants(1);};
#endif
#if defined(UCSR2A)
  #if   !defined(USART2_UDRE_vect)
      #error "USART2_UDRE_vect not defined"
  #elif !defined(USART2_TX_vect)
      #error "USART2_TX_vect not defined"
  #elif !defined(USART2_RX_vect)
      #error "USART2_RX_vect not defined"
  #endif

  class USART2 { protected: USART_constants(2);};

#endif
#if defined(UCSR3A)
  #if   !defined(USART3_UDRE_vect)
      #error "USART3_UDRE_vect not defined"
  #elif !defined(USART3_TX_vect)
      #error "USART3_TX_vect not defined"
  #elif !defined(USART3_RX_vect)
      #error "USART3_RX_vect not defined"
  #endif

  class USART3 { protected: USART_constants(3);};

#endif
/* consistency checks for all USARTn */

// end USART class // =============================================================================

// macro to define linking of interrupts
#define link_USART_Vectors(T_USART, T_SERIAL) \
ISR(T_USART##_UDRE_vect) { T_SERIAL.dreInt(); } \
ISR(T_USART##_TX_vect)   { T_SERIAL.txcInt(); } \
ISR(T_USART##_RX_vect)   { T_SERIAL.rxcInt(); } \
void serialEvent() __attribute__((weak)); \
void serialEvent() {} \
unsigned char T_USART::swReg;

/* todo: any use for these thoughts? or is it better to define our friends elsewhere.
//To resolve friendship ...
//HardwareSerial needs to be friends with the interupt vectors. This is difficult to achieve as in hardware Serial we do not know where does vectors are.
//For the USART class it is easier to become friends as the information of the vectors can be made known to the class (extention of the USART_CONSTANTS() macro.
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
// For the HardwareSerial class it is east to make the USART interrupt hanlders their friend, as the USART class is a template parameter of the HardwareSerial class
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