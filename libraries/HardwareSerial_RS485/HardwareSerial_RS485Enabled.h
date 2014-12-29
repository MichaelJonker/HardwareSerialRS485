/* HardwareSerial_RS485Enabled.h copyright notice

RS485 enabled implementation of HardwareSerial (replacing the classical version for Wiring by Nicholas Zambetti, Mark Sproul, Alarus)

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

#if !defined(HardwareSerial_RS485Enabled_h)
#define      HardwareSerial_RS485Enabled_h

/* Description
The HardwareSerial_RS485Enabled.h provides an RS485 enabled version of the HardwareSerial class. By default the code implements a dummy RS485 handler
and provides exactly the same functionality as the classical version (However, it uses less code an runs faster (to be quantified)
To effectively use this class with an external RS232 to RS485 driver receiver circuit, the helper class RS485serial (included from HardwareSerial_RS485helper.h)
must be passed as a template parameter. See also HardwareSerial_RS485.h
*/

/* TODO: improvements for HardwareSerial_RS485Enabled:
   make the input idle time-out depend on baud and priority, call entry in the helper class to set idle time.
   reconsider justification using HWSerial as a class and not instance (one class per USART) (i.e. USART is also static for efficiency (code and space), so there is one class per USART, making RingBuffers static further improves efficiency).

// auto start on number of pending characters in the buffer or on specified character.
// auto flush on specified character (to force a transmission space i.e. drops the line)
// option: add message time out (MTO) character to the buffer when the line is idle
// add function to register an error callback
 */

#include "arduino.h"
#include <inttypes.h>
#include "BitManipulation.h"
#include "USARTdef.h"

#include "Stream.h"
#include "Print.h"

#define diagnoseDetails /* if defined, it causes detailed debug version to be written when communication gets stuck */

// See $Arduino/arduino-1.0.5/hardware/arduino/cores/arduino/wiring.c
extern volatile unsigned long timer0_overflow_count; // ticks every 64*256 cycles = 16 x 1024 / F_CPU, i.e. 1.024 ms for F_CPU=16MHz

// HWKICK() can be defined externally to emulate HW for development and debugging (HWKICK() is called from enableDREI).
extern void HWKICK() __attribute__((weak)); inline void HWKICK() { /* niks */ }  // If not defined externally this weak dummy is used.

class RS485_dummy // a dummy 'yes boss' version of the RS485 protocol that does nothing
{
// this class is provided here to provide efficient non-RS485 enabled HardwareSerial functionality...
protected:

  static inline void begin()              { }               //called from the begin method
  static inline void reset()              { }               //set txState back to 
  static inline void enableStart()        { }               //not called currently
  static inline void setMinimumIdleTime() { }
  static inline bool txReady()            { return true;  } //called from the write method, returns true if the DRE interrupt should to be enabled.
  static inline bool txError()            { return false; } //called from flush, returns true if tranfer has been interrupted due to an error.
  static inline bool txInputIdleWait()    { return false; } //called from the write method, returns true if we did wait for input idle and hence DRE should be enabled
  static inline bool txStart()            { return true;  } //called from DRE interrupt to set 485 transmission enable. gives go/no-go for transmission and enables the RS485 circuitry if needed
  static inline void txCompleted()        { }               //called from TX completed interrupt when all data has gone out, clears 485 enable and updates the state
  static inline void txSendChar(unsigned char c)         {} //called from DRE interrupt to pass next character to send;
  static inline bool isEcho(unsigned char c, bool error) { return false; } //called from RXC interrupt. Returns true when the character was expected from local echo.
  static inline void diagnoseHelper(Print& out)          {}                //called from diagnose

  unsigned long volatile lastRxTime;  // last time of data reception
  public:
  
  //static inline void setMode(unsigned char mode)         {}                //called externally //TODO encapsulate by a relay function
}; // class RS485_dummy =======================================================================================================================

class MessageFilter_dummy // a passAll filter class that can be used in filters are not needed, providing appreciable code reduction and optimisation 
{
  public:
  inline MessageFilter_dummy()                          {}
  inline void terminateMessage()                        {}
  inline void setAdressList(char** anAddressList)       {}
  char        check(char c, bool& bumpMessagePointer)   { return 0; }

};

/* RingBuffer // ==================================================================================
//
// We use a template class to define a ring buffer of a fixed length
// parameters:
// T__BUFFER_SIZE_EXPONENT  exponent of the buffer size. The buffer size is given by (1<<BUFFER_SIZE_EXPONENT)
//                          note, the buffer size is forced to be a power of 2, to make wrapping the buffer a simple masking operation.
//                          defining a buffer size different from a power of 2 would break the code.
// T__runOverOldData        defines the behaviour on dealing with buffer overrun, if true old data is overwritten, if false, new data is discarded.
// T__overRunMarker         defines the overrunMarker (nb if absent or set to ' ', the overRun will not be marked).
 */
template <unsigned char T__BUFFER_SIZE_EXPONENT, bool T__runOverOldData=false, unsigned char T__overRunMarker=' '>
class RingBuffer
{
public:
  static const unsigned char BUFFER_SIZE = (1<<T__BUFFER_SIZE_EXPONENT);
  static const unsigned char SIZE_MASK   = BUFFER_SIZE-1;
  static const bool          markOverRun = (T__overRunMarker!=' ');

  volatile unsigned char head;  // pointer to the next character to be written in the buffer. //TODO volatile impacts optimizations, try to avoid its usage
  volatile unsigned char tail;  // pointer to the next character to be read  from the buffer. //TODO volatile impacts optimizations, try to avoid its usage

  unsigned char buffer[BUFFER_SIZE];

public:
  inline void reset()                       { head=tail=0;}

  inline unsigned char itemsStored()        { return (unsigned char)(head-tail); }
  inline unsigned char availableSpace()     { return (unsigned char)(BUFFER_SIZE+tail-head); }

  inline bool isFull()                      { return itemsStored() == BUFFER_SIZE; }
  inline bool isEmpty()                     { return head==tail; }

  // the following functions do not check for buffer full or empty conditions. Used directly by the transmission handling, which checks explicitly for these conditions. 
  inline unsigned char getChar()                                 { return buffer[ (tail++) & SIZE_MASK]; }
  inline unsigned char seeChar()                                 { return buffer[ (tail  ) & SIZE_MASK]; }
  inline          void putChar(unsigned char c)                  { buffer[ (head++) &SIZE_MASK] = c; }
  inline          void pokeAt(unsigned char c, unsigned char d)  { buffer[ (head+d) &SIZE_MASK] = c; }
  inline          void bumpTo(unsigned char d)                   { head += d+1; }
  inline unsigned char getBufferPosition()                       { return tail;   }
  inline          void setBufferPosition(unsigned char position) { tail=position; }

  // the next functions checks for buffer full or empty conditions and respond accordingly. Use by reception handling.
  inline void storeChar(unsigned char c)
  {
    if(isFull())
    {
      if(T__runOverOldData)
      {
        tail++; // make space by removing old data
        if(markOverRun) buffer[ tail &SIZE_MASK] = T__overRunMarker;
      }
      else
      {
        if(markOverRun) buffer[ head &SIZE_MASK] = T__overRunMarker;
        return; // we ignore the incoming data
      }
    }
    putChar(c); // store incoming data
  }

  inline bool pokeRelative(unsigned char c, unsigned char offset)
  {
    if( availableSpace() < offset ) // it won't fit
    {
      if(T__runOverOldData)
      {
        tail = tail+offset+1; // make space by removing old data
        if(markOverRun) buffer[ tail &SIZE_MASK] = T__overRunMarker;
      }
      else
      {
        return false; // we'll have to ignore the incoming message
      }
    }
    // poke the character
    pokeAt(c, offset);
    return true;
  }

  inline int fetchChar()
  {
    if(isEmpty()  ) return -1;  //note -1 != 0xFF, no need to define a special character for this

    if( !T__runOverOldData) return getChar();

    // T__runOverOldData: disable interrupts to avoid that data can be overwritten without the reader knowing it
    unsigned char temp_SREG = SREG;
    cli();
    unsigned char result = getChar();
    SREG = temp_SREG; // restore interrupts
    return result;
  }

  inline int peekChar()
  {
    // protect against the condition where data is overwritten without the reader knowing it
    if(isEmpty()  ) return -1;  //note -1 != 0xFF, no need to define a special character for this

    if( !T__runOverOldData) return seeChar();

    // T__runOverOldData active, protect against the condition where data would be overwritten without the reader knowing it
    unsigned char temp_SREG = SREG;
    cli();
    unsigned char result = seeChar();
    SREG = temp_SREG; // restore interrupts
    return result;
  }
}; // template class RingBuffer =====================================================================================================================


/* HardwareSerial class // ========================================================================
// We use a template class to define the HardwareSerial class:
// parameters:
// USART               the USART class with the USART register definitions
// T__TX_BufferSize    the size (exponent) of the transmit buffer
// T__RX_BufferSize    the size (exponent) of the receive buffer
// T__RX_OverRunMarker the data to mark RX buffer overruns. If equals ' ' (default) then there is no overrun marking
// T__RS485            plug-in class to handle the RS485 protocol (if omitted, dummy RS485 protocol included).
// T__MessageFilter    plug-in class to handle message filtering  (if omitted, dummy filter included).
*/
template <class T__USART, int T__TX_BufferSize=6, int T__RX_BufferSize=6, unsigned char T__RX_OverRunMarker=' ', class T__RS485=RS485_dummy, class T__MessageFilter=MessageFilter_dummy>
class HardwareSerial : T__USART, public Stream, public T__RS485
{
private:
  static RingBuffer<T__RX_BufferSize, false, T__RX_OverRunMarker> rxBuffer;
  static RingBuffer<T__TX_BufferSize> txBuffer;
  static T__MessageFilter myMessageFilter;
  static unsigned char ourPriority;

  static const unsigned int timeoutLimit = 16; // time out limit in number of baud
  static const unsigned int baudTime = 100; //time to send one baud in usec // TODO make dynamic (in USART class?) as function of baudRate
  static const unsigned int dataSize =  10; //number of bits per data item including start and stop. used for time-out calculation
  

  static const unsigned char ucsrb_enable  = (unsigned char) ((1<<T__USART::rxen) |(1<<T__USART::txen) |(1<<T__USART::rxcie) |(1<<T__USART::txcie));
  static const unsigned char ucsrb_disable = 0;
  static inline void enable()           { T__USART::ucsrb() = ucsrb_enable;  }
  static inline void disable()          { T__USART::ucsrb() = ucsrb_disable; }
  static inline void enableDREI()       { T__USART::ucsrb() = ucsrb_enable | (1<<T__USART::udrie); HWKICK();}
  static inline void disableDREI()      { T__USART::ucsrb() = ucsrb_enable;  }
  static inline void clear_txc()        { T__USART::ucsra() = T__USART::ucsra(); } // yep, TXC bit (and TXC bit only) is cleared by writing a 1, (ignoring recommendation for writing zero to UDRE0)
  static inline bool isParrityError()   { return  testBit(T__USART::ucsra(), T__USART::upe); }
  static inline bool isFrameError()     { return  testBit(T__USART::ucsra(), T__USART::fe ); }
  static inline bool isDataORunError()  { return  testBit(T__USART::ucsra(), T__USART::dor); }
  static inline bool isError()          { return  testBitsAny(T__USART::ucsra(), ((1<<T__USART::upe) | (1<<T__USART::fe) | (1<<T__USART::dor))); }
  static inline void setUBRR(unsigned int brs, unsigned char ucsraValue) { T__USART::ucsra() = ucsraValue; T__USART::ubbrh() = ((brs)>>8); T__USART::ubbrl() = (brs);}
  static inline void setConfig(unsigned char config)                     { T__USART::ucsrc() = (config|__AVR_ATmega8_config__);}
  static inline void setTxBusy()                                         {         setBit(T__USART::swReg,   T__USART::swReg_BUSY);}
  static inline void clrTxBusy()                                         {         clrBit(T__USART::swReg,   T__USART::swReg_BUSY);}
  static inline bool isTxBusy()                                          { return testBit(T__USART::swReg,   T__USART::swReg_BUSY);}
  static inline bool isU2X()                                             { return testBit(T__USART::ucsra(), T__USART::u2x); }
  static inline unsigned int  minIdleTime(unsigned char priority)        { return priority*(1+ (isU2X()? 0 : (unsigned char)(((unsigned char)(T__USART::ubbrh() <<2)) | ((unsigned char)(T__USART::ubbrl()>>6))))); }
  static inline unsigned int  cpuCylesPerBaudSample()                    { return 1 + T__USART::ubbrl() +(T__USART::ubbrh() <<8); }
  #define cpuCylesPerTimer0Tick 64 /* as set up by $Arduino/arduino-1.0.5/hardware/arduino/cores/arduino/wiring.c */
  // to go from cpuCylesPerBaudSample to tmr0TicksPerBaud, we have to multiply by the number of samples per baud (8 or 16), and devicde by cpuCylesPerTimer0Tick.
  static inline unsigned int  tmr0TicksPerBaud()                         { unsigned int temp = (cpuCylesPerBaudSample()-1)/(cpuCylesPerTimer0Tick/16); if(isU2X()) temp=temp/2; return temp+1; }
//end inline HW access convenience functions


  inline unsigned int getLatency()  // called from RXC interrupt. Returns latency since last rx, not counting local echo's, resets rx reference
  {
    unsigned char oldSREG = SREG;   // save interrupt state
    cli();                          // disable interrupts
    unsigned long now = timer0_overflow_count;
#if 1
/* code disabled */
/* todo normalize definition of TCNT0 */
#elif defined(TCNT0)
    unsigned char dt = TCNT0;
#elif defined(TCNT0L)
    unsigned char dt = TCNT0L;
#else
    #error TIMER 0 not defined
#endif
    unsigned long diff = now - T__RS485::lastRxTime;
    T__RS485::lastRxTime  = now;    // update for next time
    SREG = oldSREG;                 // restore interrupt state
    if((unsigned int)(diff>>16)) return 0xffff;  // optimized, but not yet fully optimized!! (can do better with __asm__)
    return (unsigned int) diff; // todo, normalize returned value in baud
  }

// interrupt handers (implemented as object methods as they need access to the RS485 object)
public: // until we figure out how, in a USART##__N## generic way, to become friends with the interrupt vectors
        // TODO make the functions defined by ISR(USART_<xx>_vect) a friend of HWSerial

  // static inline void setMode(unsigned char mode)         {}                //called externally //TODO encapsulate by a relay function

        
  inline void rxcInt()  //this interrupt is called when a newly read character is available
  {
    bool error=isError();                       // nb we have to fetch error status first, because
    unsigned char data = T__USART::udr();       // reading the character clears the error status.
    if(T__RS485::isEcho(data, error) ) return;  // the character was expected by the RS485

    if(getLatency()>timeoutLimit) myMessageFilter.terminateMessage();   // todo consider rxBuffer.storeChar(TMO); // storing tmout_char is optional

    if (error)
    {
      // todo errcnt++;
      return;   // nothing more we can do...
    }

    // message filtering during the interrupts, character are poked into the buffer, head-pointer is updated only after the message address is accepted
    bool bumpHeadPointer=true;
    unsigned char offset = myMessageFilter.check(data, bumpHeadPointer);
    if(offset<0) return; // message filtered out

    // note, messages which run into space problems will also be truncated and abandoned.
    if( !rxBuffer.pokeRelative(data, offset)) myMessageFilter.terminateMessage(); // we have no space so we reject the rest as well.
    else if(bumpHeadPointer)                  rxBuffer.bumpTo(offset);            // die binne benne benne binne
    return;
  }

  inline void dreInt()  //this interrupt is called when the transmit buffer is ready to take the next character.
  {
    if(        txBuffer.isEmpty() ) disableDREI(); // nothing left to do, (but we've got to keep on waiting... ? )
    else if( !T__RS485::txStart() ) disableDREI(); // RS485 not ready, we also stop (for the time being)
    else
    {
      unsigned char data = txBuffer.getChar();     // get the next data item
      T__USART::udr() = data;                      // copy the next data item to the transmit buffer
      setTxBusy();                                 // set transfer busy status
      T__RS485::txSendChar(data);                  // informs RS485 class on what character can be expected
    }
  }

  inline void txcInt() { clrTxBusy(); T__RS485::txCompleted(); } // this interrupt is called when transmission has stopped, we disable tx

public:

  inline void begin(unsigned long baud)
  {
    disable();
    T__RS485::begin();
    rxBuffer.reset();
    txBuffer.reset();
    setBaud(baud);
    enable();
  }

  inline void begin(unsigned long baud, unsigned char config)
  {
    disable();
    T__RS485::begin();
    rxBuffer.reset();
    txBuffer.reset();
    setBaud(baud);
    setConfig(config);
    enable();
  }

  inline void setAdressFilter(char** anAddressList, unsigned int theTimeoutValue=100)
  {
    myMessageFilter.setAdressList(anAddressList);
    // todo, or not to do, make use of theTimeoutValue
  }

  inline void setBaud(unsigned long baud)
  {
/* The ultimate Baud rate control strategy:

When U2X is used (i.e. set), the baud rate is twice as high for the same value of UBRR (Usart Baud Rate Register).
However, the number of samples taken per data bit (from now on denoted by NS) is halved (NS is 8 samples per bit
instead of 16 samples). Half the sampling makes the USART more sensitive to baud rate errors. On the other hand,
when using U2X the error on the baud rate generation can be smaller in certain circumstances. In the following we
deduce when the error of the baud rate would be larger than the sampling error, end hence where UX2 should be used.

Sampling error:
The bit value is defined by a majority vote of the three central data samples, i.e. samples 3,4,5 for NS=8
and sample 7,8,9 fof NS=16

The maximum tolerable error on the frequency occurs when either the earliest of the three samples of the stop
bit coincides with the leading edge of the stop bit, or the last of the three samples of the stop bit coincides
with the trailing edge of the stop bit, or:
  dT'*((ND+1.5)*NS+/-1) = dT*(ND+1.5 +/-0.5)*NS
  where ND the number of data bits,
        NS  the number of samples per data bit (either 8 or 16),
        dT the required period time, dT' the actual achieved period time

We assume that there will be slightly more than one stop bit and that consequently we may slightly relax. We can simplify the criteria
by requiring that the last of the three samples of the last data bit coincides with the trailing edge of the last data bit, which is
equals to the leading edge of the stop bit, we get:
  dT'*((ND +1 -/+0.5)*NS +/-1) = dT*(ND+1)*NS

  with dT' = 1/f'= 1/(f*(1+dR)=dT/(1+dR), with dR the relative frequency error, we get:

  (ND +1 -/+0.5)*NS +/-1 = (1+dr)*(ND+1)*NS
  (ND +1 -/+0.5) +/-1/NS = (ND+1) +dR*(ND+1)
  (-/+0.5 +/- 1/NS) = dR*(ND+1)
  dR = -/+ (8/16 - 1/NS)/(ND+1)

  (to be accurate: in case there is only one stop bit, the ND+1 should be replaces by ND+2 for the negative tolerance limit)

  the difference between the absolute values of the errors for NS=8 and for NS=16 is given by:
  delta-dR = (1/8 - 1/16) /(ND+1) = 1/16 /(ND+1),  which we approximate to 1/128 = 0.7%
  
  the actual sampling tolerances are given by  7/16 /(ND+1) = 4.9% and 3/8 /(ND+1) = 4.2%

Baud setting error
The UBBR register is given by UBBR = F_CPU/BAUD/NS -1, where NS the samples per bit (either 16 or 8)

The absolute value of the relative error due to the rounding is given by:
abs( (F_CPU/BAUD/NS) - ROUND(F_CPU/BAUD/NS) ) / (F_CPU/BAUD/NS)

Hence, if the difference in errors for NS=16 and for NS=8 exceeds 1/128, 8 bit sampling (i.e. set UX2) will provide more error tolerance
useUX2 = abs( (F_CPU/BAUD/16) - ROUND(F_CPU/BAUD/16) ) / (F_CPU/BAUD/16)     -abs( (F_CPU/BAUD/8) - ROUND(F_CPU/BAUD/8) ) / (F_CPU/BAUD/8)  > 1/128
       = abs( (F_CPU/BAUD/16) - ROUND(F_CPU/BAUD/16) ) / (F_CPU/BAUD/16) -0.5*abs( (F_CPU/BAUD/8) - ROUND(F_CPU/BAUD/8) ) / (F_CPU/BAUD/16) > 1/128 
       = abs( (F_CPU/BAUD/16) - ROUND(F_CPU/BAUD/16) ) -0.5*abs( (F_CPU/BAUD/8) - ROUND(F_CPU/BAUD/8) ) > (F_CPU/BAUD/16) / 128

The maximum possible error due to rounding of UBBR is 0.5, in this condition useUX2 is given by:
useUX2 =    (0.5 -0.5*0.5 > (F_CPU/BAUD/16)/128) = (1/4 > (F_CPU/BAUD/16)/128) = (1 > F_CPU/BAUD/512)     =   (baud > F_CPU/512)
Hence for F_CPU=16Mhz, U2X should never be used for baud<320000. For baud>32000 it will depend on the actual baud rate. In particular,
if the UBR value for NS=8 gives an even result, then the baud rate error for NS=16 and NS=8 will be identical and one should prefer
the options NS=16 as in this case the sampling error is better.
*/
/* Note on optimisation
this function has been coded such that if called with a compile time constant as argument, it translate into 5 or 6 instructions only.
*/

    unsigned long const bitRate_data     =(unsigned long)  (F_CPU<<4)/baud;     // 0x10 times the CPU clock cycles per baud (transmitted bit), 0x100 times the CPU clock cycles per sample (for NS=16)
    unsigned int  const bitRate_ubrr     =(unsigned short) (bitRate_data>>8);   // (down rounded!) CPU clock cycles per sample (NS=16),  value used for the baudrate register (+1)
      signed char const bitRate_fraction =(unsigned char)  bitRate_data;        // 0x100 times the remaining fraction, used to calculate the error

//  For NS=16, the error (times 0x100) to the nearest rounded ubrr value (up or down) is given by:
    unsigned char const error_NS16 = ((bitRate_fraction)<0) ? (~bitRate_fraction)  : bitRate_fraction; // i.e. negative values (= rounding up) are reversely folded to the interval [0x00 ... 0x7f]
//  For NS=8, the error (times 0x100) is the difference of bitRate_data with the nearest value of bitRate_data rounded to a multiple of 0x40, it can be obtained from error_NS16 by
//  error_NS08 = error_NS16 <0x40 ?  error_NS16 : 0x7f-error_NS16; // i.e. the values larger than [0x3f] are reversely folded to the interval [0x00 ... 0x3f]
//  For error_NS16 < 0x40, error_NS08 and error_NS16 are equal, otherwise error_NS08 is given by 0x7f-error_NS16,
//  hence the difference (error_NS16-error_NS08) is given by max(0, 2*error_NS16 -0x7f)
//  if this difference is larger than the difference of the error margins for sampling NS=16 and NS=8, (= 100/128 %, see above) we will use 8 bit sampling .i.e. U2X
    bool const useNS8 = /* (2*error_NS16-0x7f >= bitRate_data/128) = (error_NS16-0x40 >= bitRate_data/256)) */ (error_NS16 >= 0x40+bitRate_ubrr);

    if (useNS8)                  setUBRR( bitRate_ubrr<<1, 1<<T__USART::u2x ); // NS=8,  we do     use U2X, note: with rounding this will always give (2*ubrr+1)-1
    else if (bitRate_fraction<0) setUBRR( bitRate_ubrr,    0 );                // NS=16, we do not use U2X, we need to round up, i.e. do not subtract 1
    else                         setUBRR( bitRate_ubrr-1,  0 );                // NS=16, we do not use U2X, just subtract one

    T__RS485::setMinimumIdleTime(minIdleTime(ourPriority));

    #ifdef test_setBaud_report // only required for reporting during development/debugging
    unsigned char const error_NS08 = error_NS16 <0x40 ?  error_NS16 : 0x7f-error_NS16; // i.e. the values larger than [0x3f] are reversely folded to the interval [0x00 ... 0x3f]
    printf("baud=%7d bitRate=0x%08x (.ubrr=0x%04hx .fraction=0x%02hhx) error_NS16=0x%02hhx (% 6.2f%%) error_NS08=0x%02hhx(% 6.2f%%) error_NS16-error_NS08=0x%04hx(% 6.2f%%) threshold=0x%06x UBBR=0x%04x, U2X=%d\n",
            baud, bitRate_data, bitRate_ubrr, bitRate_fraction, error_NS16,(100.*error_NS16/bitRate_data), error_NS08, (100.*error_NS08/bitRate_data), error_NS16-error_NS08, (100.*(error_NS16-error_NS08)/bitRate_data), bitRate_ubrr*2, UBBR, U2X);
    #endif
   }

  inline void setPriority(unsigned char thePriority)
  {
    ourPriority = thePriority;
    T__RS485::setMinimumIdleTime(minIdleTime(ourPriority));
  }

  inline void reset()
  {
    disable();
    rxBuffer.reset();
    txBuffer.reset();
    enable();
  }

  inline void end()
  {
    flush();			// wait for transmission of outgoing data
    disable();			// uit met de pret
    rxBuffer.reset();	// clear any received data
    T__RS485::end();
  }

/* HardwareSerial::write() operation with RS485 class
   HardwareSerial::write(char c) is called with a char to be written to the serial device. When the txBuffer (transfer buffer) has space available, the character
   will be placed in the buffer and the Data Register Empty interrupt (DREI) will be enabled to ensure that characters from the output buffer are moved into the Data
   Register when this register is emptied.
   In case of a buffer full condition, then T__RS485::txReady() is checked for permanent errors, followed by a call to T__RS485::txInputIdleWait() to see if we
   did wait for buffer idle and hence whether DRE interrupt should be enabled.

   Enabling DREI will trigger a call to the Data Register Empty interrupt (DREI) method dreInt as soon as the Data Register is empty.

   The dreInt method checks the txBuffer, if there is no data to transmit, the DREIE will be cleared.
   Next dreInt will call T__RS485::txStart() to enable the 485. If the input idle condition was not met, T__RS485::txStart() will reject the request
   and return false, in which case the DREIE will be cleared.
   Otherwise data will be copied to the DRU to initiate a transmission and the RS485 handler will be informed about the character send though
   the call of T__RS485::txSendChar()

   Upon TransmissionCompleted (TXC) interrupt T__RS485::txCompleted() will be invoked to clear the RS485 tx-enable and to update the RS485 transmission state.

   All bytes written to the output are echoed back on the input and checked by the RXC interrupt handler against the bytes expected from transmission.
   Any data error or character mismatch, which indicates a data collision, will aborted the RS485 transfer.

   Currently the time of last character read interrupt is used for input idle, in future version the time of an input changed interrupt could be used.
*/
  virtual size_t write(uint8_t c)
  {
    unsigned int timeoutCount=0;

    // TODO flush before message start, message start is specified by message handler plug-in class.
    // TODO if(T__RS485::txError()) //retry, same code as flush ... ?

    while (txBuffer.isFull()) // output buffer is full, FF W88: wait for the interrupt handler to empty it a bit
    {
      if(false)                               {}              // lekker puh
      else if( !T__RS485::txReady() )         return 0;       // There is a permanent error, we should abandon.
      else if( T__RS485::txInputIdleWait() )  enableDREI();   // Waits for input idle condition, returns true if we should enable DRE interrupts
      else if( timeoutCount++ > dataSize )    return 0;       // un oeuf is an egg // TODO set error flag to abort message ?
      else                                    delayMicroseconds(baudTime); // wait for one baud
    }
    txBuffer.putChar(c);
    if( T__RS485::txReady() )
    {
      enableDREI(); // enable the Data Register Empty interrupt to make sure that the data is written to the USART
                    // n.b. if the buffer was not empty, then the interrupt was already enabled, however, it takes
                    // less instructions to just set the enable bit than to do any check. Furthermore we will then
                    // also have to worry about race conditions.
    }
    return 1;
  }

  inline size_t write(unsigned long n) { return write((uint8_t)n); }
  inline size_t write(long n)          { return write((uint8_t)n); }
  inline size_t write(unsigned int n)  { return write((uint8_t)n); }
  inline size_t write(int n)           { return write((uint8_t)n); }

  inline unsigned char pending()       { return txBuffer.itemsStored(); }
  virtual inline int   available()     { return rxBuffer.itemsStored(); }
  virtual inline int   peek()          { return rxBuffer.peekChar();    }
  virtual inline int   read()          { return rxBuffer.fetchChar();   }

  virtual inline void flush()          { (void) flush(5); }

  int flush(int retries)                        __attribute__((__used__));
  void diagnose(const __FlashStringHelper* =0)  __attribute__((__used__));

  private:
/* the following mechanism is the only way I found so far to force the emission flush(int) diagnose() and ourPriority
   test compilations:
cd $Arduino/myProjects/libraries/HardwareSerial_RS485
$Arduino/arduino-1.0.5/hardware/tools/avr/bin/avr-g++ -E -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -MMD -DARDUINO=105 -I$Arduino/arduino-1.0.5/hardware/arduino/cores/arduino -I$Arduino/arduino-1.0.5/hardware/arduino/variants/standard -I. HardwareSerial_RS485.cpp   >HardwareSerial_RS485.cpp.e
$Arduino/arduino-1.0.5/hardware/tools/avr/bin/avr-g++ -S -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -MMD -DARDUINO=105 -I$Arduino/arduino-1.0.5/hardware/arduino/cores/arduino -I$Arduino/arduino-1.0.5/hardware/arduino/variants/standard -I. HardwareSerial_RS485.cpp -o HardwareSerial_RS485.cpp.s
$Arduino/arduino-1.0.5/hardware/tools/avr/bin/avr-g++ -c -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -MMD -DARDUINO=105 -I$Arduino/arduino-1.0.5/hardware/arduino/cores/arduino -I$Arduino/arduino-1.0.5/hardware/arduino/variants/standard -I. HardwareSerial_RS485.cpp -o HardwareSerial_RS485.cpp.o
*/
  virtual void neverUsed()
  {
    flush(0);
    diagnose();
    ourPriority=0;
  }

  #ifdef diagnoseDetails
  unsigned char dbgFlushAvailableEntry;
  unsigned char dbgFlushAvailableExit;
  unsigned int  dbgFlushMillisExit;
  unsigned int  dbgFlushMicrosExtra;
#endif
}; // class HardwareSerial ==========================================================================================================================


#if defined(InHardwareSerial_RS485_CPP)
// this code is activated when the code is included from the HardwareSerial.CPP file
// this is my personal style choice to keep all HardwareSerial related code together in one file (at the price of some negligible extra bytes to be processed by the compiler).

//Instantiate the static structures and code
template<class T__USART, int T__TX_BufferSize, int T__RX_BufferSize, unsigned char T__RX_OverRunMarker, class T__RS485, class T__MessageFilter> RingBuffer<T__TX_BufferSize, false>                       HardwareSerial<T__USART, T__TX_BufferSize, T__RX_BufferSize, T__RX_OverRunMarker, T__RS485, T__MessageFilter>::txBuffer;
template<class T__USART, int T__TX_BufferSize, int T__RX_BufferSize, unsigned char T__RX_OverRunMarker, class T__RS485, class T__MessageFilter> RingBuffer<T__RX_BufferSize, false, T__RX_OverRunMarker>  HardwareSerial<T__USART, T__TX_BufferSize, T__RX_BufferSize, T__RX_OverRunMarker, T__RS485, T__MessageFilter>::rxBuffer;
template<class T__USART, int T__TX_BufferSize, int T__RX_BufferSize, unsigned char T__RX_OverRunMarker, class T__RS485, class T__MessageFilter> T__MessageFilter                                          HardwareSerial<T__USART, T__TX_BufferSize, T__RX_BufferSize, T__RX_OverRunMarker, T__RS485, T__MessageFilter>::myMessageFilter;
template<class T__USART, int T__TX_BufferSize, int T__RX_BufferSize, unsigned char T__RX_OverRunMarker, class T__RS485, class T__MessageFilter> unsigned char                                             HardwareSerial<T__USART, T__TX_BufferSize, T__RX_BufferSize, T__RX_OverRunMarker, T__RS485, T__MessageFilter>::ourPriority;
template<class T__USART, int T__TX_BufferSize, int T__RX_BufferSize, unsigned char T__RX_OverRunMarker, class T__RS485, class T__MessageFilter>

void HardwareSerial<T__USART, T__TX_BufferSize, T__RX_BufferSize, T__RX_OverRunMarker, T__RS485, T__MessageFilter>::diagnose(const __FlashStringHelper * reason)
{
//TODO optionally add hook for EEprom based HW debugging (store / report)
    unsigned char s_ucsra    = T__USART::ucsra();
    unsigned char s_ucsrb    = T__USART::ucsrb();
    unsigned char s_ubbrh    = T__USART::ubbrh();

    #ifdef diagnoseDetails
    RingBuffer<T__RX_BufferSize, false, T__RX_OverRunMarker> s_rxBuffer = rxBuffer;
    RingBuffer<T__TX_BufferSize>  s_txBuffer = txBuffer;
    #endif

	txBuffer.reset();
	rxBuffer.reset();

    T__RS485::diagnoseHelper(*this);

    print(F("Registers: UCSRA=0x")); print(s_ucsra,HEX); print(F(", UCSRB=0x")); print(s_ucsrb,HEX); print(F(", UBBRH=0x")); println(s_ubbrh,HEX); 
    #ifdef diagnoseDetails
    print(F("Flush data, AvailableEntry=")); print(dbgFlushAvailableEntry);
    print(F(" AvailableExit=")); print(dbgFlushAvailableExit);
    print(F(" MillisExit=")); print(dbgFlushMillisExit);
    print(F(" MicrosExtra=")); print(dbgFlushMicrosExtra); println();
    print(F("txBuffer: size=0x")); print(s_txBuffer.BUFFER_SIZE,HEX); print(F(", head=0x")); print(s_txBuffer.head,HEX);  print(F(", tail=0x")); println(s_txBuffer.tail,HEX); 
	s_txBuffer.head &= (s_txBuffer.SIZE_MASK);
	s_txBuffer.tail &= (s_txBuffer.SIZE_MASK);
	print(F("buffer ")); for(int i=0; i<s_txBuffer.BUFFER_SIZE; i++) { write(s_txBuffer.buffer[i]); print(i+1==s_txBuffer.head ? (i+1==s_txBuffer.tail? '*' : '<' ) : (i+1==s_txBuffer.tail? '>' :'|')); } println();
    print(F("rxBuffer: size=0x")); print(s_rxBuffer.BUFFER_SIZE,HEX); print(F(", head=0x")); print(s_rxBuffer.head,HEX);  print(F(", tail=0x")); println(s_rxBuffer.tail,HEX); 
	s_rxBuffer.head &= (s_rxBuffer.SIZE_MASK);
	s_rxBuffer.tail &= (s_rxBuffer.SIZE_MASK);
	print(F("buffer ")); for(int i=0; i<s_rxBuffer.BUFFER_SIZE; i++) { write(s_rxBuffer.buffer[i]); print(i+1==s_rxBuffer.head ? (i+1==s_rxBuffer.tail? '*' : '<' ) : (i+1==s_rxBuffer.tail? '>' :'|')); } println();
    #endif
    if(reason != NULL) { print(F("Diagnostic called for: ")); println(reason); }
}


template<class T__USART, int T__TX_BufferSize, int T__RX_BufferSize, unsigned char T__RX_OverRunMarker, class T__RS485, class T__MessageFilter>
int HardwareSerial<T__USART, T__TX_BufferSize, T__RX_BufferSize, T__RX_OverRunMarker, T__RS485, T__MessageFilter>::flush(int retries)
{

  // if RS485 was not started, kick it now!
  // TODO

  // give it a number of retries in case of RS485 collision
  while(retries > 0)
  {
    retries--;

    if(T__RS485::txError() ) // transfer is in error, retry
    {
      // reset message
      T__RS485::reset(); // activate the RS485 again for next transfer.
      enableDREI();
    }


    if( T__RS485::txInputIdleWait() ) enableDREI();   // Waits for input idle condition, enable DRE interrupts if needed

#ifdef diagnoseDetails
    unsigned long millisIn = millis();
    dbgFlushAvailableEntry = txBuffer.itemsStored();
    //// digitalWrite(12, HIGH);
#endif
    // transfer is going, wait for the buffer to be emptied

    unsigned int tmout=0;
    while ( !txBuffer.isEmpty() ) // wait for the buffer empty condition (nb, beware of optimizer txBuffer.tail should be declared volatile)
    {
      delayMicroseconds(100); if( ++tmout > 40000) { diagnose(F("flush1")); return 0; }
      if(T__RS485::txError() ) break; // if RS485 transfer is in error, escape
    }
#ifdef diagnoseDetails
    //// digitalWrite(12, LOW);
    dbgFlushAvailableExit = txBuffer.itemsStored();
    dbgFlushMillisExit    = millis() - millisIn;
    dbgFlushMicrosExtra   = 0;
#endif

    /* We have 2 characters in the pipeline after the txBuffer is emptied. Wait until the transfer is fully finished to avoid any troubles when switching to RS485 mode:
       In case we switch over to 485 mode with the pipeline not emptied, the last character in the pipeline will appear on the 485 bus:
       after accepting the first character in 485 mode, the DRE interrupt will be called as soon as the last character of the previous (non 485)
       transfer is taken out from the Data Register. The txStart called from the DRE interrupt will now set the 485 txEnable. However, the transfer
       of this last character still has to take place, and hence this character will be output to the 485 and echoed back to the input.
    */
    while(isTxBusy())
    {
#ifdef diagnoseDetails
      dbgFlushMicrosExtra +=25;
      if( dbgFlushMicrosExtra > 40000) { diagnose(F("flush-e")); return 0; }
#endif
      delayMicroseconds(25);
    }

    if(txBuffer.isEmpty() ) break; // all has gone out, we may no proceed on our way...
  }

  if(T__RS485::txError() ) diagnose(F("flush2")); // if RS485 transfer is in error abort and force diagnostics over RS232
  T__RS485::reset(); // activate the RS485 again for next transfer.
  
  return 1;
}

#endif // defined(InHardwareSerial_RS485_CPP)

#endif // !defined(HardwareSerial_RS485Enabled_h)
