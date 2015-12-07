/* HardwareSerialRS485_Enabled.h copyright notice

RS485 enabled implementation of HardwareSerial (replacing the classical version for Wiring by Nicholas Zambetti, Mark Sproul, Alarus)

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

#if !defined(HardwareSerialRS485_Enabled_h)
#define      HardwareSerialRS485_Enabled_h

/* Description
The HardwareSerialRS485_Enabled.h provides an RS485 enabled version of the HardwareSerial class. By default the code implements a dummy RS485 handler
and provides exactly the same functionality as the classical version (However, it uses less code an runs faster (to be quantified)
To effectively use this class with an external RS232 to RS485 driver receiver circuit, the helper class RS485serial (included from HardwareSerialRS485_helper.h)
must be passed as a template parameter. See also HardwareSerialRS485.h
*/

/* todo: improvements for HardwareSerialRS485_Enabled:
   reconsider justification using HardwareSerialRS485_Enabled as a class and not instance (one class per USART)
   (i.e. USART is also static for efficiency (code and space), so there is one class per USART, making RingBuffers static further improves efficiency (to be evaluated).
 */

#include <inttypes.h>
#include "utility/BitManipulation.h"
#include "utility/USARTdef.h"
#include "utility/HardwareSerialRS485_Tracer.h"

#include "Stream.h"
#include "Print.h"
#define diagnoseFlushOnError    // if defined, call diagnostics on flush error
#define NOdiagnoseBuffers         // if defined, it causes contents of buffers to be dumped by diagnostics
#define diagnoseFlushLed  12    // if defined, it causes the led to be on while flush is waiting

// HWKICK() can be defined externally to emulate HW for development and debugging (HWKICK() is called from enableDREI).
extern void HWKICK() __attribute__((weak)); inline void HWKICK() { /* niks */ }  // If not defined externally this weak dummy is used.


class RS485_dummy // a dummy version to be used if the RS485 protocol is not needed ==================================================================
{
// this class is provided here to provide efficient non-RS485 enabled HardwareSerial functionality...
protected:

// The dummy functions still have to declare the parameters although they don't use them. To avoid warning messages, we ignore them here
// Alternative is to create a dummy? reference to them in the code, e.g. txSendChar(char c) { (void) c; }
#pragma GCC diagnostic push // we only ignore them from push to pop
#pragma GCC diagnostic ignored "-Wunused-parameter"
  static inline void begin()                                { }               //called from the begin method
  static inline void reset()                                { }               //set txState back to 
  static inline void setMinimumIdleTime(long unsigned int)  { }
  static inline bool txReady()                              { return true;  } // called from the write method, returns true if the DRE interrupt should to be enabled.
  static inline bool txError()                              { return false; } // called from flush, returns true if tranfer has been interrupted due to an error.
  static inline bool txInputIdleWait()                      { return false; } // called from the write method, returns true if we did wait for input idle and hence DRE should be enabled
  static inline bool txStart()                              { return true;  } // called from DRE interrupt to set 485 transmission enable. gives go/no-go for transmission.
  static inline void txCompleted()                          { }               // called from TX completed interrupt when all data has gone out, disable tx and updates txState.
  static inline void txSendChar(unsigned char c)            {} // called from DRE interrupt to informs RS485 class on what character can be expected.
  static inline bool isEcho(unsigned char c, bool error)    { return false; } //called from RXC interrupt. Returns true when the character was expected from local echo.

  unsigned long volatile rxTime;      // time of last data reception, we also need this in dummy...
  unsigned char txState;              // and this one
  public:

  static inline unsigned char getMode()                     { return 0; }
  static inline void          setMode(unsigned char mode)   { }
  static inline void          set232Mode()                  { }

  inline void startTransaction()                                                                                      {}
  inline void setTransactionError(unsigned char theErrorType, unsigned char txState, unsigned char currentPosition)   { }
  inline unsigned char endTransaction()                                                                               { return 0; }
  inline bool isTransactionActive()                                                                                   { return false; }

  static const unsigned char RS323mode = 0x25;
//  Todo these are transaction_dummy constants, organize them better
//  const static unsigned char TransactionActive      =0x01;
  const static unsigned char WriteBufferFullTimeout =0x11;
  const static unsigned char FlushPipelineError     =0x12;
  const static unsigned char FlushTimeoutExpired    =0x14;
  const static unsigned char NoRecoveryError        =0x30;


#pragma GCC diagnostic pop
}; // class RS485_dummy ==============================================================================================================================


class MessageFilter_dummy // a passAll filter class to be used if filters are not needed =============================================================
{
  public:
  inline MessageFilter_dummy()                          {}
  inline void terminateMessage()                        {}
  inline unsigned char getTimeout()                     { return 0xff; }
  inline void setTimeout(unsigned char aTimeout)        { (void) aTimeout; }
  inline void setAddressList(char** anAddressList)      { (void) anAddressList;  }
  char        check(char c)                             { (void) c; return 0x10; }   // (void) c; to avoid a -Wunused-parameter warning
}; // MessageFilter_dummy ============================================================================================================================


/* RingBuffer // =====================================================================================================================================
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
//static const unsigned char BUFFER_SIZE =  (1<<T__BUFFER_SIZE_EXPONENT);
//static const unsigned char SIZE_MASK   =  BUFFER_SIZE-1;
  static const unsigned char SIZE_MASK   =  ~(-1<<T__BUFFER_SIZE_EXPONENT);
  static const unsigned int  BUFFER_SIZE = SIZE_MASK+1;
  static const unsigned char SPACE_LIMIT = SIZE_MASK!=0xff ? BUFFER_SIZE : SIZE_MASK;  // TODO check if this is resolved at compile time, otherwise use a constexpr function
  static const bool          markOverRun = (T__overRunMarker!=' ');

  volatile unsigned char head;  // pointer to the next character to be written in the buffer. //todo volatile impacts optimizations, try to avoid its usage
  volatile unsigned char tail;  // pointer to the next character to be read  from the buffer. //idem

  unsigned char buffer[BUFFER_SIZE];

public:
  inline void reset()                       { head=tail=0;}
  inline void rewind()                      { tail=0;}
  inline bool hasWrapped()                  { return head>SIZE_MASK;}  // this means that the buffer head pointer has wrapped around (and data was overwritten) after a reset.
                                                                       // TODO fix transaction.recovery detection for message length > 256 using a transfer count in transaction object

  inline unsigned char itemsStored()        { return (unsigned char)(head-tail); }
  inline unsigned char availableSpace()     { return (unsigned char)(SPACE_LIMIT -itemsStored() ); }

  inline bool isFull()                      { return itemsStored() == SPACE_LIMIT; }
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
}; // template class RingBuffer ======================================================================================================================

class Timeout // =====================================================================================================================================
{
  unsigned long timeLimit;
  public:
  Timeout (unsigned long value) { set(value); }
  void set(unsigned long value) { timeLimit= (nowStamp()+value); }
  bool isExpired() { return ((signed long)(nowStamp()-timeLimit))>0; }  // zero wait time is the next tick (i.e. 0.5 ticks on average), one tick wait time, until the second tick (i.e. 1.5 ticks on average)
  long remaining() { return ((signed long)(nowStamp()-timeLimit));   }
}; // Timeout ========================================================================================================================================

template <class T__USART, int T__TX_BufferSize=6, int T__RX_BufferSize=6, unsigned char T__RX_OverRunMarker=' ', class T__RS485=RS485_dummy, class T__MessageFilter=MessageFilter_dummy>
class HardwareSerialRS485 : T__USART, public Stream, public T__RS485
{
// NB: contrary to the impression given, all T__RS485::<method> occurences are instance methods on our T__RS485 subclass and not static (aka class) methods on the T__RS485 class.
private:
  static RingBuffer<T__RX_BufferSize, false, T__RX_OverRunMarker> rxBuffer;
  static RingBuffer<T__TX_BufferSize> txBuffer;
  static unsigned int     ourTimer0TicksPerSymbol;  // number of timer 0 ticks per data item, range: 0...0x3400
  static T__MessageFilter myMessageFilter;
  static inline unsigned long minimumIdleTime(unsigned char priority)    { return priority*ourTimer0TicksPerSymbol; }

//inline HW access convenience functions:
  static const unsigned char ucsrb_enable  = (unsigned char) ((1<<T__USART::rxen) |(1<<T__USART::txen) |(1<<T__USART::rxcie) |(1<<T__USART::txcie));
  static const unsigned char ucsrb_disable = 0;
  static inline void enable()           { T__USART::ucsrb() = ucsrb_enable;  }
  static inline void disable()          { T__USART::ucsrb() = ucsrb_disable; }
  static inline void enableDREI()       { T__USART::ucsrb() = ucsrb_enable | (1<<T__USART::udrie); HWKICK();}  // enables dreInt(), in case transmit buffer is ready to take a character.
  static inline void disableDREI()      { T__USART::ucsrb() = ucsrb_enable;  }
  static inline void clear_txc()        { T__USART::ucsra() = T__USART::ucsra(); } // yep, TXC bit (and TXC bit only) is cleared by writing a 1, (ignoring recommendation for writing zero to UDRE0)
  static inline bool isParrityError()   { return  testBit(T__USART::ucsra(), T__USART::upe); }
  static inline bool isFrameError()     { return  testBit(T__USART::ucsra(), T__USART::fe ); }
  static inline bool isDataORunError()  { return  testBit(T__USART::ucsra(), T__USART::dor); }
  static inline bool isError()          { return  testBitsAny(T__USART::ucsra(), ((1<<T__USART::upe) | (1<<T__USART::fe) | (1<<T__USART::dor))); }
  static inline void setUBRR(unsigned int brs, unsigned char ucsraValue) { T__USART::ucsra() = ucsraValue; T__USART::ubrrh() = ((brs)>>8); T__USART::ubrrl() = (brs);}
  static inline void setConfig(unsigned char config)                     { T__USART::ucsrc() = (config|__AVR_ATmega8_config__);}
  static inline void setTxBusy()                                         {         setBit(T__USART::swReg,   T__USART::swReg_BUSY);}
  static inline void clrTxBusy()                                         {         clrBit(T__USART::swReg,   T__USART::swReg_BUSY);}
  static inline bool isTxBusy()                                          { return testBit(T__USART::swReg,   T__USART::swReg_BUSY);}
  static inline bool isU2X()                                             { return testBit(T__USART::ucsra(), T__USART::u2x); }
//end inline HW access convenience functions

// interrupt handers (implemented as object methods as they need access to the RS485 object)
public: // until we figure out how, in a USART##__N## generic way, to become friends with the interrupt vectors
        // Todo make the functions defined by ISR(USART_<xx>_vect) a friend of HWSerial


  inline void rxcInt()  //this interrupt is called when a newly read character is available
  {
    bool error=isError();                           // nb we have to fetch error status first, because
    unsigned char data = T__USART::udr();           // reading the character clears the error status.
    if(T__RS485::isEcho(data, error) ) return;      // the character was expected by the RS485

    unsigned long lastRxTime = T__RS485::rxTime;    // temporary
    T__RS485::rxTime         = nowStamp_IC();       // update for next time
    unsigned long latency    = T__RS485::rxTime -lastRxTime;

    if(latency> myMessageFilter.getTimeout()*ourTimer0TicksPerSymbol)
    {
      TRACE(rxcInt_latencyLimit);           // latency
      myMessageFilter.terminateMessage();   // measured arrival interval (average over 8 frames @9600 baud): 260.125 ticks; diagnostic: ourTimer0TicksPerSymbol=260
      // there is no need to store a timeout character here, as we will synchronize with the next SOM anyway, hence there is no possibility that we merge two messages...
    }
    if (error) return;   // nothing more we can do...

    // message filtering during the interrupts, character are poked into the buffer, the head-pointer is updated only after the message address is accepted
    char filterState = myMessageFilter.check(data);
    if(filterState<0) return; // message filtered out

    // note, messages which run into space problems will also be truncated and abandoned.
    if( !rxBuffer.pokeRelative(data, filterState&0x0f)) myMessageFilter.terminateMessage(); // we have no space so we reject the rest as well.
    else if(filterState&0xf0)                           rxBuffer.bumpTo(filterState&0x0f);  // die binne benne benne binne
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
      T__RS485::txSendChar(data);                  // called from DRE interrupt to informs RS485 class on what character can be expected
    }
  }

  inline void txcInt()  //this interrupt is called when all tx data has gone out. Disable tx and updates txState.
  {
    clrTxBusy();
    T__RS485::txCompleted();
  }

public:
//This is a singleton class (i.e. a class with only one object). We instantiate a reference here:
  static HardwareSerialRS485<T__USART, T__TX_BufferSize, T__RX_BufferSize, T__RX_OverRunMarker, T__RS485, T__MessageFilter>& ourSerialObject;
//alt  static HardwareSerialRS485<T__USART, T__TX_BufferSize, T__RX_BufferSize, T__RX_OverRunMarker, T__RS485, T__MessageFilter> ourSerialObject;

  inline void begin(unsigned long baud)
  {
    disable();
    T__RS485::begin();
    rxBuffer.reset();
    txBuffer.reset();
    setBaud(baud);
    enable();
  }

  
// todo document this convenience function to create a config value
  static inline unsigned char makeConfig(unsigned char bits, char parity, unsigned char stop) { return 2*(bits-5)+ 8*(stop-1) + parity=='N'? 0x00 :parity=='M'? 0x10 :parity=='E'? 0x20 :parity=='O'? 0x30 : 0x00 ;}
  inline void begin(unsigned long baud, unsigned char config)
  {
    disable();
    T__RS485::begin();
    rxBuffer.reset();
    txBuffer.reset();
    setConfig(config);
    setBaud(baud);
    enable();
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
The UBRR register is given by UBRR = F_CPU/BAUD/NS -1, where NS the samples per bit (either 16 or 8)

The absolute value of the relative error due to the rounding is given by:
abs( (F_CPU/BAUD/NS) - ROUND(F_CPU/BAUD/NS) ) / (F_CPU/BAUD/NS)

Hence, if the difference in errors for NS=16 and for NS=8 exceeds 1/128, 8 bit sampling (i.e. set UX2) will provide more error tolerance
useUX2 = abs( (F_CPU/BAUD/16) - ROUND(F_CPU/BAUD/16) ) / (F_CPU/BAUD/16)     -abs( (F_CPU/BAUD/8) - ROUND(F_CPU/BAUD/8) ) / (F_CPU/BAUD/8)  > 1/128
       = abs( (F_CPU/BAUD/16) - ROUND(F_CPU/BAUD/16) ) / (F_CPU/BAUD/16) -0.5*abs( (F_CPU/BAUD/8) - ROUND(F_CPU/BAUD/8) ) / (F_CPU/BAUD/16) > 1/128 
       = abs( (F_CPU/BAUD/16) - ROUND(F_CPU/BAUD/16) ) -0.5*abs( (F_CPU/BAUD/8) - ROUND(F_CPU/BAUD/8) ) > (F_CPU/BAUD/16) / 128

The maximum possible error due to rounding of UBRR is 0.5, in this condition useUX2 is given by:
useUX2 =    (0.5 -0.5*0.5 > (F_CPU/BAUD/16)/128) = (1/4 > (F_CPU/BAUD/16)/128) = (1 > F_CPU/BAUD/512)     =   (baud > F_CPU/512)
Hence for F_CPU=16Mhz, U2X should never be used for baud<32000. For baud>32000 it will depend on the actual baud rate. In particular,
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

    // calculate tmr0TicksPerSymbol, giving the number of ticks of timer-0 per data item

    unsigned char const      startSize = 1;
    unsigned char const       charSize = (testBit(T__USART::ucsrb(), T__USART::ucsz2) ? 9: (testBit(T__USART::ucsrc(), T__USART::ucsz1)? 2:0) +(testBit(T__USART::ucsrc(), T__USART::ucsz0)? 6:5) );
    unsigned char const     paritySize = (testBit(T__USART::ucsrc(), T__USART::upm1 ) ? 1: 0                                                                                                      );
    unsigned char const       stopSize = (testBit(T__USART::ucsrc(), T__USART::usbs ) ? 2: 1                                                                                                      );
    unsigned char const baudsPerSymbol = startSize +charSize +paritySize +stopSize;                 // range:  7 ...    13
    
    unsigned char const cpuCylesPerTimer0Tick = 64; // as set up by $Arduino/arduino-1.0.5/hardware/arduino/cores/arduino/wiring.c, i.e. 4uSec / tick for fCPU=16MHz
    unsigned int  const cpuCylesPerSample     = 1 + T__USART::ubrrl() +(T__USART::ubrrh() <<8);     // range:  1 ...  4096 (ubrr has 12 effective bits)
    unsigned long const cpuCylesPerBaud       = useNS8? cpuCylesPerSample*8: cpuCylesPerSample*16;  // range:  8 ... 65536
    unsigned long const cpuCylesPerSymbol     = cpuCylesPerBaud *baudsPerSymbol;                    // range: 56 ...851968

    ourTimer0TicksPerSymbol = cpuCylesPerSymbol/cpuCylesPerTimer0Tick;                              // range:  0 ... 13312

    #ifdef test_setBaud_report // only required for reporting during development/debugging
    unsigned char const error_NS08 = error_NS16 <0x40 ?  error_NS16 : 0x7f-error_NS16; // i.e. the values larger than [0x3f] are reversely folded to the interval [0x00 ... 0x3f]
    printf("baud=%7d bitRate=0x%08x (.ubrr=0x%04hx .fraction=0x%02hhx) error_NS16=0x%02hhx (% 6.2f%%) error_NS08=0x%02hhx(% 6.2f%%) error_NS16-error_NS08=0x%04hx(% 6.2f%%) threshold=0x%06x UBRR=0x%04x, U2X=%d\n",
            baud, bitRate_data, bitRate_ubrr, bitRate_fraction, error_NS16,(100.*error_NS16/bitRate_data), error_NS08, (100.*error_NS08/bitRate_data), error_NS16-error_NS08, (100.*(error_NS16-error_NS08)/bitRate_data), bitRate_ubrr*2, UBRSR, U2X);
    #endif
   }

  void startTransaction(unsigned char thePriority=0x1f)
  {
      flush(); // makes sure pipeline is empty
      T__RS485::startTransaction();
      T__RS485::setMinimumIdleTime( minimumIdleTime(thePriority & 0x1f ) );
      txBuffer.reset();
  }

  unsigned char endTransaction()
  {
      flush(); // makes sure pipeline is empty
      return T__RS485::endTransaction();
  }
  
  inline void setAddressFilter(char** anAddressList, unsigned int theTimeoutValue=10)
  {
    myMessageFilter.setAddressList(anAddressList);
    myMessageFilter.setTimeout(theTimeoutValue);
  }

  inline unsigned char getInputLatency()  // returns the delay since last character read, in number of characters equivalent time.
  {
    unsigned long latency    = (nowStamp_IC() -T__RS485::rxTime)/ourTimer0TicksPerSymbol;
    return (latency>0xff) ? 0xff : (unsigned char) latency;
  }

  inline int availableForWrite(void)    { return txBuffer.availableSpace(); }
  inline size_t write(unsigned long n) 	{ return write((uint8_t)n); }
  inline size_t write(long n)          	{ return write((uint8_t)n); }
  inline size_t write(unsigned int n)  	{ return write((uint8_t)n); }
  inline size_t write(int n)           	{ return write((uint8_t)n); }

  inline unsigned char pending()       	{ return txBuffer.itemsStored(); }
  virtual inline int   available()     	{ return rxBuffer.itemsStored(); }
  virtual inline int   peek()          	{ return rxBuffer.peekChar();    }
  virtual inline int   read()          	{ return rxBuffer.fetchChar();   }

  virtual inline void  flush()          { (void) flush(5); }

  virtual size_t write(uint8_t c)                                                   __attribute__((__used__));
  int flush(unsigned char retries)                                                  __attribute__((__used__));
  void diagnose(const __FlashStringHelper* reason=0, Print& out=ourSerialObject)    __attribute__((__used__)); // TODO investigate why I cannot use *this ?

private:

inline void emptyPipeline() // method to wait until all data in the pipeline have been emptied.
{
  /* There are still 2 characters in the pipeline after the txBuffer is emptied. Wait until the transfer is fully finished to avoid the last character
     in the pipeline to appear on the 485 bust when switching over to RS485 mode. In absence of this wait the following will happen:
     After accepting the first character in 485 mode, the DRE interrupt will be called when the last character of the previous (non 485) transfer is
     taken from the Data Register. The txStart, called from the DRE interrupt, sets the 485 txEnable. However, the transfer of this last character
     still has to take place, and hence this character will be output to the 485 and echoed back to the input.
  */
  TRACE(emptyPipeline); // isTxBusy()
  Timeout timeout(ourTimer0TicksPerSymbol+230); // 4 ticks spare //TODO TODO It seem to take longer on the MICRO than the UNO. Validate and investigate why. Possible difference is use of TRACE during tests on UNO?
  while(isTxBusy())
  {
    if( timeout.isExpired() )
    {
       TRACE(emptyPipeline_expired); // nowStamp2(), txState
       return;
    }
  }
  TRACE(emptyPipeline_completed); // nowStamp2(), txState
}


inline void recoverTransfer(bool waitForIdle) // method to recover from an RS485 error
{
  if(T__RS485::txError() )          // resolve any blockage of RS485 due to collision
  {
    emptyPipeline();                // 
    T__RS485::reset();              //
    if(T__RS485::isTransactionActive())
    {
      if(txBuffer.hasWrapped())     { T__RS485::setTransactionError(T__RS485::NoRecoveryError, T__RS485::txState, txBuffer.head ); return; }
      txBuffer.rewind();            // we start all over now
    }
  }
  if(waitForIdle)                   // hoge nood
  {
    if (T__RS485::txState ==1) T__RS485::txState =0;    // Shit piss and corruption, the previous caller did not flush
    else if (T__RS485::txState !=0) return;             // transfer is alreaddy going, do not wait
    TRACE(txInputIdleWait); // nowStamp2(), txState, timeSinceRX()
    T__RS485::txInputIdleWait();
    TRACE(recoverTransfer_enableDREI); // nowStamp2(), txState
    enableDREI();
  }

}


/* the following mechanism is the only way I found so far to force the emission of flush(int) and diagnose() */
  virtual void neverUsed() //todo, check for alternatives, look at possibility of #pragma interface / #pragma implementation
  {
    flush(0);
    diagnose();
  }


}; // class HardwareSerialRS485 ======================================================================================================================


/* to test compilations (bash shell syntax):
export ArduinoIDE=/fill-in-your-path-to-arduino-IDE-folder
export ArduinoProjects=/fill-in-your-path-to-arduino-project-folder
cd $ArduinoProjects/libraries/HardwareSerialRS485
$ArduinoIDE/hardware/tools/avr/bin/avr-g++ -E -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -MMD -DARDUINO=105 -I$ArduinoIDE/hardware/arduino/avr/cores/arduino -I$ArduinoIDE/hardware/arduino/avr/variants/standard -I. HardwareSerialRS485.cpp   >HardwareSerialRS485.cpp.e
$ArduinoIDE/hardware/tools/avr/bin/avr-g++ -S -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -MMD -DARDUINO=105 -I$ArduinoIDE/hardware/arduino/avr/cores/arduino -I$ArduinoIDE/hardware/arduino/avr/variants/standard -I. HardwareSerialRS485.cpp -o HardwareSerialRS485.cpp.s
$ArduinoIDE/hardware/tools/avr/bin/avr-g++ -c -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -MMD -DARDUINO=105 -I$ArduinoIDE/hardware/arduino/avr/cores/arduino -I$ArduinoIDE/hardware/arduino/avr/variants/standard -I. HardwareSerialRS485.cpp -o HardwareSerialRS485.cpp.o
*/


#if defined(Implement_HardwareSerialRS485) // ++=========================================================================================================
// this code is activated when the code is included from the HardwareSerialRS485.CPP file
// this is my personal style choice to keep all HardwareSerialRS485 related code together in one file (at the price of some negligible extra bytes to be processed by the compiler).

//Instantiate the static structures and code
#define template_HardwareSerialRS485_context     template<class T__USART, int T__TX_BufferSize, int T__RX_BufferSize, unsigned char T__RX_OverRunMarker, class T__RS485, class T__MessageFilter>
#define template_HardwareSerialRS485_invocation  HardwareSerialRS485<T__USART, T__TX_BufferSize, T__RX_BufferSize, T__RX_OverRunMarker, T__RS485, T__MessageFilter>
template_HardwareSerialRS485_context RingBuffer<T__TX_BufferSize, false>                       template_HardwareSerialRS485_invocation::txBuffer;
template_HardwareSerialRS485_context RingBuffer<T__RX_BufferSize, false, T__RX_OverRunMarker>  template_HardwareSerialRS485_invocation::rxBuffer;
template_HardwareSerialRS485_context T__MessageFilter                                          template_HardwareSerialRS485_invocation::myMessageFilter;
template_HardwareSerialRS485_context unsigned int                                              template_HardwareSerialRS485_invocation::ourTimer0TicksPerSymbol;

template_HardwareSerialRS485_context void template_HardwareSerialRS485_invocation::diagnose(const __FlashStringHelper* reason, Print& out) /* todo validate */// ==============================
{
    #ifdef diagnoseBuffers
    RingBuffer<T__RX_BufferSize, false, T__RX_OverRunMarker> s_rxBuffer = rxBuffer;
    RingBuffer<T__TX_BufferSize>  s_txBuffer = txBuffer;
    #endif // diagnoseBuffers


	TRACE(_DISABLE_); // alternative: make snapshot instead...

    unsigned char savedMode =T__RS485::getMode();
    T__RS485::set232Mode(); // disable 485

	txBuffer.reset();
	rxBuffer.reset();

    if(reason != NULL) { out.print(F("\nDiagnostic called for: ")); out.println(reason); }
    out.print("ourTimer0TicksPerSymbol="  ); out.print(ourTimer0TicksPerSymbol);
    out.println();

    #ifdef diagnoseBuffers
    out.print(F("txBuffer: size=0x")); out.print(s_txBuffer.BUFFER_SIZE,HEX); out.print(F(", head=0x")); out.print(s_txBuffer.head,HEX);  out.print(F(", tail=0x")); out.println(s_txBuffer.tail,HEX); 
	s_txBuffer.head &= (s_txBuffer.SIZE_MASK);
	s_txBuffer.tail &= (s_txBuffer.SIZE_MASK);
	out.print(F("buffer ")); for(int i=0; i<s_txBuffer.BUFFER_SIZE; i++) { write(s_txBuffer.buffer[i]); out.print(i+1==s_txBuffer.head ? (i+1==s_txBuffer.tail? '*' : '<' ) : (i+1==s_txBuffer.tail? '>' :'|')); } out.println();
    out.print(F("rxBuffer: size=0x")); out.print(s_rxBuffer.BUFFER_SIZE,HEX); out.print(F(", head=0x")); out.print(s_rxBuffer.head,HEX);  out.print(F(", tail=0x")); out.println(s_rxBuffer.tail,HEX); 
	s_rxBuffer.head &= (s_rxBuffer.SIZE_MASK);
	s_rxBuffer.tail &= (s_rxBuffer.SIZE_MASK);
	out.print(F("buffer ")); for(int i=0; i<s_rxBuffer.BUFFER_SIZE; i++) { write(s_rxBuffer.buffer[i]); out.print(i+1==s_rxBuffer.head ? (i+1==s_rxBuffer.tail? '*' : '<' ) : (i+1==s_rxBuffer.tail? '>' :'|')); } out.println();
    #endif // diagnoseBuffers

	TRACE(_REPORT_);
    flush();
	TRACE(_ENABLE_);
    T__RS485::setMode(savedMode); // restore 485 state

} // diagnose() // ===================================================================================================================================

/* HardwareSerialRS485::write(char c) is called with a char to be written to the serial device. In case the RS485 transfer problems, appropriate actions 
   are taken to restart the transfer. If the txBuffer is full, write will wait a duration equal to the transmission time of one character for space
   to become available. Provided the txBuffer has space available, the character is stored in the txBuffer and the Data Register Empty Interrupt
   (DREI) is enabled.
   Enabling DREI triggers a call to the Data Register Empty Interrupt (DREI) method dreInt as soon as the Data Register is empty.

   The dreInt method checks the txBuffer, if there is no data to transmit, the DREIE will be cleared.
   Next dreInt will call T__RS485::txStart() to enable the 485. If the input idle condition was not met, T__RS485::txStart() will reject the request
   and return false, in which case the DREIE will be cleared.
   Otherwise data will be copied to the DRU to initiate a transmission and the RS485 handler will be informed about the character send though
   the call of T__RS485::txSendChar()

   Upon TransmissionCompleted (TXC) interrupt T__RS485::txCompleted() will be invoked to clear the RS485 disables tx and to update the RS485 txState.

   All bytes written to the output are echoed back on the input and checked by the RXC interrupt handler against the bytes expected from transmission.
   Any data error or character mismatch, which indicates a data collision, will aborted the RS485 transfer.

   Currently the time of last character read interrupt is used for input idle, in future version the time of an input changed interrupt could be used.
*/
template_HardwareSerialRS485_context size_t template_HardwareSerialRS485_invocation::write(uint8_t c) // ===============================================
{
  TRACE(write); // nowStamp4(), txState, txBuffer.itemsStored(), c
  recoverTransfer(txBuffer.isFull());	// and wait for idle input if the buffer is full

  Timeout timeout(ourTimer0TicksPerSymbol+4);

  while (txBuffer.isFull()) if (timeout.isExpired() | T__RS485::txError())
  {
    TRACE(write_timeout); // nowStamp2(), txState
    T__RS485::setTransactionError(T__RS485::WriteBufferFullTimeout, T__RS485::txState, txBuffer.head );
    return 0;
  }

  txBuffer.putChar(c);

  TRACE(write_enableDREI); // nowStamp2(), txState, txBuffer.itemsStored()()
  enableDREI (); // this will set all actions going
  return 1;
} // write() =========================================================================================================================================

template_HardwareSerialRS485_context int template_HardwareSerialRS485_invocation::flush(unsigned char retries) // ========================================================
{
  while (retries--)
  { 
    TRACE(flush); // nowStamp4(), txState, txBuffer.itemsStored(), retries

    recoverTransfer(!txBuffer.isEmpty());   // only wait for idle input if the txBuffer has data

    Timeout timeout(ourTimer0TicksPerSymbol);
    unsigned char available=0xff;
    while (true)
    {
      unsigned char nowAvailable=txBuffer.itemsStored();
      if(nowAvailable == 0)        // nothing left to do
      {
        TRACE(flush_1); // nowStamp2(), txState, nowAvailable
        emptyPipeline();	       // but we got to keep on waiting
		if(T__RS485::txError() )   // waiting for the miracle to come? 
        { // we got a collision while emptying the pipeline
          T__RS485::setTransactionError(T__RS485::FlushPipelineError, T__RS485::txState, txBuffer.head );
          T__RS485::reset();
          break;
        }
        return 1;
      }
      else if(available != nowAvailable) // one more character sent
      {
        TRACE(flush_1); // nowStamp2(), txState, nowAvailable
        available = nowAvailable;
        timeout.set(ourTimer0TicksPerSymbol+4);
      }
      else if (timeout.isExpired() | T__RS485::txError())
      {
        T__RS485::setTransactionError(T__RS485::FlushTimeoutExpired, T__RS485::txState, txBuffer.head );
        break;
      }
    }
  }

  T__RS485::reset(); // be ready for next time
  return 0;
}


#endif // defined(Implement_HardwareSerialRS485) ++=======================================================================================================

#endif // !defined(HardwareSerialRS485_Enabled_h)
