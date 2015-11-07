/* HardwareSerialRS485_Helper.h copyright notice

Helper class to extend HardwareSerial communication over half-duplexed RS485.

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

#if !defined(HardwareSerialRS485_Helper_h)
#define      HardwareSerialRS485_Helper_h

#include <avr/interrupt.h>
extern "C" void delay(unsigned long);

#include <inttypes.h>
#include "utility/BitManipulation.h"
#include "Print.h"
#include "Printable.h"
#include "utility/HardwareSerialRS485_Tracer.h"

#define RS485_showoff 	// make led lights flashing

// See $Arduino/arduino-1.0.5/hardware/arduino/cores/arduino/wiring.c
extern volatile unsigned long timer0_overflow_count; // ticks every 64*256 cycles = 16 x 1024 

/* controlling output pins
DDR<R>  = mask; // activate pins for output:
PORT<R> = mask; // Sets output values, note: setting a disabled output high will enable the internal pull-up register.
where <R> is
D – digital pins  7 to 0 (bank D)
B – digital pins 13 to 8 (bank B)
C – analogue pins 5 to 0 (bank C)
*/

/* TRxControl class doc
 * The TRxControl class is a helper class used by the RS485serial class. This class contains the knowledge of the port and pins
 * that control the TxE and RxE of the 485 driver circuit. This class provides the methods trxStart() and trxTerminate(), which
 * manipulates the 485 enable pins as a function of the selected transfer mode.
 * Various set and clr methods allow to control the 485 communication mode.
 * RS485 receive (Rx) modes: (NB: one cannot read from both RS232 and RS485 at the same time, to be able to read data over the RS232 port, RS485 read should be disabled)
 * - permaRead  set:  485 read is  enabled between transmissions
 * -            clr:  485 read is disabled between transmissions
 * during transmission, the read enable is controlled by the RS485 transmit mode (see hereunder).
 * RS485 transmit mode: (NB: all data written will always be transmitted over RS232, independent of the RS485 Tx mode)
 * - PermaSilence       : Tx is permanently disabled,  Rx lines as define by Rx mode.
 * - PermaWrite         : Tx is permanently enabled,   Rx lines as define by Rx mode.
 * - WriteNoLocalEcho   : Tx is enabled during writes, Rx will be disabled during writes.
 * - WriteLocalEcho     : Tx is enabled during writes, Rx will be enabled during writes (hence, all written characters are echoed back to the input)
 * - WriteDataCheck     : as WriteLocalEcho, however,  the RS485 class will filter and check the characters echoed back for transmission errors and collisions.
 * The most common modes are
 * RS232 mode: PermaSilence   + permaRead-cleared
 * RS485 mode: WriteDataCheck + permaRead-set
 * other modes are useful for debugging only.
 *
 * This class assumes that Tx and Rx enable/disable are controlled from output pins connected to the same port. This choice is made for efficiency.
 * For a good reason and a beer, the author will provide the help to create a class that will suit your needs.
 *
 * Todo use D10 D11 style template parameters, provided we can assert (at compile time!) that they are on the same port.
 */
template<         char T__TRx_PORT    = 'B',    // port B digital pin D8 - D13
         unsigned char T__TxEnable    =  2,     // TxE  (1<<2) : PB2 shield D10, SS,   pin 16, (pwm)
         unsigned char T__RxDisable   =  3      // RxE* (1<<3) : PB3 shield D11, MOSI, pin 17, (pwm)
         >
class TRxControl
{
private:
#pragma push_macro("_MMIO_BYTE")  // was _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#undef  _MMIO_BYTE
#define _MMIO_BYTE
#define Atmega328p_Ports(__p) __p == 'B' ? PORTB : __p == 'C' ? PORTC : __p == 'D' ? PORTD :
#ifdef USBCON
  #define Atmega348u_Ports(__p) __p == 'E' ? PORTE : __p == 'E' ? PORTF :
#else
  #define Atmega348u_Ports(__p)
#endif
//TODO try a cleanup with a consexpr function, i.e. a function that is evaluatd at compile time.
  const static unsigned char TRx_PORT = (Atmega328p_Ports(T__TRx_PORT) Atmega348u_Ports(T__TRx_PORT) PORTB); 
  const static unsigned char DDR      = (DDRB-PORTB) + TRx_PORT; // this works because DDR registers are always located at a fixed offset to the PORT registers
#pragma pop_macro("_MMIO_BYTE")


/* DONE: we cannot declare TRx_PORT and DDR as unsigned char*, or unsigned char&
   const static unsigned char* TRx_PORT = (unsigned char*) (Atmega328p_Ports(T__TRx_PORT) Atmega348u_Ports(T__TRx_PORT) PORTB); 
   error: 'constexpr' needed for in-class initialization of static data member 'const unsigned char* TRxControl<T__TRx_PORT, T__TxEnable, T__RxDisable>::TRx_PORT' of non-integral type [-fpermissive]
 */
//const unsigned char& _TRx_PORT = *(unsigned char*)(Atmega328p_Ports(T__TRx_PORT) Atmega348u_Ports(T__TRx_PORT) PORTB); 
//const unsigned char& _DDR      = *(unsigned char*)(TRx_PORT - (DDRB-PORTB)); 
//const unsigned char& _TRx_PORT = *(unsigned char*) TRx_PORT; // TODO exploit this, dit mag wel


// special feature, passing a ~value will inverts the logic
  const static unsigned char M_TxEnable  = (1<<( T__TxEnable  & 0x0f)) & 0xff;
  const static unsigned char M_TxDisable = (1<<(~T__TxEnable  & 0x0f)) & 0xff;
  const static unsigned char M_RxEnable  = (1<<(~T__RxDisable & 0x0f)) & 0xff;
  const static unsigned char M_RxDisable = (1<<( T__RxDisable & 0x0f)) & 0xff;

  const static unsigned char MASK_TRxValue = (M_TxEnable|M_TxDisable|M_RxEnable|M_RxDisable);

  unsigned char volatile txStart_TRxValue;     // mode dependent rtx value when transmission is started
  unsigned char volatile txTerminate_TRxValue; // mode dependent rtx value when transmission is terminated

  inline void writeTRxValue(unsigned char value) { *(unsigned char *)TRx_PORT = (*(unsigned char *)TRx_PORT & ~MASK_TRxValue) | value; }

public:
  inline unsigned char getTxStart_TRxValue()     { return txStart_TRxValue;     }
  inline unsigned char getTxTerminate_TRxValue() { return txTerminate_TRxValue; }
  inline unsigned char readTRxValue()            { return (*(unsigned char *)TRx_PORT & ~MASK_TRxValue); }

  inline void initialize()
  {
    *(unsigned char *)DDR  = (*(unsigned char *)DDR | MASK_TRxValue);  // set data direction register


    #if defined(RS485_showoff)
    for(int i=0; i<7; i++)
    {
      writeTRxValue(M_TxEnable |M_RxDisable);
      delay(140-14*i);
      writeTRxValue(M_TxDisable|M_RxEnable);
      delay(140+14*i);
    }
    #endif
    set232Mode();
  }

  inline void trxStart()            { writeTRxValue(txStart_TRxValue);  }
  inline void trxTerminate()        { writeTRxValue(txTerminate_TRxValue); }

  inline void setPermaRead()        { txTerminate_TRxValue = (txTerminate_TRxValue & ~M_RxDisable) | M_RxEnable;  }
  inline void clrPermaRead()        { txTerminate_TRxValue = (txTerminate_TRxValue & ~M_RxEnable)  | M_RxDisable; }
  inline void setPermaWrite()       { txStart_TRxValue = txTerminate_TRxValue = (txTerminate_TRxValue & ~M_TxDisable) | M_TxEnable  ; }
  inline void setPermaSilence()     { txStart_TRxValue = txTerminate_TRxValue = (txTerminate_TRxValue & ~M_TxEnable ) | M_TxDisable ; }
  inline void setWriteLocalEcho()   { txStart_TRxValue = M_TxEnable|M_RxEnable;  txTerminate_TRxValue = (txTerminate_TRxValue & ~M_TxEnable ) | M_TxDisable ;}
  inline void setWriteNoLocalEcho() { txStart_TRxValue = M_TxEnable|M_RxDisable; txTerminate_TRxValue = (txTerminate_TRxValue & ~M_TxEnable ) | M_TxDisable ;}

  inline void set485Mode()          { txStart_TRxValue = M_TxEnable|M_RxEnable;  txTerminate_TRxValue = M_TxDisable|M_RxEnable;  writeTRxValue(txTerminate_TRxValue); }
  inline void set232Mode()          { txStart_TRxValue =                         txTerminate_TRxValue = M_TxDisable|M_RxDisable; writeTRxValue(txTerminate_TRxValue); }

  inline bool isPermaRead()         { return (txTerminate_TRxValue & (M_RxEnable|M_RxDisable)) == M_RxEnable;  }
  inline bool isPermaWrite()        { return (txTerminate_TRxValue & (M_TxEnable|M_TxDisable)) == M_TxEnable;  }
  inline bool isPermaSilence()      { return (txStart_TRxValue     & (M_TxEnable|M_TxDisable)) == M_TxDisable; }
  inline bool isWriteLocalEcho()    { return (txStart_TRxValue == (M_TxEnable|M_RxEnable)  ); }
  inline bool isWriteNoLocalEcho()  { return (txStart_TRxValue == (M_TxEnable|M_RxDisable) ); }

}; // class TRxControl


/* General purpose function to get raw time stamps (in 4 us units)
 * The following functions return the raw time stamp (in units of 4 us, with a maximum range of 4:46 hours). The usage of such
 * 'raw' time stamp encompasses less overhead and is faster.
 * This function manipulate 4 byte objects that must be done atomically to exclude spurious errors in the time stamp. For this
 * purpose two versions are provided, an interrupt safe version: nowStamp(), and an unprotected version nowStamp_IC() that can
 * be used from within an interrupt context. The ATMEGAdoc section 6.3.1 specifies that during the execution of an interrupt, the
 * interrupt enable (the I bit in the SREG status register) is cleared and the interrupt can not be interrupted by an other
 * interrupt, unless the interrupt function explicitly set the I bit in the SREG register. 
 * Hence interrupt functions can safely use the nowStamp_IC(), while functions that can run in non interrupt context should use the
 * interrupt safe version: nowStamp().
 */
inline unsigned long nowStamp_IC()
{
// inspired by micros(), see $Arduino/hardware/arduino/avr/cores/arduino/wiring.c
	unsigned long m = timer0_overflow_count;

#if     defined(TCNT0) 
#elif defined(TCNT0L)
	#define  TCNT0 TCNT0L
#else
	#error TIMER 0 not defined
#endif

#if !defined(TIFR0)
	#define  TIFR0 TIFR
#endif

	unsigned char t = TCNT0;
	if (testBit(TIFR0, TOV0) && (t != 255)) m++; // if the timer overflow interrupt has not yet been handled we have to correct timer0_overflow_count for this.
	return (m << 8) | t;
}

inline unsigned long nowStamp()       // return 'interrupt safe' raw time stamp 
{
  unsigned char oldSREG = SREG;                 // save interrupt state
  cli();                                        // disable interrupts
  unsigned long now = nowStamp_IC();
  SREG = oldSREG;                               // restore interrupt state
  return now;
}


class Transaction_dummy // ===========================================================================================================================
{
    public:
    const static unsigned char TransactionActive      =0x01;
    const static unsigned char WriteBufferFullTimeout =0x11;
    const static unsigned char FlushPipelineError     =0x12;
    const static unsigned char FlushTimeoutExpired    =0x14;
    const static unsigned char NoRecoveryError        =0x30;

    inline Transaction_dummy()                                                                                          {}
    inline void startTransaction()                                                                                      {}
    inline void setTransactionError(unsigned char theErrorType, unsigned char txState, unsigned char currentPosition)   { (void) theErrorType; (void) txState; (void) currentPosition; }
    inline unsigned char endTransaction()                                                                               { return 0; }
    inline bool isTransactionActive()                                                                                   { return false; }
}; // Transactiond_dummy =============================================================================================================================


class Transaction : public Transaction_dummy // ======================================================================================================
{
// records the transaction status
    public:

    unsigned char errorCount;
    unsigned char statusMask;
    unsigned char errorCode;
    unsigned char errorPosition;
    // todo add bytecount? to replace also buffer-wrapped (fixes feature of wrap detection for retry)

  public:
    inline Transaction()
    {
        errorCount=0;
        statusMask=0;
        errorCode=0;
        errorPosition=0;
    }
    inline void startTransaction()
    {
        errorCount=0;
        statusMask=TransactionActive;
        errorCode=0;
        errorPosition=0;
    }
    inline void setTransactionError(unsigned char theErrorType, unsigned char txState, unsigned char currentPosition)
    {
        errorCount++;
        statusMask   |=theErrorType;
        errorCode     =txState;
        errorPosition =currentPosition;
    }
    inline unsigned char endTransaction()
    {
        statusMask &= ~TransactionActive;
        return errorCode;
    }
    inline bool isTransactionActive()
    {
        return testBitsAny(statusMask, TransactionActive);
    }
    Transaction& getTransaction()
    {
      return *this;
    }

}; // Transaction ====================================================================================================================================


template<class T__TRxControl=TRxControl<>, class T__Transaction=Transaction >
class RS485serial : public T__Transaction // =========================================================================================================
{

  T__TRxControl trxControl;

protected:
  unsigned char txState;
  unsigned char inHwShiftRegister;   // data in shiftout register
  unsigned char inTxDataRegister;    // data in data out register
  unsigned long minimumIdleTime;     // time that input should be idle before transmitting */
  unsigned long volatile rxTime;     // time of last data reception

  inline bool txStateNoDataCheck()   {return testBitsAny(txState,0x80); }
  inline void setMinimumIdleTime(unsigned long mit) { minimumIdleTime = mit; }


/* txState bit fields and transitions
   bit field 0x03  transmission state
           = 0x00    no transfer / waiting for Rx idle
           = 0x01    transfer finished, not yet acknowledged
           = 0x02    data in shift register
           = 0x03    data in shift and data register
   bit field 0x0C  transmission status
           = 0x00     no errors
           = 0x04     pipeline under/overrun condition
           = 0x08     data  error
           = 0x0c     frame error
   bit field 0x10   transmission busy
   bit field 0x20   reserved
   bit field 0x40   reserved
   bit field 0x80   no data check

state transitions for txState
  txStart()      : called from DRE interrupt to set 485 transmission enable. gives go/no-go for transmission and enables the RS485 circuitry if needed
                 - input not idle: {0x00, 0x10}->0x10
                 - first  character put in pipeline: {0x00, 0x10}->0x12
                 - second character put in pipeline: 0x12->0x13
                 - pipeline overflow: 0x13->0x17                 (internal bug or HW fault)
                 - data after terminated transmission 0x11->0x15 (internal bug or HW fault)?
  txSendChar()   : called from DRE interrupt to informs RS485 class on what character can be expected
                   none
  isEcho()       : called from RXC interrupt. Returns true when the character was expected from local echo.
                 - rx still busy: {0x00, 0x10}->0x10,
                 - last character out/mismatch/frame-error: 0x12 -> { 0x11 / 0x1a / 0x1e }
                 - next character out/mismatch/frame-error: 0x13 -> { 0x12 / 0x1b / 0x1f }
                 - pipeline underflow: 0x11->0x1d  (internal bug or HW fault)
  txCompleted()  : called from TX completed interrupt when all data has gone out, disables tx and updates the state
                 - termination: {0x10..0x1f}->={0x00..0x0f}, exception: 0x10->0x06, 0x12 -> 0x08, 0x13 -> 0x09
user transitions
  ()              *->0x00
  begin()           
  txReady()	 test if txState € {0x10, 0x11, 0x12, 0x13 or 0x*0}
  - write()
  setTxMode()

combined:

Transfer active states:
0x10	n.u.
0x11	transient: no data (all data read, before tx-completion interrupt)
0x12	transient: data in shift register
0x13	transient: data in shift and data register
0x14	n.u.
0x15	error: Tx data following no data (all data read, before tx-completion interrupt)
0x16	n.u.
0x17	error: pipeline overflow
0x18	error: new data before data transfer finished, unlikely but not impossible
0x19	n.u.
0x1a	error: data mismatch last data item
0x1b	error: data mismatch data item
0x1c	error: we missed out on a local echo
0x1d	error: pipeline underflow
0x1e	error: frame error last data item
0x1f	error: frame error data item


Transfer completed states:	
0x00	kicked in: start state for data transfer (DRE not yet executed)
0x01	normal completion
0x02	n.u.
0x03	n.u
0x04	n.u. // option: abnormal completion while not busy
0x05	completed with error: Tx data following no data (all data read, before tx-completion interrupt)
0x06.	abnormal completion while waiting for Rx idle               0x10 -> 0x06    if((txState^0x12)<=0x02) txState += (-0x10+0x06);
0x07	completed with error: while pipeline overflow
0x08.	abnormal completion with data in shift register				0x12 -> 0x08    if((txState^0x12)<=0x02) txState += (-0x10+0x06);
0x09.	abnormal completion with data in shift and data register	0x13 -> 0x09    if((txState^0x12)<=0x02) txState += (-0x10+0x06);
0x0a	completed with error: data mismatch last data item
0x0b	completed with error: data mismatch data item
0x0c	n.u.
0x0d	completed with error: pipeline underflow
0x0e	completed with error: frame error last data item
0x0f	completed with error: frame error data item

*/

public:

  RS485serial()
  {
    // RS485 has already been deactivated by the constructor of TRxControl.
  }

  inline unsigned long timeSinceRX_IC()     // Returns time since last rx, not counting local echo's. called from interrupt context (txStart).
  {
    return nowStamp_IC() -rxTime;
  }

  inline unsigned long timeSinceRX()        // Returns time since last rx, not counting local echo's. Called from non-interrupt context (txInputIdleWait).
  { 
    unsigned char savedSREG = SREG;                 // save interrupt state
    cli();                                          // disable interrupts
    unsigned long timeSince = timeSinceRX_IC();
    SREG = savedSREG;                               // restore interrupt state
    return timeSince;
  }

  protected:
  // Called from the write method, returns true if an error occurred.	Also called from the Flush method
  inline bool txError()  { return testBitsAny(txState,0x0C); }

  // Called from the flush method, and the write method for buffer-full condition, checks if the input was idle or not.
  inline bool txInputIdleWait()
  {
    while (true)
    { // refresh the input idle time in the loop as new data may have come in.
      unsigned long timeSince = timeSinceRX();          // note, do not be 'smart' and evaluate diff = minimumIdleTime - timeSince; this will cause problems when timeSinceRX() >= 0x8000

      if( timeSince >= minimumIdleTime ) return true;   // note minimumIdleTime = 0...prio*0x3400, this would fit in a int (also timeSinceRX(), if we would limit priority to 19,  or priority to 32 and baud >= 1200)
	  // TODO, we can get stuck here if someone hugs the line. Alternative keep a global time-out such that we may escape...
    }
  }

  inline void reset()   { clrBits(txState, 0x1F);   }   // Clears error and dynamic fields in txState, not special mode bits

  inline void begin()   { trxControl.initialize(); txState = 0x80;}

  inline void end()     { begin(); }

  // Interrupt service routines txStart (dreInt), txSendChar (dreInt), isEcho (rxcInt), txCompleted (TX completed)
  inline bool txStart() //called from DRE interrupt to set 485 transmission enable. gives go/no-go for transmission and enables the RS485 circuitry if needed, updates txState.
  {
    TRACE(txStart); // nowStamp2(), txState, timeSinceRX_IC()

    if( txStateNoDataCheck() )   // noDataCheck: this implies some unconditional or permanent / enable or disable mode.
    {
      trxControl.trxStart();     // this will set the correct mode (some of the special modes are dynamic)
	  txState |= 0x10;           // set busy bit (in case it was not yet set...
      return true;
    }

	// note, the following test on the conditions are ordered by the expected frequency of the conditions.
    if (txState == 0x12) // active data in output shift register
    {
      txState = 0x13;    // we now have also data pending in the data register
      return true;
    }

    if (txState == 0x00) // waiting to start
    {
      if( timeSinceRX_IC() < minimumIdleTime) return false; // input still busy

	  txState=0x12;         // we will now have active data in output shift register
      trxControl.trxStart(); // so we allow transmission, enable the RS485-tx
      return true;      
    }
    if ((txState|0x02) == 0x13 ) // 0x11: new data before data transfer finished, software bug
	                             // 0x13: we missed out on a local echo
    {
      trxControl.trxTerminate();
	  txState = 2*(txState-5); // 0x11, 0x13 mapped to 0x18, 0x1c
      return false;
    }

    return false; // we are in an error or stopped state
  } // txStart()

//called from dreInt() - Data Register Empty interrupt - after a character was copied to the DR. Keeps track of what is expected on the local echo
  inline void txSendChar(unsigned char c)
  {
    if(     txState == 0x12) inHwShiftRegister=c;
    else if(txState == 0x13) inTxDataRegister =c;
  }

//called from rxcInt() - RXC interrupt. Allows to remove expected data, also detects collisions or unexpected extra characters, updates txState
  inline bool isEcho(unsigned char c, bool error)
  {
    TRACE(echo); // nowStamp2(), txState, c, error

    if(false) { /* lekker puh */ }
    else if(  txState         == 0x00)  { /* nothing */ } // we are waiting (or not) for Rx idle and [sigh] we just received another byte
    else if( (txState | 0x01) == 0x13)  // txState € {0x12, 0x13} we are expecting data echoed back
    {
      if(error)                     txState|=0x0c; // frame error, mark the state               // fix txState+=0x0c;
      else if(inHwShiftRegister!=c) txState|=0x08; // not the data we expected, mark the state  // fix txState+=0x0a;
      else
      { // we got the data we expected
        txState--;
        inHwShiftRegister=inTxDataRegister;
        return true; // it is an echo, absorb the character.
      }
      // we have a collision, abort any ongoing transmission on the 485
      trxControl.trxTerminate();
    }
    else if( txState == 0x11) txState = 0x1d;   // implausible if not impossible: we receive data after the last data item sent was acknowledged, but before transmission was terminated. // fix txState=0x0b;
    //se if( txState == 0x00) { /* nothing */ }; // we are waiting for Rx idle and [sigh] we just received another byte
    //se if( txState == 0x10) { /* nothing */ }; // we are waiting for Rx idle and [sigh] we just received another byte

    TRACE(echo_error); // txState
    return false;
  }

  inline void txCompleted() // called from TX completed interrupt when all data has gone out, disable tx and updates the txState
  {
    TRACE(txCompleted); // nowStamp2(), txState

    trxControl.trxTerminate(); // first things first

    if      ((txState^0x12)<=0x02) txState += -0x10 +6;    // abnormal completion during transient states: 0x10, 0x12 and 0x13 are mapped into error space: 0x06, 0x08 and 0x09
//  else if ((txState&0x10)==0x00) txState  =  0x04;       // not busy, completed again
    else                           txState &= ~0x10;       // clear busy flag
  }

public:
  static const unsigned char RS323mode = 0x25;
  static const unsigned char RS485mode = 0x11;

  inline void setMode(unsigned char mode)
  {
/*  mode -- 
    0x0f: write mode mask
    0x01: write with dataCheck
    0x02: write with local echo
    0x03: write no   local echo
    0x04: permanent write
    0x05: permanent silence
    other: no change

    0xf0: read mode mask:
    0x10: enable read
    0x20: disable read
    other: no change
    
    combined:
    RS485: 0x11
    RS232: 0x05
*/
    if(      (mode&0xf0)==0x10 ) trxControl.setPermaRead();
    else if( (mode&0xf0)==0x20 ) trxControl.clrPermaRead();

	mode &= 0x0f;
	if(mode!=0)
    {
      if      (mode==0x01) trxControl.setWriteLocalEcho(); // write with dataCheck
	  else if (mode==0x02) trxControl.setWriteLocalEcho();
      else if (mode==0x03) trxControl.setWriteNoLocalEcho();
      else if (mode==0x04) trxControl.setPermaWrite();
      else if (mode==0x05) trxControl.setPermaSilence();

	  // txState = (mode==0x01) ? (txState & ~0x80) : (txState | 0x80);
	  txState = (mode==0x01) ? 0 : 0x80;
    }

    trxControl.trxTerminate(); // force the selected mode (we rely on no tx being in progress!)
  }

  unsigned char getMode()
  {
    unsigned char mode;

	// evaluate write mode:
    if (false)                                  mode = 0x00; // lekker puh
    else if ( trxControl.isPermaSilence()     ) mode = 0x05;
    else if ( trxControl.isPermaWrite()       ) mode = 0x04;
    else if ( trxControl.isWriteNoLocalEcho() ) mode = 0x03;
    else if ( (txState & 0x80) != 0           ) mode = 0x02;
    else                                        mode = 0x01;

    // add read mode:
    mode |= ( trxControl.isPermaRead() ) ? 0x10 : 0x20;

    return mode;
  }

  inline void          set232Mode() { trxControl.set232Mode(); }
  inline void          set485Mode() { trxControl.set485Mode(); }
}; // class RS485serial ==============================================================================================================================

#endif // !defined(HardwareSerialRS485_Helper_h)
