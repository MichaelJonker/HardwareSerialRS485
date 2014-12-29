/* HardwareSerial_RS485Helper.h copyright notice

Helper class to extend HardwareSerial communication over half-duplexed RS485, and providing message filter capabilities.

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

#if !defined(HardwareSerial_RS485Helper_h)
#define      HardwareSerial_RS485Helper_h

#include "arduino.h"
#include <inttypes.h>
#include "BitManipulation.h"
#include "Print.h"

extern volatile unsigned long timer0_overflow_count; // ticks every 64*256 cycles = 16 x 1024 

/* controlling output pins
DDR<R>  = mask; // activate pins for output:
PORT<R> = mask; // Sets output values, note: setting a disabled output high will enable the internal pull-up register.
where <R> is
D – digital pins seven to zero (bank D)
B – digital pins thirteen to eight (bank B)
C – analogue pins five to zero (bank … C!)
*/

/* TRxControl class doc
 * The TRxControl class is a helper class used by the RS485serial class. This class contains the knowledge of the port and pins
 * that control the TxE and RxE of the 485 driver circuit. This class provides the methods txStart() and txTerminate(), which
 * manipulates the 485 enable pins as a function of the selected transfer mode.
 * Various set and clr methods allow to control the 485 communication mode.
 * RS485 receive (Rx) modes: (NB: one cannot read from both RS232 and RS485 at the same time, to be able to read data over the RS232 port, RS485 read should be disabled)
 * - permaRead  set:  485 read is always active between transmissions
 * -            clr:  485 read is disabled between transmissions
 * during transmission, the read enable is controlled by the transmission mode (see below).
 * RS485 transmit mode: (NB: all data written will always be transmitted over RS232, independent of the RS485 Tx mode)
 * - PermaSilence       : Tx is permanently disabled,  Rx lines as define by Rx mode.
 * - WriteNoLocalEcho   : Tx is enabled during writes, Rx will be disabled during writes.
 * - WriteLocalEcho     : Tx is enabled during writes, Rx will be enabled during writes (hence, all written characters are echoed back to the input)
 * - WriteDataCheck     : as WriteLocalEcho, however,  the RS485 class will filter and check the characters echoed back for transmission errors and collisions.
 * - PermaWrite         : Tx is permanently enabled,   Rx lines as define by Rx mode.
 * The most common modes are
 * RS232 mode: PermaSilence   + permaRead-cleared
 * RS485 mode: WriteDataCheck + permaRead-set
 * other modes are useful for debugging only.
 *
 * This class assumes that Tx and Rx enable/disable are controlled from output pins connected to the same port. This choice is made for efficiency.
 * For a good reason and a beer, the author will provide the help to create a class that will suit your needs.
 *
 */

//TODO use D10 D11 style template parameters, provided we can assert (at compile time) that they are on the same port.
template<         char T__TRx_PORT    = 'B',    // port B digital pin D8 - D13
         unsigned char T__TxEnable    =  2,     // TxE  (1<<2) : PB2 shield D10, SS,   pin 16, (pwm)
         unsigned char T__RxDisable   =  3      // RxE* (1<<3) : PB3 shield D11, MOSI, pin 17, (pwm)
         >
class TRxControl
{
private:
#undef  _MMIO_BYTE
#define _MMIO_BYTE
  const static unsigned char TRx_PORT = T__TRx_PORT == 'B' ? PORTB : T__TRx_PORT == 'C' ? PORTC : T__TRx_PORT == 'D' ? PORTD : PORTB; 
  const static unsigned char DDR      = T__TRx_PORT == 'B' ? DDRB  : T__TRx_PORT == 'C' ? DDRC  : T__TRx_PORT == 'D' ? DDRD  : DDRB ; 
#undef _MMIO_BYTE
#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))

// undocumented feature, passing a ~value will invert the logic
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

  inline void initialize()
  {
    *(unsigned char *)DDR  = (*(unsigned char *)DDR | MASK_TRxValue);

    for(int i=0; i<7; i++)
    {
      writeTRxValue(M_TxEnable |M_RxDisable);
      delay(140);
      writeTRxValue(M_TxDisable|M_RxEnable);
      delay(140);
    }
    set232Mode(); txTerminate();
  }

  inline void txStart()             { writeTRxValue(txStart_TRxValue);  }
  inline void txTerminate()         { writeTRxValue(txTerminate_TRxValue); }

  inline void setPermaRead()        { txTerminate_TRxValue = (txTerminate_TRxValue & ~M_RxDisable) | M_RxEnable;  }
  inline void clrPermaRead()        { txTerminate_TRxValue = (txTerminate_TRxValue & ~M_RxEnable)  | M_RxDisable; }
  inline void setPermaWrite()       { txStart_TRxValue = txTerminate_TRxValue = (txTerminate_TRxValue & ~M_TxDisable) | M_TxEnable  ; }
  inline void setPermaSilence()     { txStart_TRxValue = txTerminate_TRxValue = (txTerminate_TRxValue & ~M_TxEnable ) | M_TxDisable ; }
  inline void setWriteLocalEcho()   { txStart_TRxValue = M_TxEnable|M_RxEnable;  txTerminate_TRxValue = (txTerminate_TRxValue & ~M_TxEnable ) | M_TxDisable ;}
  inline void setWriteNoLocalEcho() { txStart_TRxValue = M_TxEnable|M_RxDisable; txTerminate_TRxValue = (txTerminate_TRxValue & ~M_TxEnable ) | M_TxDisable ;}

  inline void set485Mode()          { txStart_TRxValue = M_TxEnable|M_RxEnable;  txTerminate_TRxValue = M_TxDisable | M_RxEnable;  }
  inline void set232Mode()          { txStart_TRxValue = txTerminate_TRxValue = M_TxDisable|M_RxDisable; }

  inline bool isPermaRead()         { return (txTerminate_TRxValue & (M_RxEnable|M_RxDisable)) == M_RxEnable;  }
  inline bool isPermaWrite()        { return (txTerminate_TRxValue & (M_TxEnable|M_TxDisable)) == M_TxEnable;  }
  inline bool isPermaSilence()      { return (txStart_TRxValue     & (M_TxEnable|M_TxDisable)) == M_TxDisable; }
  inline bool isWriteLocalEcho()    { return (txStart_TRxValue == (M_TxEnable|M_RxEnable) );  }
  inline bool isWriteNoLocalEcho()  { return (txStart_TRxValue == (M_TxEnable|M_RxDisable) ); }
}; // class TRxControl


#define RS485dbg
template<class T_TRxControl=TRxControl<> > class RS485serial
{
// The 485 has states which are stored in the instance and not as class variables (contrary to HWSerial).
// The use of objects is unavoidable for calling virtual methods that are implemented by a sub class...
// Virtual methods only work for objects, i.e. a class by itself does not know which subclass has extended the objects of its class.

  T_TRxControl trxControl;

protected:
  unsigned char txState;
  unsigned char inHwShiftRegister;   // data in shiftout register
  unsigned char inTxDataRegister;    // data in data out register
  unsigned char minimumIdleTime;     // time that input should be idle before transmitting */
  unsigned long volatile lastRxTime; // last time of data reception

  //inline bool txStateIsError()      {return (txState & 0x0c)!=0; }
  //inline bool txStateIsBusy()       {return (txState & 0x10)!=0; }
  //inline bool txStateIsCompleted()  {return (txState & 0x10)==0; }
  inline bool txStateIsSpecial()    {return (txState & 0xE0)!=0; }
  inline void setMinimumIdleTime(unsigned char mit) { minimumIdleTime = mit; }


/* txState fields
   bit field 0x03  transmission state
           = 0x00	 waiting for Rx idle
           = 0x01	 no data (transient mode)
           = 0x02	 data in shift register
           = 0x03	 data in shift and data register
   bit field 0x0C	transmission status
           = 0x00     no errors
           = 0x04     pipeline under/overrun condition
           = 0x08     frame error
           = 0x0c     data  error
   bit field 0x10   transmission busy
   bit field 0x20   reserved
   bit field 0x40   reserved
   bit field 0x80   no data check

state transitions for txState
  txStart()      - input not idle: {0x00, 0x10}->0x10
                 - first  character put in pipeline: {0x00, 0x10}->0x12
                 - second character put in pipeline: 0x12->0x13
                 - pipeline overflow: 0x13->0x17                 (internal bug or HW fault)
                 - data after terminated transmission 0x11->0x15 (internal bug or HW fault)?
  txSend()       none
  isNotEcho()    - rx still busy: {0x00, 0x10}->0x10,
                 - last character out/frame-error/mismatch: 0x12 -> { 0x11 / 0x1a / 0x1e }
                 - next character out/frame-error/mismatch: 0x13 -> { 0x12 / 0x1b / 0x1f }
                 - pipeline underflow: 0x11->0x1d  (internal bug or HW fault)
  txCompleted()  - termination: {0x10..0x1f}->={0x00..0x0f}, exception: 0x10->0x06, 0x12 -> 0x08, 0x13 -> 0x09
user transitions
  ()              *->0x00
  begin()           
  retryMessage()  *->0x00
  txReady()	 test if txState € {0x10, 0x11, 0x12, 0x13 or 0x*0}
  - write()
  setTxMode()

combined:

Transfer active states:
0x10	transient: waiting for Rx idle
0x11	transient: no data (all data read, before tx-completion interrupt)
0x12	transient: data in shift register
0x13	transient: data in shift and data register
0x14	n.u.
0x15	error: Tx data following no data (all data read, before tx-completion interrupt)
0x16	n.u.
0x17	error: pipeline overflow
0x18	error: new data before data transfer finished, unlikely but not impossible
0x19	n.u.
0x1a	error: frame error last data item
0x1b	error: frame error data item
0x1c	error: we missed out on a local echo
0x1d	error: pipeline underflow
0x1e	error: data mismatch last data item
0x1f	error: data mismatch data item

Transfer completed states:	
0x00	kicked in: start state for data transfer (DRE not yet executed)
0x01	normal completion
0x02	n.u.
0x03	n.u
0x04	n.u. // option: abnormal completion while not busy
0x05	completed with error: Tx data following no data (all data read, before tx-completion interrupt)
0x06	abnormal completion while waiting for Rx idle               0x10 -> 0x06    if((txState^0x12)<=0x02) txState += (-0x10+0x06);
0x07	completed with error: while pipeline overflow
0x08	abnormal completion with data in shift register				0x12 -> 0x08    if((txState^0x12)<=0x02) txState += (-0x10+0x06);
0x09	abnormal completion with data in shift and data register	0x13 -> 0x09    if((txState^0x12)<=0x02) txState += (-0x10+0x06);
0x0a	completed with error: frame error last data item
0x0b	completed with error: frame error data item
0x0c	n.u.
0x0d	completed with error: pipeline underflow
0x0e	completed with error: data mismatch last data item
0x0f	completed with error: data mismatch data item

*/


#ifdef RS485dbg // use template parameter for debug object ?
  unsigned char dbgTrace;
  unsigned char dbgtxReady;
  unsigned char dbgtxCompleted;
  unsigned char dbgisNotEcho;
  unsigned long dbgLastTxStartTime;
  unsigned long dbgLastInputIdleWaitTime;
#endif // RS485dbg

public:

  inline unsigned int timeSinceRX()  // Returns time since last rx, not counting local echo's.
  {
    unsigned char oldSREG = SREG;   // save interrupt state
    cli();                          // disable interrupts
    unsigned long diff = timer0_overflow_count - lastRxTime;
#if 1
/* code disabled */
#elif defined(TCNT0)
    unsigned char dt = TCNT0;
#elif defined(TCNT0L)
    unsigned char dt = TCNT0L;
#else
    #error TIMER 0 not defined
#endif
    SREG = oldSREG;                 // restore interrupt state
    if((unsigned int)(diff>>16)) return 0xffff;  // optimized, but not yet fully optimized!! (can do better with __asm__)
    return (unsigned int) diff;
  }

  RS485serial()
  {
    // RS485 has already been deactivated by the constructor of TRxControl.
    minimumIdleTime=10;
  }

protected:
  // Called from the write method, returns true if the DRE interrupt can be enabled.	Also called from the Flush method
  inline bool txReady()  { return ((txState&~0x03)==0x10 or (txState&0x0f)==0x00); } // txState € {0x10, 0x11, 0x12, 0x13 or 0x*0} busy and no error bits set, or armed to go (also in special modes)

  // Called from the write method, returns true if an error occurred.	Also called from the Flush method
  inline bool txError()  { return ((txState&0x0C)!=0x00 ); }

  // Called from the write method for buffer-full conditions, checks if the input was idle or not.
  inline bool txInputIdleWait()
  { // check if the message has not been started due to bus contingency. If so wait until the bus is idle again.
    if(txState!=0x10) return false; // no need not wait
    // TODO keep a global n-second time-out or count limit such that we cannot get stuck here if someone hugs the line, we should disable 485 in that case

    #ifdef RS485dbg
      dbgTrace = (dbgTrace<<2) | 0x1;
	  dbgLastInputIdleWaitTime = millis();
    #endif // RS485dbg

    while (true)
    { // check the idle time in a loop as new data may have come in.
      unsigned int timeSince = timeSinceRX();   //note, do not be 'smart' and evaluate diff = minimumIdleTime - timeSince; this will cause problems when timeSinceRX() >= 0x8000
      if( timeSince >= minimumIdleTime ) return true;
      delay( minimumIdleTime - timeSince );
    }
  }

  inline void enableStart() { if(txState==1) txState=0; }   // Changes txState to 0x00 (if previous transmission ended properly)
  inline void reset()       { txState &= ~0x1F; }           // Clears error and dynamic fields in txState, not special mode bits //TODO use bitfield functions  and define special mode bit-mask

//todo check possible hangup scenario: no rearm, fill txBuffer, rearm, send one more character. ok if rearm also kicks
//todo check possible hangup scenario: input not idle, fill txBuffer, send one more character, because who kicks...
//TODO in that case we should add a delay for input idle and kick.


//called from dreInt() - Data Register Empty interrupt - and there is data to be sent. Gives go/no-go for transmission, enables the RS485 circuitry if needed, updates txState.
  inline bool txStart()
  {
    #ifdef RS485dbg
      dbgTrace = (dbgTrace<<2) | 0x2;
	  dbgtxReady = txState;
	  dbgLastTxStartTime = millis();
    #endif // RS485dbg
    if( txStateIsSpecial() )    // special: unconditional or permanent / enable or disable mode.
    {
      trxControl.txStart();     // this will set the correct mode (some of the special modes are dynamic)
	  txState |= 0x10;          // set busy bit (in case it was not yet set...
      return true;
    }

	// note, the following test on the conditions are ordered by the expected frequency of the conditions.
    if (txState == 0x12) // active data in output shift register
    {
      txState = 0x13;    // we now have also data pending in the data register
      return true;
    }

    if (txState == 0x00) txState = 0x10; // intention to send, set busy (by intention) bit. NB the actual HW busy can be derived from the TxEnable bit if needed.

    if (txState == 0x10) // waiting to start
    {
      if( timeSinceRX() < minimumIdleTime) return false; // input still busy

	  txState=0x12;         // we will now have active data in output shift register
      trxControl.txStart(); // so we allow transmission, enable the RS485-tx
      return true;      
    }
    if ((txState|0x02) == 0x13 ) // 0x11: new data before data transfer finished, unlikely but not impossible
	                             // 0x13: we missed out on a local echo
    {
      trxControl.txTerminate();
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

//called from rxcInt() - RXC interrupt. Allows to remove expected data, also detects collisions or unexpected extra charters, updates txState
  inline bool isEcho(unsigned char c, bool error)
  {
    #ifdef RS485dbg
      dbgTrace = (dbgTrace<<2) | 0x3;
	  dbgisNotEcho = txState;
    #endif // RS485dbg

//    if( (txState | 0x01) == 0x13)   // txState € {0x12, 0x13} we are expecting data echoed back
    if( txState == 0x12 or txState == 0x13 )   // txState € {0x12, 0x13} we are expecting data echoed back
    {
      if(error)                     txState|=0x0c; // frame error, mark the state
      else if(inHwShiftRegister!=c) txState|=0x08; // not the data we expected, mark the state
      else
      { // we got the data we expected
        txState--;
        inHwShiftRegister=inTxDataRegister;
        return true; // it is an echo, absorb the character.
      }
      // we have a collision, abort any ongoing transmission on the 485
      trxControl.txTerminate();
    }
    else if( txState == 0x11) txState = 0x1d;   // implausible if not impossible: we receive data after the last data item sent was acknowledged, but before transmission was terminated.
    //se if( txState == 0x00) { /* nothing */ }; // we are waiting for Rx idle and [sigh] we just received another byte
    //se if( txState == 0x10) { /* nothing */ }; // we are waiting for Rx idle and [sigh] we just received another byte

    // note: lastRxTime is updated in HardwareSerial_RS485Enabled.h

    return false;
  }

//called from TXC interrupt when all data has gone out, clear txEnable, updates txState.
  inline void txCompleted()
  {
    #ifdef RS485dbg
      dbgTrace = (dbgTrace<<2) | 0x0;
	  dbgtxCompleted = txState;
    #endif // RS485dbg

    trxControl.txTerminate(); // first things first

    if      ((txState^0x12)<=0x02) txState += -0x10 +6;    // abnormal completion during transient states: 0x10, 0x12 and 0x13 are mapped into error space: 0x06, 0x08 and 0x09
//  else if ((txState&0x10)==0x00) txState  =  0x04;       // not busy, completed again
    else                           txState &= ~0x10;       // clear busy flag
  }

  inline void begin()   { trxControl.initialize(); txState = 0x80;}

  inline void end()     { begin(); }

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
    trxControl.txTerminate(); // force the selected mode (we rely on no tx being in progress!)
  }

  unsigned char getMode()
  {
    unsigned char mode=0x80;

	// evaluate write mode:
    if (false) { /* lekker puh */ }
    else if ( trxControl.isPermaSilence()     ) mode = 0x05;
    else if ( trxControl.isPermaWrite()       ) mode = 0x04;
    else if ( trxControl.isWriteNoLocalEcho() ) mode = 0x03;
    else if ( (txState & 0x80) != 0           ) mode = 0x02;
    else                                        mode = 0x01;

    // add read mode:
    mode |= ( trxControl.isPermaRead() ) ? 0x10 : 0x20;

    return mode;
  }

//  using Print::write; // pull in write(str) and write(buf, size) from Print
  void diagnoseHelper(Print &out)
  {
    #ifdef RS485dbg
//  get status information on 485: 16 bytes*/
    unsigned long saved_millis                   = millis();
    unsigned long saved_lastRxTime               = lastRxTime;
    unsigned long saved_dbgLastTxStartTime	     = dbgLastTxStartTime;
    unsigned long saved_dbgLastInputIdleWaitTime = dbgLastInputIdleWaitTime;
    unsigned char saved_txState  		         = txState;
    unsigned char saved_dbgTrace                 = dbgTrace;
    unsigned char saved_dbgtxReady	             = dbgtxReady;
    unsigned char saved_dbgtxCompleted1          = dbgtxCompleted;
    #define       saved_dbgtxCompleted2            (((saved_dbgtxCompleted1^0x12)<=0x02)? saved_dbgtxCompleted1 -0x10 +6 : saved_dbgtxCompleted1)
    unsigned char saved_dbgisNotEcho             = dbgisNotEcho;
    unsigned char saved_inHwShiftRegister        = inHwShiftRegister;
    unsigned char saved_inTxDataRegister         = inTxDataRegister;
    unsigned char saved_getMode                  = getMode();
    //unsigned char saved_txStart_TRxValue         = trxControl.getTxStart_TRxValue();
    //unsigned char saved_txTerminate_TRxValue     = trxControl.getTxTerminate_TRxValue();


    setMode(RS323mode); // disable 485

#define outradx(__ITEM__, radix) {out.print( F( #__ITEM__ "=0n") ); out.println(saved_##__ITEM__, radix); }
#define outhexa(__ITEM__) {out.print( F( #__ITEM__ "=0x") ); out.println(saved_##__ITEM__,HEX); }
#define outlong(__ITEM__) {out.print( F( #__ITEM__ "="  ) ); out.println(saved_##__ITEM__); }
#define outtime(__ITEM__) {out.print( F( #__ITEM__ "="  ) ); out.print(saved_##__ITEM__); out.print(F(" (" )); out.print((signed long)(saved_##__ITEM__ - saved_millis)); out.println(F(")"));}
#define outchar(__ITEM__) {out.print( F( #__ITEM__ "='" ) ); out.write(saved_##__ITEM__); out.println( F("'" )); }

    out.println("\nDiagnostics dump:");

    outhexa(txState);
    outradx(dbgTrace, 4);
    outhexa(dbgtxReady);
    outhexa(dbgtxCompleted1);
    outhexa(dbgtxCompleted2);
    outhexa(dbgisNotEcho);
    outlong(millis);
    outtime(lastRxTime);
    outtime(dbgLastTxStartTime);
    outtime(dbgLastInputIdleWaitTime);
    outchar(inHwShiftRegister);
    outchar(inTxDataRegister);
    outhexa(getMode);
    //outhexa(txStart_TRxValue);
    //outhexa(txTerminate_TRxValue);
    #endif // RS485dbg
  }


  int checkTxStatus(bool reset)  // deprecated !!
  {
    int ret = txState;
    if(reset) txState =0x00; // rearm
	return ret;
  }

};  // class RS485serial


class MessageFilterRT
{
  char**        myAddressList;
  char          filterState;
  unsigned char addressActiveMask;

//todo
//static char*  blockData[]   = {0};
//static char** passAllData = 0;
//static char*  matchAllMessages[]          = {(char*)&matchAllMessages[1], 0}; // aka = {(char*)"", 0};
//static char*  anySlaveAddressList[]       = {(char*)"S", 0};
//static char*  slaveBroadcastAddressList[] = {(char*)"S*", 0};

  static const char SOM = '{'; //todo templatify, extract from template parameter (see MessageFilter.h)
  static const char EOM = '}'; //todo templatify, extract from template parameter (see MessageFilter.h)
  static const char MTO = '<'; //todo templatify, extract from template parameter (see MessageFilter.h)
  public:
  inline MessageFilterRT()
  {
    myAddressList=0; // passes all data;
    filterState=0x10;
  }

  inline void setAdressList(char** anAddressList)
  {
    myAddressList = anAddressList;
    // to refect. To avoid surprises, one should not change the addresslist while reception is active.
    // avoiding such suprises has a cost: cli(); myAddressList = anAddressList; if(filterState&0xf) filterState = 0xff; eni();
  }

  inline void terminateMessage()
  {
    filterState = (myAddressList==0)? 0x10: 0xff;
    return;
  }

  char check(char c, bool& bumpMessagePointer)
  {
    #define label(l) asm("MJ_label_" #l ":") // add to facilitate code inspection
    label(1);

    if(c==SOM)                  // start of message, common action for all states
    {
      label(2);
      if(myAddressList==0) return 0;    // eat, don't argue
      filterState=0;                    // set filterState to address check mode
      addressActiveMask = 0xff;         // prepare for a fresh round
      bumpMessagePointer=false;
      return 0;                         // we'll keep the SOM character
    }
    
    if (filterState<0)          // ignore all data until start of message
    {
      label(3);
      return -1;                // we'll ignore the character
    }

    if (filterState==0x10)      // message active
    { label(4);
      if (c==EOM)
        if(myAddressList!=0) filterState=0xff; // end of message, unless we are pass all, set filterState to ignore all data
      return 0; //but we'll always eat the character
    }

    // address check
    label(5);
    char** headerAddress = myAddressList;
    unsigned char testMask = 1;

    // loop over address list and check for active addresses if still valid
    while( (unsigned char)(addressActiveMask & (-testMask)) ) // address filters active
    { label(6);
      if ((unsigned char)(addressActiveMask&testMask) ) // this address filter is still active
      { label(6_0);
        if( *headerAddress==0) { label(6_1); addressActiveMask &= ~(-testMask); break;}               // or (testMask-1) // we are at the end, clear any remaining bits of our addressActiveMask
        label(6_2);
        if(     (*headerAddress)[(unsigned char)filterState] == c) { }                                               // this filter is still in the race
        else if((*headerAddress)[(unsigned char)filterState] != 0) { label(6_3); addressActiveMask &= ~testMask; }   // this filter drops out
        else                                                                                          // this filter is a match
        { label(6_4);
          unsigned char result = filterState+1;
          filterState = 0x10;
          return result;                // we accept the address
        }
        label(6_5);
      }
      label(7);
      testMask = testMask<<1;
      headerAddress++;
      label(8);
    }

    label(9);
    // match not yet confirmed, is there still hope... ?
    if(addressActiveMask!=0)    // yes. increase filterState, the character will be added to the buffer
    {
      filterState++;
      bumpMessagePointer=false;
      return filterState;
    }
    else                        // no, there are no match candidates left, reject the header accumulated so far
    {
      filterState = 0xff;
      return -1;
    }
  } // check()
};

#endif // !defined(HardwareSerial_RS485Helper_h)
