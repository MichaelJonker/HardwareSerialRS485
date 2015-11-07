/* HardwareSerialRS485_Tracer.h copyright notice

class to provide tracing functionality for debug during development 

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

#if !defined(HardwareSerialRS485_Tracer_h)
#define      HardwareSerialRS485_Tracer_h


#if !defined(RS485configuration_useTRACE)
// We will not use TRACE, define a dummy trace macro
#define TRACE(item)
#else
// NB: all trace interactions are handled through the TRACE macro which translates TRACE(action) into TRACE_action
#define TRACE(item) TRACE_##item

#ifdef TEST // activate this for stand alone testing of the HardwareSerialRS485_Tracer class
#include <stdio.h>
#define F(x) ((char*) x)
#define HEX 16
class Out
{
public:
  int print(char  c) { return printf("%c",c); }
  int print(char* c) { return printf("%s",c); }
  template<typename T>
  int print(T c) { return printf("%d",c); }
  template<typename T>
  int print(T c, int fc) { return printf("%x",c); }
  template<typename... Args>
  int print(Args... args) { return printf("args..."); }
  int println() { printf("\n"); }
} out;

unsigned char SREG;
bool isTxBusy() {return false; }
unsigned char  c;
unsigned char  error;
unsigned char  txState;
unsigned char  retries;
unsigned char  nowAvailable=4;
unsigned char  nowStampData=1;
unsigned short timeSince =11;
unsigned long  nowStamp_IC()    { return nowStampData++; }
unsigned long timeSinceRX() { return timeSince--; }
class TxBuffer
{
public:
  unsigned char itemsStored() { return nowAvailable--; }
} txBuffer;
void cli() {}
#endif // TEST

// Tracer class, provides untainted tracing functionality // todo externalize in separate header file.
template<unsigned char T_bufferSizeExp=8>
class Tracer
{
  protected:
  const static char bufferSizeExp = T_bufferSizeExp;
  const static int  bufferSize = 1<<bufferSizeExp;
  const static char ptrMask = bufferSize-1;
  unsigned char     buffer[bufferSize];
  unsigned char     herePtr;
  bool              enabled;

  void store(unsigned char  item)
  {
    buffer[herePtr++ & ptrMask]=(unsigned char)item; 
  }
  void store(unsigned short item)
  {
    buffer[herePtr++ & ptrMask]=(unsigned char)item; item = item>>8;
    buffer[herePtr++ & ptrMask]=(unsigned char)item;;
  }
  void store(unsigned long  item)
  {
    buffer[herePtr++ & ptrMask]=(unsigned char)item; item = item>>8;
    buffer[herePtr++ & ptrMask]=(unsigned char)item; item = item>>8;
    buffer[herePtr++ & ptrMask]=(unsigned char)item; item = item>>8;
    buffer[herePtr++ & ptrMask]=(unsigned char)item;
  }
  void store(unsigned int   item) { store((unsigned short) item ); }

  void store(bool  item) { store((unsigned char ) item ); }
  void store(char  item) { store((unsigned char ) item ); }
  void store(int   item) { store((unsigned short) item ); }
  void store(short item) { store((unsigned short) item ); }
  void store(long  item) { store((unsigned long ) item ); }
  
  void record() {}

  void retrieve(unsigned char &item)
  {
    item=(unsigned char)buffer[--herePtr & ptrMask];
  }
  void retrieve(unsigned short &item)
  {
    item=(unsigned char)buffer[--herePtr & ptrMask];
    item=(unsigned char)buffer[--herePtr & ptrMask] | item<<8;
  }
  void retrieve(unsigned long &item)
  {
    item=(unsigned char)buffer[--herePtr & ptrMask];
    item=(unsigned char)buffer[--herePtr & ptrMask] | item<<8;
    item=(unsigned char)buffer[--herePtr & ptrMask] | item<<8;
    item=(unsigned char)buffer[--herePtr & ptrMask] | item<<8;
  }
  void retrieve(unsigned int &item) { retrieve((unsigned short &) item); }
  void retrieve(bool  &item) { retrieve((unsigned char &) item); }
  void retrieve(char  &item) { retrieve((unsigned char &) item); }
  void retrieve(short &item) { retrieve((unsigned short&) item); }
  void retrieve(int   &item) { retrieve((unsigned short&) item); }
  void retrieve(long  &item) { retrieve((unsigned long &) item); }


  unsigned long&  asLong (const float& aFloat) { return (unsigned long&)  aFloat;} // interpret a float as long
  float& asFloat(const unsigned long&  aLong)  { return (float&) aLong; }          // interpret a long  as float
  void store(float item) { store(        asLong( item)); }
  void retrieve(float &item) { retrieve((unsigned long&) item);}
// void retrieve(float &itemr) { unsigned long item; retrieve(item); itemr=asFloat(item);}
  
  void recover() {}

public:
  Tracer()
  {
    herePtr =0;
    enabled =true;
  }
  template<typename T, typename... Args>
  void record(T item, Args... args)
  {
    store(item);
	record(args...);
  }
  template<typename T, typename... Args>
  void recover(T &item, Args&... args)
  {
    retrieve(item);
	recover(args...);
  }
  bool isEnabled() { return enabled;  }
  void enable()    { enabled = true;  }
  void disable()   { enabled = false; }
};

#if 1 // definition of TRACE calls
// NB #if 1 allows decent editors (e.g. notepad++ )to collaps the whole structure 
#define TRACE_FLAG         ( serialTracer.isEnabled() )
#define TRACE_CLI()          unsigned char saved_sreg=SREG; cli()
#define TRACE_STI()          SREG=saved_sreg
#define TRACE_REC(args...)   serialTracer.record(args)

#define nowStamp2() ((unsigned int) nowStamp_IC())
#define nowStamp4() (               nowStamp_IC())
#define TRACE__DISABLE_                    if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(                                                                 (unsigned char)0x00);\
        serialTracer.disable();                                                                                                                                           TRACE_STI(); } // 0
#define TRACE__ENABLE_                     if(!TRACE_FLAG) {TRACE_CLI();\
        serialTracer.enable();                                           TRACE_REC(nowStamp4(),                                                     (unsigned char)0x04); TRACE_STI(); } // 4
#define TRACE__REPORT_                     \
        serialTracer.disable();            \
        serialTracer.report(*this)
#define TRACE__timeSinceRX ((unsigned short) T__RS485::timeSinceRX())  // save space in tracing timeSinceRX
#define TRACE_emptyPipeline                if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(                                isTxBusy(),                      (unsigned char)0x11); TRACE_STI(); } // 1 // can be optimised to use only one byte, if we differentiate the trace codes
#define TRACE_emptyPipeline_expired        if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(nowStamp2(), T__RS485::txState,                                  (unsigned char)0x13); TRACE_STI(); } // 3
#define TRACE_emptyPipeline_completed      if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(nowStamp2(), T__RS485::txState,                                  (unsigned char)0x23); TRACE_STI(); } // 3
#define TRACE_recoverTransfer_enableDREI   if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(nowStamp2(), T__RS485::txState,                                  (unsigned char)0x33); TRACE_STI(); } // 3 // not needed, tracing of txInputIdleWait is sufficient
#define TRACE_write                        if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(nowStamp2(), T__RS485::txState, txBuffer.itemsStored(), c,       (unsigned char)0x35); TRACE_STI(); } // 6
#define TRACE_write_timeout                if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(nowStamp2(), T__RS485::txState,                                  (unsigned char)0x43); TRACE_STI(); } // 3
#define TRACE_write_enableDREI             if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(nowStamp2(), T__RS485::txState, txBuffer.itemsStored(),          (unsigned char)0x44); TRACE_STI(); } // 4
#define TRACE_flush                        if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(nowStamp4(), T__RS485::txState, txBuffer.itemsStored(), retries, (unsigned char)0x47); TRACE_STI(); } // 6
#define TRACE_flush_1                      if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(nowStamp2(), T__RS485::txState, nowAvailable,                    (unsigned char)0x54); TRACE_STI(); } // 4
#define TRACE_txInputIdleWait              if( TRACE_FLAG) {TRACE_CLI(); TRACE_REC(nowStamp2(), T__RS485::txState, TRACE__timeSinceRX,              (unsigned char)0x55); TRACE_STI(); } // 5
// called from interrupt, no need to disable interrupts:
#define TRACE_txStart                      if( TRACE_FLAG) {             TRACE_REC(nowStamp2(),           txState, TRACE__timeSinceRX,              (unsigned char)0x65);              } // 5
#define TRACE_echo                         if( TRACE_FLAG) {             TRACE_REC(nowStamp2(),           txState, c, error,                        (unsigned char)0x75);              } // 5
#define TRACE_echo_error                   if( TRACE_FLAG) {             TRACE_REC(                       txState,                                  (unsigned char)0x81);              } // 1
#define TRACE_txCompleted                  if( TRACE_FLAG) {             TRACE_REC(nowStamp2(),           txState,                                  (unsigned char)0x83);              } // 2
#define TRACE_rxcInt_latencyLimit          if( TRACE_FLAG) {             TRACE_REC(                                latency,                         (unsigned char)0x84);              } // 4


#define RT_nStmp2            unsigned short nowStamp2;   recover(nowStamp2);
#define RT_nStmp4            unsigned long  nowStamp4;   recover(nowStamp4);
#define RT_txState           unsigned char  txState;     recover(txState);
#define RT_isTxBusy                   bool  isTxBusy;    recover(isTxBusy);
#define RT_available         unsigned char  available;   recover(available);
#define RT_retries           unsigned char  retries;     recover(retries);
#define RT_timeSinceRX       unsigned short timeSinceRX; recover(timeSinceRX);
#define RT_c                 unsigned char  c;           recover(c);
#define RT_c_err                      bool  error;       recover(error);         RT_c;
#define RT_latency           unsigned long  latency;     recover(latency);

#define someDelay /* delay(500) // activation of this delay maybe needed when diagnose is used to debug RS232 communication */

#define PR_nStmp2            someDelay; out.print(F(" nowStamp="   )); someDelay; out.print(nowStamp2,HEX,6);
#define PR_nStmp4            someDelay; out.print(F(" nowStamp="   )); someDelay; out.print(nowStamp4,HEX,6);
#define PR_txState           someDelay; out.print(F(" txState="    )); someDelay; out.print(txState     ,HEX);
#define PR_isTxBusy          someDelay; out.print(F(" isTxBusy="   )); someDelay; out.print(isTxBusy    );
#define PR_available         someDelay; out.print(F(" available="  )); someDelay; out.print(available   );
#define PR_retries           someDelay; out.print(F(" retries="    )); someDelay; out.print(retries     );
#define PR_timeSinceRX       someDelay; out.print(F(" timeSinceRX=")); someDelay; out.print(timeSinceRX );
#define PR_c                 someDelay; out.print(F(" c='"         )); someDelay; out.write(c           ); out.write('\'');
#define PR_c_err       PR_c; someDelay; out.print(F(" error="      )); someDelay; out.print(error       );
#define PR_latency           someDelay; out.print(F(" latency="    )); someDelay; out.print(latency     );
#endif //1

class SerialTracer : public Tracer<7> // SerialHardware specific specialisation of the Tracer class
{
  void report_DISABLE                   (Print &out) {                                                    out.print(F("TRACE_disabled            "));                                                    }
  void report_ENABLE                    (Print &out) {                                         RT_nStmp4; out.print(F("TRACE_enabled             ")); PR_nStmp4;                                         }
  void report_emptyPipeline             (Print &out) { RT_isTxBusy;                                       out.print(F("emptyPipeline             "));                        PR_isTxBusy;                }
  void report_emptyPipeline_expired     (Print &out) {                             RT_txState; RT_nStmp2; out.print(F("emptyPipeline_expired     ")); PR_nStmp2; PR_txState;                             }
  void report_emptyPipeline_completed   (Print &out) {                             RT_txState; RT_nStmp2; out.print(F("emptyPipeline_completed   ")); PR_nStmp2; PR_txState;                             }
  void report_recoverTransfer_enableDREI(Print &out) {                             RT_txState; RT_nStmp2; out.print(F("recoverTransfer_enableDREI")); PR_nStmp2; PR_txState;                             }
  void report_write                     (Print &out) { RT_c;       RT_available;   RT_txState; RT_nStmp2; out.print(F("write                     ")); PR_nStmp2; PR_txState; PR_available;   PR_c;       }
  void report_write_timeout             (Print &out) {                             RT_txState; RT_nStmp2; out.print(F("write_timeout             ")); PR_nStmp2; PR_txState;                             }
  void report_write_enableDREI          (Print &out) {             RT_available;   RT_txState; RT_nStmp2; out.print(F("write_enableDREI          ")); PR_nStmp2; PR_txState; PR_available;               }
  void report_flush                     (Print &out) { RT_retries; RT_available;   RT_txState; RT_nStmp4; out.print(F("flush                     ")); PR_nStmp4; PR_txState; PR_available;   PR_retries; }
  void report_flush_1                   (Print &out) {             RT_available;   RT_txState; RT_nStmp2; out.print(F("flush_1                   ")); PR_nStmp2; PR_txState; PR_available;               }
  void report_txInputIdleWait           (Print &out) {             RT_timeSinceRX; RT_txState; RT_nStmp2; out.print(F("txInputIdleWait           ")); PR_nStmp2; PR_txState; PR_timeSinceRX;             }
  void report_txStart                   (Print &out) {             RT_timeSinceRX; RT_txState; RT_nStmp2; out.print(F("txStart                   ")); PR_nStmp2; PR_txState; PR_timeSinceRX;             }
  void report_echo                      (Print &out) { RT_c_err;                   RT_txState; RT_nStmp2; out.print(F("echo                      ")); PR_nStmp2; PR_txState;                 PR_c_err;   }
  void report_echo_error                (Print &out) {                             RT_txState;            out.print(F("echo_error                "));            PR_txState;                             }
  void report_txCompleted               (Print &out) {                             RT_txState; RT_nStmp2; out.print(F("txCompleted               ")); PR_nStmp2; PR_txState;                             }
  void report_rxcInt_latencyLimit       (Print &out) { RT_latency;                                        out.print(F("rxcInt_latencyLimit       "));                        PR_latency;                 }
  void report_default(unsigned char code,Print &out)
  {
    out.print(F("unknown code 0x"));
    out.print(code,HEX);
    out.print(F("         "));
    int items = code &0x07;
    for (int i=0;i<items; i++)
    {
      unsigned char item;
      recover(item);
      out.print(F(" 0x"));
      out.print(item, HEX);
    }
  }

public:
  void report(Print &out)
  {
    unsigned int size_left=ptrMask+1;
	
	//static unsigned int prevCode=0xff;
    while(size_left)
    {
      unsigned char code;
      recover(code);
      if(size_left< (unsigned char)(1 +(code&0x07))) break;
	  size_left = size_left -1 -(code&0x07) ;
      switch (code)
      {
      case 0x00: report_DISABLE                   (out); break;
      case 0x04: report_ENABLE                    (out); break;
      case 0x11: report_emptyPipeline             (out); break;
      case 0x13: report_emptyPipeline_expired     (out); break;
      case 0x23: report_emptyPipeline_completed   (out); break;
      case 0x33: report_recoverTransfer_enableDREI(out); break;
      case 0x35: report_write                     (out); break;
      case 0x43: report_write_timeout             (out); break;
      case 0x44: report_write_enableDREI          (out); break;
      case 0x47: report_flush                     (out); break;
      case 0x54: report_flush_1                   (out); break;
      case 0x55: report_txInputIdleWait           (out); break;
      
      case 0x65: report_txStart                   (out); break;
      case 0x75: report_echo                      (out); break;
      case 0x81: report_echo_error                (out); break;
      case 0x83: report_txCompleted               (out); break;
      case 0x84: report_rxcInt_latencyLimit       (out); break;
      default:   report_default              (code,out); break;
      }
      out.println();
	  //if(code==0x04) break;	// stop reporting after we hit the enable.
	  //if(code == 0x00 and prevCode == 0x00) break;
	  //prevCode=code;
    }
  }
};

extern SerialTracer serialTracer;

#if defined(InHardwareSerialRS485__CPP) // ++=========================================================================================================
// this code is activated when the code is included from the HardwareSerialRS485.CPP file
// this is my personal style choice to keep all HardwareSerial related code together in one file (at the price of some negligible extra bytes to be processed by the compiler).

SerialTracer serialTracer;
#endif // defined(InHardwareSerialRS485__CPP) // ++===================================================================================================

#ifdef TEST
int main()
{
//	serialTracer.record((unsigned char)1, (unsigned int)2, (unsigned long)3);
//	serialTracer.record((float)4.2);
//	float f1;
//	char  b1;
//	int   i1;
//	long  l1;
//	serialTracer.recover(f1,l1);
//	serialTracer.recover(i1);
//	serialTracer.recover(b1);
//	printf("test\nb:0x%02x i:0x%04x l:0x%08x f:%4.2f\n", b1, i1, l1, f1);

    TRACE(_ENABLE_);
    TRACE(emptyPipeline);
    TRACE(emptyPipeline_expired);
    TRACE(emptyPipeline_completed);
    TRACE(recoverTransfer_enableDREI);
    TRACE(write);
    TRACE(write_timeout);
    TRACE(write_enableDREI);
    TRACE(flush);
    TRACE(flush_1);
    TRACE(txInputIdleWait);
    TRACE(txStart);
    TRACE(echo);
    TRACE(echo_error);
    TRACE(txCompleted);
	out.print(F("\n** Trace report\n"));
    TRACE(_REPORT_);
    TRACE(_ENABLE_);
    TRACE(write);
    TRACE(write_enableDREI);
    TRACE(txStart);
    TRACE(echo);
    TRACE(echo_error);
    TRACE(txCompleted);
    TRACE(flush);
    TRACE(emptyPipeline);
    TRACE(emptyPipeline_expired);
    TRACE(flush);
    TRACE(emptyPipeline);
    TRACE(emptyPipeline_completed);
    TRACE(recoverTransfer_enableDREI);
    TRACE(txStart);
    TRACE(echo);
    TRACE(txCompleted);
	out.print(F("\n**Trace report\n"));
    TRACE(_REPORT_);
}

/* to test:
g++ -g -std=c++11 -DTEST Dtest.cpp -o Dtest.exe
gdb -q Dtest.exe
*/

#endif // TEST

#endif // !defined(RS485configuration_TraceActive)

#endif // !defined(HardwareSerialRS485_Tracer_h)