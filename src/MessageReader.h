/* MessageReader.h copyright notice

Reader class to build up messages and filter stuff that is not for us.

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

#if !defined(MessageReader_h)
#define      MessageReader_h

/* Description
Reader class to build up messages and filter stuff that is not for us.
*/

#if !defined(testCode)
#include "Arduino.h" // we need Serial
#else
#define new_dev
#include <stdio.h>
#include <string.h>
class SerialTest
{
  char* test;
  char* next;
  int   nc;
  char  wait;
  char  time;
public:
  SerialTest()
  {
    test = (char*) "{S*Next is empty}{S*}BUG1 (see code){S*S*hoi1}{S1S1hi1}{S2boe1!}{ssdoei1!}{m1!}{S*S*hoi2{S1S1hi2{S2boe2!{ssdoei2!{m2!{}S*hoi!}S1hi!}S2boe!}ssdoei!}m!}{S*S*hoi3${S1S1hi3${S2boe3!${ssdoei3!${m3!${S1S123456789abcdefghijkl...4}{123456789abcdefghijkl!}{S*]}";
//  BUG1: the message closing character of an empty message is ignored at the address match
    next = test;
    wait = 0;
    time = 0;
  }

  char* getTestString() { return test; }

  int available()
  {
    if(wait>0) {wait--; time++; return 0;}
    nc = *next++;
    if     (nc == '$') wait=15;
    else if(nc == '%') wait= 5;
    else return 1;
    time=0;
    return 0;
  }
  int read()
  {
    int v = nc;
    nc=-1;
    return v;
  }
  int getInputLatency()
  {
    return time;
  }
  void setAddressFilter(char** anAddressList, unsigned int theTimeoutValue=100)
  {
  }
} Serial;

#endif

#include "utility/MessageFilter.h"


class MessageReaderState
{
    unsigned char myState;

public:
    static const unsigned char IDL  = 0x00;  // no data received
    static const unsigned char HDR  = 0x10;  // collecting header
    static const unsigned char ACT  = 0x20;  // message active
    static const unsigned char TRM  = 0x80;        // message terminated
    static const unsigned char RDY  = TRM | 0x01;  // message ready
    static const unsigned char ABT  = TRM | 0x02;  // abnormal termination
    static const unsigned char MTO  = TRM | 0x03;  // timeout
    static const unsigned char BOF  = TRM | 0x04;  // buffer over flow

    inline void set(unsigned char aState)     { myState =aState; }
    inline unsigned char get()                { return  myState; }
    inline bool equals(unsigned char aState)  { return  myState == aState; }

    inline bool messagePending()              { return myState  & TRM; }
    inline bool messageActive()               { return myState == ACT; }
};

// The unique Serial<n> instances of the HardwareSerialRS485_<n> class is accessible through the static reference HardwareSerialRS485_<n>::ourSerialObject.
// TODO make independent of HardwareSerialRS485  // this requires:  
//                                                  - passage of SOM and EOF characters by template parameters
//                                                  - Serial object passage on object creation.
//                                                  - abandoning setAddressFilter service.

template<unsigned char t_MessageBufferLength, class T__HardwareSerialRS485, class t_MFP=RS485configuration_MessageFilterParameters>
class MessageReader : public MessageFilter<RS485configuration_MessageFilterParameters>
{
private:
    char   myMessageBuffer[t_MessageBufferLength];
    unsigned char myCurrentPosition;
    unsigned char myMessageTimeout;
    MessageReaderState messageState;
    bool messagePending() { return messageState.messagePending(); }

public:
    MessageReader(char** theAddressList, unsigned char theMessageTimeout=0)
    {
      T__HardwareSerialRS485::ourSerialObject.setAddressFilter(theAddressList, theMessageTimeout);    // this set the RT filter
      setAddressList(theAddressList);
      myCurrentPosition = 0;
      myMessageTimeout  = theMessageTimeout;
      messageState.set(MessageReaderState::IDL);
    }

    unsigned char getState()
    {
        if( messageState.messagePending() ) return messageState.get(); // this will happen if we do not acknoledge the current message with a call to nextMessage()

        while (T__HardwareSerialRS485::ourSerialObject.available() > 0)
        {
          char c = T__HardwareSerialRS485::ourSerialObject.read();

          if(c == t_MFP::SOM and messageState.messageActive()) {messageState.set(MessageReaderState::ABT); break; } // abnorml termination

          // message filtering, character are poked into the buffer, myCurrentPosition is updated only after the message address is accepted
          char filterState = check(c);

//        if(filterState<0) // this can happen if we would call terminateMessage() (either by us, or by our callers, should we hide terminateMessage() or make it protected?)
          if(filterState&0x70) // note: an address match could end up as 0x20
          {
              if(c == t_MFP::EOM) { messageState.set(MessageReaderState::RDY); break; }
              myMessageBuffer[myCurrentPosition++] = c;
              if( myCurrentPosition == t_MessageBufferLength-1 ) { messageState.set(MessageReaderState::BOF); break; } // no space, we continue
              messageState.set(MessageReaderState::ACT);
          }
        }

        if( messageState.messageActive() and myMessageTimeout!=0)
        {
          if(T__HardwareSerialRS485::ourSerialObject.getInputLatency() > myMessageTimeout) messageState.set(MessageReaderState::MTO);
        }
        return messageState.get();
    }

    bool hasMessage()                          { getState(); return messagePending(); }
    
    static bool isMessage(unsigned char state) { return state >= MessageReaderState::RDY; }

    void nextMessage()
    {
      if( !messagePending() ) return;                       // message building in progress
      else if(messageState.equals(MessageReaderState::BOF)) // we were in overflow
      {
        messageState.set(MessageReaderState::ACT);              // we just continue
      }
      else if(messageState.equals(MessageReaderState::ABT)) // we already got the SOM
      {
          check(t_MFP::SOM);                                    // MessageFilter did not know about it yet.
          messageState.set(MessageReaderState::HDR);
      }
      else messageState.set(MessageReaderState::IDL);       // we are ready
      myCurrentPosition = 0;
    }

    char* getMessage()
    {
      myMessageBuffer[myCurrentPosition  ] = '\0';
      return myMessageBuffer;
    }

    char* getMessageOrigin()
    {
      return getActiveAddress();
    }

    unsigned char getMessageLength()
    {
      // to debate: getState();
      return messagePending() ? myCurrentPosition : 0;
    }

};

/* ASCII table
  Binary	Dec	Hex	Abbr c  e   Name
000 0000	0	00	NUL	^@	\0	Null character
000 0001	1	01	SOH	^A		Start of Header
000 0010	2	02	STX	^B		Start of Text
000 0011	3	03	ETX	^C		End of Text
000 0100	4	04	EOT	^D		End of Transmission
000 0101	5	05	ENQ	^E		Enquiry
000 0110	6	06	ACK	^F		Acknowledgment
000 0111	7	07	BEL	^G	\a	Bell
000 1000	8	08	BS	^H	\b	Backspace
000 1001	9	09	HT	^I	\t	Horizontal Tab
000 1010	10	0A	LF	^J	\n	Line feed
000 1011	11	0B	VT	^K	\v	Vertical Tab
000 1100	12	0C	FF	^L	\f	Form feed
000 1101	13	0D	CR	^M	\r	Carriage return
000 1110	14	0E	SO	^N		Shift Out
000 1111	15	0F	SI	^O		Shift In
001 0000	16	10	DLE	^P		Data Link Escape
001 0001	17	11	DC1	^Q		Device Control 1 (oft. XON)
001 0010	18	12	DC2	^R		Device Control 2
001 0011	19	13	DC3	^S		Device Control 3 (oft. XOFF)
001 0100	20	14	DC4	^T		Device Control 4
001 0101	21	15	NAK	^U		Negative Acknowledgment
001 0110	22	16	SYN	^V		Synchronous idle
001 0111	23	17	ETB	^W		End of Transmission Block
001 1000	24	18	CAN	^X		Cancel
001 1001	25	19	EM	^Y		End of Medium
001 1010	26	1A	SUB	^Z		Substitute
001 1011	27	1B	ESC	^[	\e	Escape
001 1100	28	1C	FS	^\		File Separator
001 1101	29	1D	GS	^]		Group Separator
001 1110	30	1E	RS	^^		Record Separator
001 1111	31	1F	US	^_		Unit Separator
111 1111	127	7F	DEL	^?		Delete 
 */

#ifdef testCode
/* compilation / testing

# options to get it compiled outside Arduiono environment (for debugging and code optimisation)
cd $Arduino/myProjects/libraries/MessageFilter
$Arduino/arduino-1.0.5/hardware/tools/avr/bin/avr-g++ -mmcu=atmega328p -DtestCode -O3 -S test.cpp

see -
http://gcc.gnu.org/wiki/avr-gcc
http://gcc.gnu.org/onlinedocs/gcc-4.3.6/gcc/Optimize-Options.html#Optimize-Options

g++ -DtestCode -E test.cpp > test.p   #preprocessor only
g++ -DtestCode -S test.cpp            #assemble only
g++ -DtestCode -c test.cpp            #compile only
g++ -DtestCode -mmcu=atmega328p -Os -S test.cpp
g++ -DtestCode -mmcu=atmega328p -Os -c test.cpp
g++ -DtestCode -mmcu=atmega328p -Os    test.cpp

*/

int main()
{
  char messageBuffer[8];
  char* anAddressList[] = {(char*)"S1", (char*)"S*", 0};
  MessageReader<8> myMessageReader(anAddressList, 10);

  printf("TestString: %s\n",T__HardwareSerialRS485::ourSerialObject.getTestString());
  while (true)
  {
    unsigned char state = myMessageReader.getState();
    if(state >= MessageReaderState::RDY)
    {
      printf("messageState=0x%02x from:%s message:%s\n", state, myMessageReader.getMessageOrigin(), myMessageReader.getMessage());
      if(myMessageReader.getMessage()[0]==']') break;
      myMessageReader.nextMessage();
    }
  }
  printf("done\n");
  return 0;
}
#endif // testCode

#endif // !defined(MessageReader_h)