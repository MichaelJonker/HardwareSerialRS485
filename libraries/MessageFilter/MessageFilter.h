/* MessageFilter.h copyright notice

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

#if !defined(MessageFilter_h)
#define      MessageFilter_h

/* Description
helper class to build up messages and filter stuff that is not for us.
//TODO rewrite as subclass of messageFilterRT class
*/

#include "Arduino.h"

class MTP_readable
{
public:
static const unsigned char SOM='{' ;   // Start of message       ^A  SOH     Start of Header
static const unsigned char SOT='|' ;   // Start of text          ^B  STX     Start of Text
static const unsigned char EOM='}' ;   // End of message         ^D  EOT     End of transmission
static const unsigned char PTE='!' ;   // Parity error           ^U  NAK     Negative Acknoledgment
static const unsigned char FTE='~' ;   // Frame error            ^G  BELL    Bell
static const unsigned char CME='<' ;   // Character mismatch     ^Z  SUB     Substitute
static const unsigned char MAB='<' ;   // Message Abort          ^X  CAN     Cancel
static const unsigned char MTO='>' ;   // Message Timeout        ^Y  EM      End of Medium
};
class MTP_special
{
public:
static const unsigned char SOM=0x01;   // Start of message       ^A  SOH     Start of Header
static const unsigned char SOT=0x02;   // Start of text          ^B  STX     Start of Text
static const unsigned char EOM=0x04;   // End of message         ^D  EOT     End of transmission
static const unsigned char PTE=0x15;   // Parity error           ^U  NAK     Negative Acknoledgment
static const unsigned char FTE=0x17;   // Frame error            ^G  BELL    Bell
static const unsigned char CME=0x1A;   // Character mismatch     ^Z  SUB     Substitute
static const unsigned char MAB=0x18;   // Message Abort          ^X  CAN     Cancel
static const unsigned char MTO=0x19;   // Message Timeout        ^Y  EM      End of Medium
};

class MessageFilterState
{
    unsigned char myState;

public:
    static const unsigned char IDL  = 0x00;  // no data received
    static const unsigned char HDR  = 0x10;  // collecting header
    static const unsigned char ACT  = 0x20;  // message active
    static const unsigned char IGN  = 0x40;  // ignore message
    static const unsigned char RDY  = 0x80;  // message ready
    static const unsigned char ABT  = 0x81;  // abnormal termination
    static const unsigned char BOF  = 0x82;  // buffer over flow
    static const unsigned char MTO  = 0x83;  // timeout

    bool isMessage() { return myState >= MessageFilterState::RDY; }
};

template<unsigned char t_MessageBufferLength, class t_MTP> class MessageFilter
{
public:

private:
    char   myMessageBuffer[t_MessageBufferLength];
    char** myAdressList;
    unsigned char myCurrentPosition;
    unsigned char messageState;
    unsigned char adressListIndex;

public:
    MessageFilter(char** theAdressList)
    {
      myAdressList      = theAdressList;
      myCurrentPosition = 0;
      messageState      = MessageFilterState::IDL;
      adressListIndex   = 0xFF;
    }

    unsigned char getState(unsigned int aTimeout=0)
    {
        if (messageState >= MessageFilterState::RDY) return messageState;

        while (Serial.available() > 0)
        {
      //digitalWrite(12, HIGH);
          char c = Serial.read();
          if(false) {} // lekker puh
          #ifdef handlesEscapes /* not yet tested */
          // TODO fold escape flag into message state
          else if(escaped) // in the escape state we blindly add the character to the message if active
          {
            if(messageState == MessageFilterState::ACT) ...
            escape=false;
          }
          else if(c==t_MTP::ESC) // escape character
          {
            escape=true; // we could filter for active and ignore states
          }
          #endif
          else if(c==t_MTP::SOM) // start of message
          {
            // if we were in the middle of collecting a message addressed to us, report it.
            if(messageState == MessageFilterState::ACT) return messageState = MessageFilterState::ABT;
            // if(messageState != messageState_IDL) ; // this is also an error but we silently ignore it
            myCurrentPosition = 0;
            messageState      = MessageFilterState::HDR;
            adressListIndex   = 0xFF;
          }
          else if (c==t_MTP::EOM)  // end of message          //TODO else if (c==t_MTP::TMOUT)  // message timeout
          {
            if(messageState == MessageFilterState::ACT) return messageState = MessageFilterState::RDY;
            // if(messageState != messageState_IGN) ; // this is also an error but we silently ignore it
            messageState = MessageFilterState::IDL;
          }
          else if(messageState != MessageFilterState::IGN and messageState != MessageFilterState::IDL) // we are not ignoring this message
          {
            myMessageBuffer[myCurrentPosition++]=c; // so we should store the character
            myMessageBuffer[myCurrentPosition  ]=0; // and add a terminator (do it here so we are ready any time to return the message, and avoid extra code at four other places)
            if(messageState == MessageFilterState::HDR)
            {
              char ac = addressCheck();
              if     (ac > 0) { messageState = MessageFilterState::ACT; adressListIndex = ac-1; }
              else if(ac < 0) messageState = MessageFilterState::IGN;
              //else ; // we cannot resolve this yet
            }

            if(myCurrentPosition==t_MessageBufferLength-1) return messageState = MessageFilterState::BOF;
          }
        }

        if( messageState == MessageFilterState::ACT and aTimeout!=0)
        {
          if(Serial.timeSinceRX() > aTimeout) messageState = MessageFilterState::MTO;
        }
        return messageState;
    }

    bool hasMessage()                          { return messageState >= MessageFilterState::RDY; }
    
    static bool isMessage(unsigned char state) { return state >= MessageFilterState::RDY; }

    void nextMessage()
    {
      if( messageState < MessageFilterState::RDY) return; // message building in progress
      else if(messageState == MessageFilterState::BOF)
      {
        messageState       = MessageFilterState::ACT;    // we were in overflow, do we need a special treatment for the continuation???
        myMessageBuffer[0] = t_MTP::SOM; // we keep this as a header
        myCurrentPosition  = 1;
        return;
      }
      else if(messageState == MessageFilterState::ABT) messageState = MessageFilterState::HDR;    // we already got the SOM
      else                                             messageState = MessageFilterState::IDL;    // we are ready
      myCurrentPosition = 0;
    }

    char* getBuffer()
    {
      return myMessageBuffer+strlen(myAdressList[adressListIndex]);
    }

    char* getMessageDestination()
    {
      return myAdressList[adressListIndex];
    }

    unsigned char getMessageLength()
    {
      // to debate: getState();
      if (messageState < MessageFilterState::RDY) return 0;
      return myCurrentPosition;
    }

private:
/* checks message header to see if there is a match with the list of accepted addresses
 * returns
  *   > 0 address match
  *   < 0 no address match
  *  == 0 undecided (need more header characters)
 */
    char addressCheck()
    {
      char result = -1; // unless decided otherwise, we have a no-match
      for(char** addressPointer = myAdressList; *addressPointer!=0; addressPointer++)
      {
        char* address = *addressPointer;
        char* header  = &myMessageBuffer[0];
        if(*address==0) return 1+(addressPointer-myAdressList);   // special case of an empty address string which matches all
        for(; true; )
        {
          if(*address++ != *header++) break; // this is not a match

          if(*address==0) return 1+(addressPointer-myAdressList); // all characters in the header match, this is a match

          if(*header==0) // we are at the end of message header, result is not yet defined.
          {
            result = 0;  // so it's maybe
            break;
          }
        } // single address test
      } // list of addresses
      return result;
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

#include <stdio.h>
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
    test = (char*) "{S*hoi}{S1hi}{S2boe}{ssdoei}{m}{S*hoi1{S1hi1{S2boe1{ssdoei1{m1{}S*hoi2}S1hi2}S2boe2}ssdoei2}m2}{S*hoi3${S1hi3${S2boe3${ssdoei3${m3${S123456789abcdefghijkl}{123456789abcdefghijkl}{S*]}";
    next = test;
    wait = 0;
    time = 0;
  }
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
  int getChar()
  {
    int v = nc;
    nc=-1;
    return v;
  }
  int timeSinceRX()
  {
    return time;
  }
} Serial;

int main()
{
  char messageBuffer[8];
  char* anAddressList[] = {(char*)"S1", (char*)"S*", 0};
  MessageFilter<8, MTP_readable> myMessageFilter(anAddressList);

  while (true)
  {
    unsigned char state = myMessageFilter.getState(10);
    if(state >= MessageFilterState::RDY)
    {
      printf("messageState=0x%02x %s\n", state, myMessageFilter.getBuffer());
      myMessageFilter.nextMessage();
      if(myMessageFilter.getBuffer()[2]==']') break;
    }
  }
  printf("done\n");
  return 0;
}
#endif // testCode

#endif // !defined(MessageFilter_h)