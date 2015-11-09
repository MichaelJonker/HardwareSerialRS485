/* MessageFilter.h copyright notice

Helper class to providing message filter capabilities.

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

#if !defined(MessageFilter_h)
#define      MessageFilter_h

/* Description
Filter class to filter messages addressed to destimations that are not on out address list. The filter class can be used by the interrupt routine of the HardwareSerial class to filter the date while it is coming in.
Messages are delimited by the StartOfMessage (T__MFP::SOM) en EndOfMessage (T__MFP::EOM) characters. These characters are passed through a template class paramater T__MFP

For each received character c the following method should be invoked:
unsigned char filterStatus = check(c);
filterStatus :
0x80 character should be discarted
0x0x header character, poke in dataBuffer: bufferBuffer(dataPointer+x)=c;
0xnx (n=1...7) message character, poke in dataBuffer:  bufferBuffer(dataPointer+x)=c; bump dataPointer:  dataPointer += x+1;
0x10           message character, shortened to: bufferBuffer(dataPointer++)=c;

In case the header data is to be discarted, it is sufficient to store only characters when if (filterState&0x70!=0) bufferBuffer(dataPointer++)=c;

*/

#include "utility/BitManipulation.h"
template< unsigned char T__SOM=0x01, unsigned char T__EOM=0x04 >
class MFP
{
public:
static const unsigned char SOM=T__SOM;   // Start of message     '{'   ^A  SOH     Start of Header
static const unsigned char EOM=T__EOM;   // End of message       '}'   ^D  EOT     End of transmission
/* eventually foreseen for future extentions:
static const unsigned char SOT=0x02;     // Start of text        '|'   ^B  STX     Start of Text
static const unsigned char ESC=0x08;     // Escape               '\\'  ^H  ???
static const unsigned char PTE=0x15;     // Parity error         '!'   ^U  NAK     Negative Acknoledgment
static const unsigned char FTE=0x17;     // Frame error          '~'   ^G  BELL    Bell
static const unsigned char CME=0x1A;     // Character mismatch   '<'   ^Z  SUB     Substitute
static const unsigned char MAB=0x18;     // Message Abort        '<'   ^X  CAN     Cancel
static const unsigned char MTO=0x19;     // Message Timeout      '>'   ^Y  EM      End of Medium
*/
};

typedef MFP< >          MFP_controlCharacters;
typedef MFP< '{', '}' > MFP_readableCharacters;



template<class T__MFP=MFP_readableCharacters>
class MessageFilter : public T__MFP // ===============================================================================================================
{
#define __label__(l) asm("MJ_label_" #l ":") // add to facilitate code inspection

  protected:
  char**        myAddressList;
  char          filterState;            // 0x80: ignore data until next start; 0x70: pass all data; 0x00 - 0x0f
  unsigned char addressActiveMask;
  unsigned char messageTimeout;
  public:

//possibly implement as static class constants:
//static char** passAllData                 = 0;
//static char*  blockData[]                 = {0};
//static char*  matchAllMessages[]          = {(char*)&matchAllMessages[1], 0}; // aka = {(char*)"", 0};
//static char*  anySlaveAddressList[]       = {(char*)"S", 0};
//static char*  slaveBroadcastAddressList[] = {(char*)"S*", 0};


  inline MessageFilter()
  {
    myAddressList=0; // passes all data;
    filterState=0x10;
  }

  inline void setAddressList(char** anAddressList)
  {
    myAddressList = anAddressList;
    filterState = (myAddressList)? 0xff: 0x10;
    // to reflect. To avoid surprises, one should not change the addresslist while reception is active.
    // avoiding such suprises has a cost: cli(); myAddressList = anAddressList; if(filterState&0xf) filterState = (myAddressList==0)? 0x10: 0xff; eni();
  }

  inline void setTimeout(unsigned char aTimeout)
  {
    messageTimeout = aTimeout;
  }

  inline unsigned char getTimeout()
  {
    return messageTimeout;
  }

  inline void terminateMessage()
  {
    filterState = (myAddressList)? 0xff: 0x10;
    return;
  }

  inline char check(char c)
  {
    __label__(1);

    // Todo implement escape handling ...
    if(c== T__MFP::SOM)                          // start of message, common action for all states
    {
      __label__(2);
      if(myAddressList==0) return 0x10;             // eat, don't argue
      filterState=0;                                // set filterState to address check mode
      addressActiveMask = 0xff;                     // prepare for a fresh round
      return 0;                                     // we'll retain the SOM character
    }

    if (filterState<0)                          // ignore all data until start of message
    {
      __label__(3);
      return 0x80;                                  // we'll ignore the character
    }

    if( testBitsAny(filterState, 0xf0))         // message active (note, we may end up with 0x20 if the matching adress has length 0x0f)
    { __label__(4);
      if (c==T__MFP::EOM)                            // end of message,
        if(myAddressList!=0) filterState=0x80;      //                 unless we are pass all, set filterState to ignore all data
      return 0x10;                                  // but we'll always eat the character
    }

    // address check
    __label__(5);
    char** headerAddress = myAddressList;
    unsigned char testMask = 1;

    // loop over address list and check for active addresses if still valid
    while( testBitsAny(addressActiveMask, -testMask) )  // there are still untested address filters active. Nb: -testMask equals ~(testMask-1)
    { __label__(6);
      if ( testBitsAny(addressActiveMask,  testMask) )  // this address filter is still active
      { __label__(6_1);
        if( *headerAddress==0) { __label__(6_11); clrBits(addressActiveMask, -testMask); break; }                               // we are at the end, clear any remaining bits of our addressActiveMask
        __label__(6_12);
        if(     (*headerAddress)[(unsigned char)filterState] == c) { }                                                          // this filter is still in the race
        else if((*headerAddress)[(unsigned char)filterState] != 0) { __label__(6_13); clrBits(addressActiveMask, testMask); }   // this filter drops out
        else                                                                                                                    // this filter is a match
        { __label__(6_14);
          unsigned char result =(filterState+1)|0x10;       // we accept the address, and also mark this as message data (& bump pointer)
          addressActiveMask = headerAddress-myAddressList;  // this will alow a sub class to find the matching address.
          if (c==T__MFP::EOM) filterState=0x80;              // if this is also the end of the message set filterState to ignore all following data
          else               filterState=0x10;              // set to message body mode
          return result;
        }
        __label__(6_15);
      }
      __label__(6_2);
      testMask = testMask<<1;
      headerAddress++;
      __label__(6_3);
    }

    __label__(7);
    // match not yet confirmed, is there still hope... ?
    if(addressActiveMask!=0)    // yes, increase filterState, the character will be added to the buffer
    {
      filterState++;
      return filterState;
    }
    else                        // no, there are no match candidates left, reject the header accumulated so far
    {
      filterState = 0x80;
      return 0x80;
    }
  } // check()

  inline char* getActiveAddress()
  {
      return myAddressList[addressActiveMask]; // NB addressActiveMask has been 'borrowed' to hold the index of the active address
  }
}; // class MessageFilter ===========================================================================================================================

#endif // !defined(MessageFilter_h)