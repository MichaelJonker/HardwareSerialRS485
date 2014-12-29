/* CommunicationParameters.h copyright notice

Copyright (c) 2014 Michael Jonker.  All right reserved.

 http://en.wikipedia.org/wiki/Beerware
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Michael Jonker <EPID: 52.36040, 4.87001, 5m, 19540508.621> wrote this file.
 * As long as you retain this notice you can do whatever you want with this stuff.
 * If we meet some day, and you think this stuff is worth it, you can buy me a beer in return.

This code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details. You can obtain a copy of this license
from the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef CommunicationParameters_h
#define CommunicationParameters_h

#include <HardwareSerial_RS485.h>

#include "Printable.h"
#include "Arduino.h"

// Hartbeats:
unsigned const int HBM_TheBeatGoesOn       = 0x01; // value after masking to switch on
unsigned const int HBM_TheBeatGoesOff      = 0x03; // value after masking to switch off
const int HBM_Slow_Tonic          = 0x77; // 0B01110111  SUHBM=119
const int HBM_Normal_Tonic        = 0x37; // 0B00110111  SUHBM=55
const int HBM_Normal_Soft         = 0x2f; // 0B00101111  SUHBM=47
const int HBM_Normal_TrippleBeats = 0x27; // 0B00100111  SUHBM=39
const int HBM_AdrelanieShot       = 0x1b; // 0B00011011  SUHBM=27
const int HBM_Palpitation         = 0x07; // 0B00000111  SUHBM=7
const int HBM_Fibrilation         = 0x03; // 0B00000011  SUHBM=3 
const int HBM_CardiacArrest       = 0x01; // 0B00000001  SUHBM=1
const int HBM_Dead                = 0x00; // 0B00000000  SUHBM=0

template<unsigned char T_HBled = 12>
class CommunicationParameters : public Printable // ==================================================================================================
{
  unsigned char myRSid;
  unsigned char myRS485Mode;
  //0x11: enable read and write with local echo and collision check. !! need to call flush after every message
  //0x13: enable read and write without local echo. 
  //0x23: disable read and enable permanent write.
  unsigned char myLoopDelay;
  unsigned int  myVerboseLevel;
  unsigned int  myHartBeatMask;
  unsigned int  myBreathingMask;

  static CommunicationParameters  rs232Communication;
  static CommunicationParameters  rs485Communication;
  static CommunicationParameters* activeCommunicationParameters;
  static unsigned long ourLoopCount;
  static const unsigned char HB_led = T_HBled;  // defines the alternative output LED pin
  static unsigned long class_init();

  public:

  static inline void activateRS232()                      { rs232Communication.activate(); }
  static inline void activateRS485()                      { rs485Communication.activate(); }

  static inline bool isRS232Mode()                        { return  activeCommunicationParameters->myRS485Mode==0x25; }
  static inline bool isVerboseLevel(int test)             { return (activeCommunicationParameters->myVerboseLevel & test)!=0 ; }
  static inline int  getVerboseLevel()                    { return  activeCommunicationParameters->myVerboseLevel;  }
  static inline int  getHartBeatMask()                    { return  activeCommunicationParameters->myHartBeatMask;  }
  static inline int  getBreathingMask()                   { return  activeCommunicationParameters->myBreathingMask; }
  static inline int  getLoopDelay()                       { return  activeCommunicationParameters->myLoopDelay;     }
  static inline int  getLoopCount()                       { return  ourLoopCount;     }

  static inline void setLoopDelay(    int aLoopDelay)     { activeCommunicationParameters->myLoopDelay     = aLoopDelay;     }
  static inline void setHartBeatMask( int aHartBeatMask)  { activeCommunicationParameters->myHartBeatMask  = aHartBeatMask;  }
  static inline void setBreathingMask(int aBreathingMask) { activeCommunicationParameters->myBreathingMask = aBreathingMask; }
  static inline void setVerboseLevel( int aVerboseLevel)  { activeCommunicationParameters->myVerboseLevel  = aVerboseLevel;  }
  static inline void setModeRS485(    int mode)           { rs485Communication.myRS485Mode = mode; }

  static inline boolean doLoopDelay()
  { 
    delay (activeCommunicationParameters->myLoopDelay);
    ourLoopCount++;

    if     ( (ourLoopCount & activeCommunicationParameters->myHartBeatMask ) == HBM_TheBeatGoesOn  )  digitalWrite(HB_led, HIGH);
    else if( (ourLoopCount & activeCommunicationParameters->myHartBeatMask ) == HBM_TheBeatGoesOff )  digitalWrite(HB_led, LOW);

    return ( (ourLoopCount & activeCommunicationParameters->myBreathingMask ) == 1);
  }

  CommunicationParameters(unsigned char aRSid, unsigned char aRS485Mode, unsigned int aLoopDelay, unsigned int aVerboseLevel, unsigned int aHartBeatMask, unsigned int aBreathingMask)
  {
    myRSid          = aRSid;
    myRS485Mode     = aRS485Mode;
    myLoopDelay     = aLoopDelay;
    myVerboseLevel  = aVerboseLevel;
    myHartBeatMask  = aHartBeatMask;
    myBreathingMask = aBreathingMask;
  }

  inline const __FlashStringHelper* getName() const { return (myRSid==0)? F("RS232"): F("RS485"); }
  size_t printTo(Print& p) const;

  void activate()
  {
    if( isRS232Mode() ) { Serial.print(F("Switching to ")); Serial.println(*this); }

    Serial.flush(); // flush anything still present before switching over

    activeCommunicationParameters = this;

    Serial.setMode(myRS485Mode); // this defines the control of the TxEnable an RdEnable of the 485 chip, and hence the communication mode.

    if( isRS232Mode() ) { Serial.print(F("Switched to ")); Serial.println(*this);}
  }

};
#endif // CommunicationParameters_h =======================================================================================================

