/* RS485CommandResponse.ino copyright notice

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

/* Description
   Command responds program to demonstrate RS485 communication.
*/


#include "HardwareSerialRS485.h"

// note, normally the IDE inserts a #include "Arduino.h" for us, but the IDE does not appreciate the implications of the #if defined... and places the #include at an inopportune place.
#include "Arduino.h"


#if defined( USBCON) // we have an on chip USB controler
// for the demo purpose, we creata a simple wrapper class 'MySerialClass' using the SerialUSBSwitch<> template that allows us to switch
// between RS485 (over HardwareSerialRS485_1) and USB (over Serial_).
typedef SerialUSBSwitch<HardwareSerialRS485_1, Serial_> MySerialClass;
// and we instanciate an object and assign it to the ourSerialObject reference in this class.
static  MySerialClass mySerialSwitch(Serial);
template<> MySerialClass& MySerialClass::ourSerialObject = mySerialSwitch;

#else // we have an external RS232 to USB, HardwareSerialRS485 can do the switching
typedef HardwareSerialRS485_0   MySerialClass;       // class used for communication
#endif

MySerialClass& mySerial=MySerialClass::ourSerialObject; // create a reference so we do not need long names




const unsigned char myHBled = 12;  // defines the output LED pin for the hartbeat

// convenience function
inline unsigned int seconds() { return (unsigned int) (millis()/1000); }    // wraps after 0xffff seconds =~18.2 hours



#define APP_FW_VERSION  "V2.0"
#define APP_IDENTIFIER  66

// we declare the weak method getBuildTime() which returns the compile time. This method can be overloaded externally with the real BuildTime (defined in boards.txt)
extern const __FlashStringHelper* getBuildTime() __attribute__((weak));
const __FlashStringHelper* getBuildTime()
{
  static const char __BuildTime__[] PROGMEM = "Compiled: " __DATE__ " " __TIME__;
  return reinterpret_cast<const __FlashStringHelper *> (&__BuildTime__[0]);
}


// Define Hartbeats:
// you can change the hartbeat with the command {S*HBM=...}. You find here some examples. Enjoy!
const unsigned char HBM_Slow_Tonic          = 0B01110111; //  {S*HBM=119}
const unsigned char HBM_Normal_Tonic        = 0B00110111; //  {S*HBM=55 }
const unsigned char HBM_Normal_Soft         = 0B00101111; //  {S*HBM=47 }
const unsigned char HBM_Normal_TrippleBeats = 0B00100111; //  {S*HBM=39 }
const unsigned char HBM_AdrelanieShot       = 0B00011011; //  {S*HBM=27 }
const unsigned char HBM_Palpitation         = 0B00000111; //  {S*HBM=7  }
const unsigned char HBM_Fibrilation         = 0B00000011; //  {S*HBM=3  }
const unsigned char HBM_CardiacArrest       = 0B00000001; //  {S*HBM=1  }
const unsigned char HBM_Dead                = 0B00000000; //  {S*HBM=0  }

const unsigned char HBM_TheBeatGoesOn       = 0x01; // value after masking to switch on
const unsigned char HBM_TheBeatGoesOff      = 0x03; // value after masking to switch off



#include "MessageReader.h"
typedef MessageReader<32, MySerialClass> MyMessageReaderClass;

template<class T_Serial, unsigned char T_HBled = 0>
class ApplicationControl // ==========================================================================================================================
{
  // This class manages various communication parameters and aspects of the application

  // default initialization values
  const static unsigned char default_applicationType = APP_IDENTIFIER;
  const static unsigned char default_slaveId         = '+';
  const static unsigned char default_priority        = 0x1f;
  const static unsigned char default_expirationLimit = 0;

  // saved on eeprom  
  unsigned char applicationType;
  unsigned char applicationInstance;
  unsigned char RS485_Priority;
  unsigned char RS232_ExpirationLimit;

  // eeprom addresses
  static unsigned char const A_ApplicationType       = 0;
  static unsigned char const A_ApplicationInstance   = 1;
  static unsigned char const A_Priority              = 2;
  static unsigned char const A_RS232_ExpirationLimit = 3;

  // not saved on eeprom
  unsigned int  RS232_ExpirationTime;             // time at which we will switch over to RS485

  struct CommunicationModeParameters
  {
    // The communicationModeParameters structure defines the communication mode dependent parameters, such as verboseLevel, hartbeat,etc.
    // Two instances are created from this class, one for rs232Communication, one for rs485 communication. The value of the myRS485Mode parameter in these instances
    // is initialized to RS232 and full-RS485 operational mode respectively. The actual myRS485Mode of the rs485Communication instance can be changed (with the command M485= descibed further down the code).
    unsigned char myRS485Mode;      // as described above
    unsigned char myLoopDelay;      // delay between checking for new commands
    unsigned char myVerboseLevel;   // defines what is said and what we keep for ourselves
    unsigned char myHartBeatMask;   // mode dependent hartbeat
    unsigned int  myBreathingMask;  // frequency of 'breathing' messages written to the output
  };

  CommunicationModeParameters  rs232Communication={0x25, 20, 0xff, HBM_Normal_Tonic, 0x00ff};
  CommunicationModeParameters  rs485Communication={0x11, 20, 0x80, HBM_Normal_Soft,  0xffff};
  CommunicationModeParameters* activeCommunicationParameters=&rs232Communication;
  unsigned long myLoopCount = init(); // a bodge, it would be better to use a constructor but that complicates the initialization of myHeader[5].
  unsigned long init()       { pinMode(T_HBled, OUTPUT); return 0; }

  public:
  inline          bool isRS232Mode()                             { return  activeCommunicationParameters == &rs232Communication; }
  inline          bool rs485Muted()                              { return (activeCommunicationParameters->myRS485Mode    & 0x0f)==0x05; }
  inline          bool isVerboseLevel(char test)                 { return (activeCommunicationParameters->myVerboseLevel & test)!=0 ; }
  inline unsigned char getVerboseLevel()                         { return  activeCommunicationParameters->myVerboseLevel;  }
  inline unsigned char getHartBeatMask()                         { return  activeCommunicationParameters->myHartBeatMask;  }
  inline unsigned int  getBreathingMask()                        { return  activeCommunicationParameters->myBreathingMask; }
  inline unsigned char getLoopDelay()                            { return  activeCommunicationParameters->myLoopDelay;     }
  inline unsigned int  getLoopCount()                            { return  myLoopCount;     }

  inline void setLoopDelay(     unsigned char aLoopDelay)        { activeCommunicationParameters->myLoopDelay     = aLoopDelay;     }
  inline void setHartBeatMask(  unsigned char aHartBeatMask)     { activeCommunicationParameters->myHartBeatMask  = aHartBeatMask;  }
  inline void setBreathingMask( unsigned int  aBreathingMask)    { activeCommunicationParameters->myBreathingMask = aBreathingMask; }
  inline void setVerboseLevel(  unsigned int  aVerboseLevel)     { activeCommunicationParameters->myVerboseLevel  = aVerboseLevel;  }
  inline void setModeRS485(     unsigned char mode)              { rs485Communication.myRS485Mode = mode; }

  inline boolean loopCheck()
  {
    delay (activeCommunicationParameters->myLoopDelay); // todo replace by a timer mechanism, also investigate interrupt based delays (to save power?)
    myLoopCount++; 
    if( T_HBled == 0) { /* niks */ } // no led no hart beat
    else if( (myLoopCount & activeCommunicationParameters->myHartBeatMask  ) == HBM_TheBeatGoesOn  )  digitalWrite(T_HBled, HIGH);
    else if( (myLoopCount & activeCommunicationParameters->myHartBeatMask  ) == HBM_TheBeatGoesOff )  digitalWrite(T_HBled, LOW);

    return ( (myLoopCount & activeCommunicationParameters->myBreathingMask ) == 1);
  }

  inline void PrintComPar(CommunicationModeParameters& theCommunicationParameters)
  {
    T_Serial::ourSerialObject.print(&theCommunicationParameters == &rs232Communication ? F("RS232"): F("RS485"));
    T_Serial::ourSerialObject.print(F(" communication. (485 mode="));
    T_Serial::ourSerialObject.print(theCommunicationParameters.myRS485Mode,HEX);
    T_Serial::ourSerialObject.println(F(")"));
  }

  inline void activate(CommunicationModeParameters& newCommunicationParameters)
  {
    if( rs485Muted() ) { T_Serial::ourSerialObject.print(F("Switching to ")); PrintComPar(newCommunicationParameters); }

    T_Serial::ourSerialObject.flush(); // flush anything still present before switching over
    activeCommunicationParameters = &newCommunicationParameters;
    T_Serial::ourSerialObject.setMode(activeCommunicationParameters->myRS485Mode); // this defines the control of the TxEnable an RdEnable of the 485 chip, and hence the communication mode.

    if( rs485Muted() ) { T_Serial::ourSerialObject.print(F("Switched to ")); PrintComPar(newCommunicationParameters); }
  }


  inline void activateRS232() { activate(rs232Communication); }
  inline void activateRS485() { activate(rs485Communication); }

  char          myHeader[5] = "M*S*";
  char*         myAddress   = &myHeader[2];

  inline void EEPROM_restore()
  {
      // read our 'identity' from the EEPROM
      applicationType = eeprom_read_byte((unsigned char*) A_ApplicationType);
      if(applicationType == default_applicationType)    // but only if the application_type matches the value of accepted_application_type.
      {
        setRS485_SlaveId        (eeprom_read_byte((unsigned char*) A_ApplicationInstance));
        setRS485_Priority       (eeprom_read_byte((unsigned char*) A_Priority));
        setRS232_ExpirationLimit(eeprom_read_byte((unsigned char*) A_RS232_ExpirationLimit));
      }
      else                                              // otherwise we will initialize our identity these default values:
      {
        applicationType          =default_applicationType;
        setRS485_SlaveId         (default_slaveId);
        setRS485_Priority        (default_priority);
        setRS232_ExpirationLimit (default_expirationLimit);
      }
  }
  inline void EEPROM_save()
  {
      eeprom_write_byte((unsigned char*) A_ApplicationType,       applicationType);
      eeprom_write_byte((unsigned char*) A_ApplicationInstance,   applicationInstance);
      eeprom_write_byte((unsigned char*) A_Priority,              RS485_Priority);
      eeprom_write_byte((unsigned char*) A_RS232_ExpirationLimit, RS232_ExpirationLimit);
  }
  inline void setRS485_SlaveId(char theSlaveId)
  {
      applicationInstance=theSlaveId;
      myAddress[1]       =theSlaveId;
  }
  inline void setRS485_Priority(unsigned char thePriority)    { RS485_Priority=thePriority <= 31 ?  thePriority : 31; }
  inline void setRS232_ExpirationLimit(unsigned char theExpirationLimit)
  {
      RS232_ExpirationLimit =theExpirationLimit;
      RS232_ExpirationTime  =seconds() +theExpirationLimit;
  }

  inline void setupRS232_ExpirationTime()           { RS232_ExpirationTime = seconds() +(RS232_ExpirationLimit&7); } // defines  short expiration time in the range of 0..7
  inline void resetRS232_ExpirationTime()           { RS232_ExpirationTime = seconds() + RS232_ExpirationLimit;    }
  inline bool isRS232_Expired()                     { return RS232_ExpirationLimit? ((signed int)(seconds()-RS232_ExpirationTime))>0 : false; } // this construct to cope with wrapping (after 0xffff seconds =~18.2 hours)

  inline void startMessage(unsigned char aPriority=0xff)
  {
    if(isRS232Mode()) return; // we do not bother with message dressing-up in 232 mode()
    mySerial.startTransaction(aPriority!=0xff ? aPriority:  RS485_Priority);
    mySerial.print((char) MyMessageReaderClass::SOM);
    mySerial.print(myHeader);
  }
  inline unsigned char endMessage()
  {
    if(isRS232Mode()) { mySerial.println(); return 0; } // we do not bother with message dressing-up in 232 mode()
    mySerial.print((char) MyMessageReaderClass::EOM);
    mySerial.println();
    return mySerial.endTransaction();
  }
  void printAliveMessage(Print& aPrinter)
  {
      startMessage();
      aPrinter.print(F("RS485CmdResp alive. Version "));  aPrinter.print(F(APP_FW_VERSION));
      aPrinter.print(F("("));                             aPrinter.print( getBuildTime() );
      aPrinter.print(F(") my address="));                 aPrinter.print(myAddress);
      aPrinter.print(F(" RS485_priority="));              aPrinter.print(RS485_Priority);
      aPrinter.print(F(" RS232_ExpirationTime="));        aPrinter.print((signed int)(RS232_ExpirationTime-seconds()));  // what is (or posibbly was) left
      aPrinter.print(F("/"));                             aPrinter.print(RS232_ExpirationLimit);                         // out of the quota
      endMessage();
  }
};

ApplicationControl<MySerialClass, myHBled> myApplicationControl;


// examples of address filters
//static char*  blockAllData[]   = {0};
//static char** passAllData = 0;
//static char*  matchAllMessages[]          = {(char*)&matchAllMessages[1], 0}; // aka = {(char*)"", 0};
//static char*  anySlaveAddressList[]       = {(char*)"S", 0}; // nb the remaining part of the slave address will be prepended to the message
//static char*  slaveBroadcastAddressList[] = {(char*)"S*", 0};

// char* myExactAddressList[] = {myApplicationControl.myAddress, 0};
char* myAddressList[]      = {myApplicationControl.myAddress, (char*)"S*", 0};

MyMessageReaderClass myMessageReader(myAddressList, 20); // N.B., parameters are passed to MySerialClass before the run of setup.


int PrintMemstat(Print& aPrinter)
{
    /* report on: */
    extern char *__brkval;
    extern char __heap_start;
//  extern char __heap_end;               // note __heap_end equals 0, i.e. SP is effectively used for heap end 
//  extern size_t __malloc_margin;        // actual value used by malloc (if changed by user before first allocation)
//  extern char *__malloc_heap_start;     // idem
//  extern char *__malloc_heap_end;       // idem
    unsigned int bv   = (int) (__brkval == 0 ? &__heap_start : __brkval); // effective break value
    unsigned int free = ((unsigned int) SP) - bv;                         // subtracted from free are the 4 bytes for local variables
//  Note local variables are pre-reserved on the stack. Hence there is no difference to read the stack pointer before or after declaration of local variables

//  The SP grows towards smaller values, starting from ??.
    aPrinter.print(F("MemStat: "));
    aPrinter.print(F(" &__heap_start=0x")); aPrinter.print((int)&__heap_start, HEX);   // , 0x23);
    aPrinter.print(F(" __brkval=0x"));      aPrinter.print(bv,                 HEX);   // , 0x23);
    aPrinter.print(F(" stackPointer=0x"));  aPrinter.print(SP,                 HEX);   // , 0x23);
    aPrinter.print(F(" free=0x"));          aPrinter.print(free,               HEX);   // , 0x23);
    return free; 
}

void debug()
{
    unsigned char savedMode =mySerial.getMode();
    mySerial.flush();

    mySerial.setMode(0x25); // disable 485
    mySerial.flush();
    mySerial.print("dbg-");
    mySerial.flush();
    mySerial.setMode(0x11); // enable  485
    mySerial.print("dbg.");
    mySerial.flush();
    mySerial.diagnose(F("\ndbg"));
    mySerial.setMode(savedMode); // enable  485
}

void setup()
{
    myApplicationControl.EEPROM_restore();  // read our identity from the EEPROM

    mySerial.setAddressFilter(myAddressList);

//  initialize mySerial communication at 9600 bits per second:
    mySerial.begin(9600);

//  report to the mySerial port
    myApplicationControl.printAliveMessage(Serial);
    PrintMemstat(Serial);

//  this will set a (short) initial value of the Expiration time
    myApplicationControl.setupRS232_ExpirationTime();
}

void loop()
{
    Print& thePrinter = mySerial;

    unsigned char state = myMessageReader.getState();
    if( myMessageReader.isMessage(state) )
    {
        char* messageParser      = myMessageReader.getMessage();
        char* messageDestination = myMessageReader.getMessageOrigin();
//      char  messageLength      = myMessageReader.getMessageLength();

// todo (someday) add message reception statistics
        // test for errors in message reception (truncated, timeout, error, ...)
        // if(state == ) // TODO


        if( myApplicationControl.isVerboseLevel(0x04) )
        {
          thePrinter.print(F("MessageFilterState="));  thePrinter.print(state,HEX);
          thePrinter.print(F(" messageDestination=")); thePrinter.print(messageDestination);
          thePrinter.print(F(" messageParser="));      thePrinter.println(messageParser);
        }

        // convenience macros for readability and to avoid typing errors, using progmem for test strings //TODO, consider to include in MessageReader Class??
        #define startWith( _message_,_string_)  (!strncmp_P((_message_),PSTR(_string_), sizeof((_string_))-1) )
        #define canRemove( _message_,_string_)  (!strncmp_P((_message_),PSTR(_string_), sizeof((_string_))-1) ? (_message_ += sizeof((_string_))-1, true) : false)

/* command examples 
{S*}{S*}{S*?}{S*Memstat}{S*EEPROM.PRINT}{S*VBL?}{S*ECHO:Waar eens de boterbloempjes bloeiden...}{S*RS485}
{SARS232}
{SAVBL?}
{SAPRIO=64}{SA?}{SAEEPROM.RESTORE}{SA?}{SAPRIO=64}{SA?}{SAEEPROM.SAVE}{SA?}{SAEEPROM.RESTORE}{SA?}
{SA?}{SARS485}{SA?}{SARS232}{SA?}
{SAdbg}{S*?}{SA?}

switching between RS485 and RS232 mode can be done with the commands RS232 and RS485, assuming a slave ID A, the commands are {SARS232} and {SARS485}
THe command SR232 will always lead to switching of any activity on the 485 bus. The command RS485 will select the RS485 communication mode. However,
in this mode there is full control over the actual RS485 mode (including a pure RS232)
Most notably:
{SAM485=17}{SARS485}  will activate 485 mode=0x11: read from RS485 & write to RS485 with local echo and collision check; This is the full RS485 mode
{SAM485=18}{SARS485}  will activate 485 mode=0x12: read from RS485 & write to RS485 with local echo and no collision check; beware, this may lead to some confusing effects as the output are reinterpreted as commands.
{SAM485=19}{SARS485}  will activate 485 mode=0x13: read from RS485 & write to RS485 without local echo
{SAM485=20}{SARS485}  will activate 485 mode=0x14: read from RS485 & write to RS485 without local echo, write enable permanently activated even after write is finished
{SAM485=21}{SARS485}  will activate 485 mode=0x15: read from RS485 & write to RS485 disabled

{SAM485=33}{SARS485}  will activate 485 mode=0x21: read from RS232 & write to RS485 with local echo and collision check
{SAM485=34}{SARS485}  will activate 485 mode=0x22: read from RS232 & write to RS485 with local echo and no collision check
{SAM485=35}{SARS485}  will activate 485 mode=0x23: read from RS232 & write to RS485 without local echo
{SAM485=36}{SARS485}  will activate 485 mode=0x24: read from RS232 & write to RS485 without local echo, write enable permanently activated even after write is finished
{SAM485=37}{SARS485}  will activate 485 mode=0x25: read from RS232 & write to RS485 disabled;  This is identical to the RS232 mode.
*/

        
    
    
        if(false); // lekker puh
        else if(*messageParser=='\0')                      { myApplicationControl.startMessage(); myApplicationControl.endMessage(); }
        else if(startWith(messageParser,"?"))              { myApplicationControl.printAliveMessage(thePrinter); }
        else if(canRemove(messageParser,"SLID="))          { myApplicationControl.setRS485_SlaveId(messageParser[0]); }
        else if(canRemove(messageParser,"PRIO="))          { myApplicationControl.setRS485_Priority(atoi(messageParser)); }
        else if(canRemove(messageParser,"EXPL="))          { myApplicationControl.setRS232_ExpirationLimit(atoi(messageParser)); }
        else if(canRemove(messageParser,"EEPROM."))
        {    if(false) {} //lekker puh
             else if(startWith(messageParser,"SAVE"))      { myApplicationControl.EEPROM_save(); }
             else if(startWith(messageParser,"RESTORE"))   { myApplicationControl.EEPROM_restore(); }
        }
        else if(startWith(messageParser,"Memstat"))        { myApplicationControl.startMessage(); PrintMemstat(thePrinter); myApplicationControl.endMessage(); }
        else if(canRemove(messageParser,"Echo:"))          { myApplicationControl.startMessage(); thePrinter.print(messageParser); myApplicationControl.endMessage(); }
        
        else if(canRemove(messageParser,"VBL="))           { myApplicationControl.setVerboseLevel(atoi(messageParser)); }
        else if(startWith(messageParser,"VBL?"))           { myApplicationControl.startMessage(); thePrinter.print(F("VerboseLevel=0x")); thePrinter.println( myApplicationControl.getVerboseLevel(),HEX); myApplicationControl.endMessage(); }
        else if(canRemove(messageParser,"LDY="))           { myApplicationControl.setLoopDelay(atoi(messageParser)); }
        else if(canRemove(messageParser,"HBM="))           { myApplicationControl.setHartBeatMask(atoi(messageParser)); }
        else if(canRemove(messageParser,"BRM="))           { myApplicationControl.setBreathingMask(atoi(messageParser)); }

        else if(canRemove(messageParser,"M485="))          { myApplicationControl.setModeRS485( atoi(messageParser)); myApplicationControl.startMessage(); thePrinter.print(F("RS485Mode=")); thePrinter.print(atoi(messageParser),HEX); myApplicationControl.endMessage(); }
        else if(startWith(messageParser,"RS485"))          { myApplicationControl.activateRS485(); }
        else if(startWith(messageParser,"RS232"))          { myApplicationControl.activateRS232(); }
        else if(startWith(messageParser,"dbg"))            { debug(); } // outputs "dbg" to in on RS232, "dbg" on RS485, and then calls diagnose(). only meaningfull in conjunction with tracing active.
// blink TxE, RxE duration
//      else if(canRemove(messageParser,"BlinkTx"))        { debug(); }
//      else if(canRemove(messageParser,"BlinkRx"))        { debug(); }
// RS485 echo: no local echo, local echo, local echo check
//        else if(canRemove(messageParser,"EchoRS485:"))     { myApplicationControl.activateRS485(); myApplicationControl.startMessage(); thePrinter.print(messageParser); myApplicationControl.endMessage(); }myApplicationControl.activateRS232(); }
//        else if(canRemove(messageParser,"EchoRS485:"))     { myApplicationControl.activateRS485(); myApplicationControl.startMessage(); thePrinter.print(messageParser); myApplicationControl.endMessage(); }myApplicationControl.activateRS232(); }
//RS485 read to read one string
//
//      unrecognized command:
        else
        {
            if( myApplicationControl.isVerboseLevel(-1) ) { thePrinter.print(myApplicationControl.myHeader); thePrinter.print(F("_?:")); thePrinter.println(messageParser); }
        }
        myMessageReader.nextMessage();

        if( myApplicationControl.rs485Muted() )
        {
          myApplicationControl.resetRS232_ExpirationTime(); // as we got a message, reset the ExpirationTime for RS232 communications

          if( myApplicationControl.isVerboseLevel(0x08) )
          {
            // print some report ...
          }
        }

        return; // so we can read next message if needed
    }

    // there was no message ...
    if( myApplicationControl.rs485Muted() )
    {
      if(myApplicationControl.isRS232_Expired() ) myApplicationControl.activateRS485();
    }

    if( myApplicationControl.loopCheck() )
    {
        myApplicationControl.startMessage(); thePrinter.print(F("HB: ")); thePrinter.print((unsigned int)myApplicationControl.getLoopCount(),DEC); myApplicationControl.endMessage();
    }
}
