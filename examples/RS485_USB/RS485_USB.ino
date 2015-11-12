/* RS485_USB.ino copyright notice

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
   Simple RS485 to USB bridge.
*/


#include "HardwareSerialRS485.h"

// note, normally the IDE inserts a #include "Arduino.h" for us, but the IDE does not appreciate the implications of the #if defined... and places the #include at an inopportune place.
#include "Arduino.h"


#if !defined( USBCON)
#error "The RS485_USB.ino sketch only runs on devices with a native USB controler. Sorry"
#endif

#include "USBAPI.h"

class UsbSerialWrapper : public Serial_ // ===========================================================================================================
{
// A small wrapper arround a HardwareSerial class, providing some of the missing methods to make it compatibe with HardwareSerialRS485.

public:
//This is a singleton class (i.e. a class with only one object). We instantiate a reference here:
  static UsbSerialWrapper& ourSerialObject;

  UsbSerialWrapper(Serial_& aSerial) : Serial_(aSerial) {} // todo, check construct

#pragma GCC diagnostic push // we only ignore them from push to pop
#pragma GCC diagnostic ignored "-Wunused-parameter"
//  inline void          startTransaction(unsigned char thePriority=0x1f)                         {}
//  inline unsigned char endTransaction()                                                         { return 0; }
  inline void          setAddressFilter(char** anAddressList, unsigned int theTimeoutValue=10)  {}
  inline unsigned char getInputLatency()                                                        { return 0; } // TODO replace with equivalent?
//  inline void          diagnose(const __FlashStringHelper* reason =0)                           {}
//  inline void          setMode(unsigned char mode)                                              {}
//  inline unsigned char getMode()                                                                { return 0; }
#pragma GCC diagnostic pop
}; // class UsbSerialWrapper =========================================================================================================================

static  UsbSerialWrapper myUsbSerial(Serial);
UsbSerialWrapper& UsbSerialWrapper::ourSerialObject = myUsbSerial;


#define APP_FW_VERSION  "V1.0"
#define APP_IDENTIFIER  34

// we declare the weak method getBuildTime() which returns the compile time. This method can be overloaded externally with the real BuildTime (defined in boards.txt)
extern const __FlashStringHelper* getBuildTime() __attribute__((weak));
const __FlashStringHelper* getBuildTime()
{
  static const char __BuildTime__[] PROGMEM = "Compiled: " __DATE__ " " __TIME__;
  return reinterpret_cast<const __FlashStringHelper *> (&__BuildTime__[0]);
}


#include "MessageReader.h"
typedef MessageReader<32, UsbSerialWrapper> USBMessageReaderClass;

class ApplicationControl // ==========================================================================================================================
{
  // This class manages various communication parameters and aspects of the application

  // default initialization values
  const static unsigned char default_applicationType = APP_IDENTIFIER;
  const static unsigned char default_slaveId         = '$';
  const static unsigned char default_priority        = 0x1f;

  // saved on eeprom  
  unsigned char applicationType;
  unsigned char applicationInstance;
  unsigned char RS485_Priority;

  // eeprom addresses
  static unsigned char const A_ApplicationType       = 0;
  static unsigned char const A_ApplicationInstance   = 1;
  static unsigned char const A_Priority              = 2;

  public:


  inline unsigned char getApplicationInstance() { return applicationInstance; }
  inline void EEPROM_restore()
  {
      // read our 'identity' from the EEPROM
      applicationType = eeprom_read_byte((unsigned char*) A_ApplicationType);
      if(applicationType == default_applicationType)    // but only if the application_type matches the value of accepted_application_type.
      {
        setRS485_SlaveId        (eeprom_read_byte((unsigned char*) A_ApplicationInstance));
        setRS485_Priority       (eeprom_read_byte((unsigned char*) A_Priority));
      }
      else                                              // otherwise we will initialize our identity these default values:
      {
        applicationType          =default_applicationType;
        setRS485_SlaveId         (default_slaveId);
        setRS485_Priority        (default_priority);
      }
  }
  inline void EEPROM_save()
  {
      eeprom_write_byte((unsigned char*) A_ApplicationType,       applicationType);
      eeprom_write_byte((unsigned char*) A_ApplicationInstance,   applicationInstance);
      eeprom_write_byte((unsigned char*) A_Priority,              RS485_Priority);
  }
  inline void setRS485_SlaveId(char theSlaveId)                 { applicationInstance=theSlaveId; }
  inline void setRS485_Priority(unsigned char thePriority)      { RS485_Priority=thePriority <= 31 ?  thePriority : 31; }
  inline unsigned char getRS485Priority()                       { return RS485_Priority; }
  inline void startMessage(unsigned char aPriority=0xff)        { Serial.print((char) USBMessageReaderClass::SOM); }
  inline unsigned char endMessage()                             { Serial.print((char) USBMessageReaderClass::EOM); Serial.println(); }

  void printAliveMessage()
  {
      Serial.print(F("RS485_USB alive. Version "));     Serial.print(F(APP_FW_VERSION));
      Serial.print(F("("));                             Serial.print( getBuildTime() );
      Serial.print(F(") my applicationInstance="));     Serial.print(applicationInstance);
      Serial.print(F(" RS485_priority="));              Serial.print(RS485_Priority);
  }
};

ApplicationControl myApplicationControl;



static char*  matchAllMessages[] = {(char*)&matchAllMessages[1], 0};    // aka = {(char*)"", 0};    // address list that matches all well formed messages
static char** passAllData = 0;                                                                      // address list that switches off filtering

USBMessageReaderClass usbMessageReader(matchAllMessages, 20); // N.B., parameters are passed to MySerialClass before the run of setup.


void setup()
{
    myApplicationControl.EEPROM_restore();  // read our identity from the EEPROM

//  initialize (USB) Serial and (RS485) Serial1 communication
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial1.setMode(0x11);                  // set Serial1 into RS485 mode
    Serial1.setAddressFilter(passAllData);  // disable filter

//  report to the USB Serial port
    myApplicationControl.printAliveMessage();
}

void loop()
{
    // check for USB messages
    unsigned char state = usbMessageReader.getState();
    if( usbMessageReader.isMessage(state) )
    {
        char* messageParser      = usbMessageReader.getMessage();

        // convenience macros for readability and to avoid typing errors, using progmem for test strings //TODO, consider to include in MessageReader Class??
        #define startWith( _message_,_string_)  (!strncmp_P((_message_),PSTR(_string_), sizeof((_string_))-1) )
        #define canRemove( _message_,_string_)  (!strncmp_P((_message_),PSTR(_string_), sizeof((_string_))-1) ? (_message_ += sizeof((_string_))-1, true) : false)

        if(messageParser[0]==myApplicationControl.getApplicationInstance() )
        {
            // messages starting our applicationInstance identifier configure the gateway
            messageParser++; // skip the id

            if(false); // lekker puh
            else if(startWith(messageParser,"?"))              { myApplicationControl.printAliveMessage(); }
            else if(canRemove(messageParser,"SLID="))          { myApplicationControl.setRS485_SlaveId(messageParser[0]); }
            else if(canRemove(messageParser,"PRIO="))          { myApplicationControl.setRS485_Priority(atoi(messageParser)); }
            else if(canRemove(messageParser,"EEPROM."))
            {    if(false) {} //lekker puh
                 else if(startWith(messageParser,"SAVE"))      { myApplicationControl.EEPROM_save(); }
                 else if(startWith(messageParser,"RESTORE"))   { myApplicationControl.EEPROM_restore(); }
            }
//          else if(startWith(messageParser,"Memstat"))        { myApplicationControl.startMessage(); PrintMemstat(Serial);        myApplicationControl.endMessage(); }
            else if(canRemove(messageParser,"Echo:"))          { myApplicationControl.startMessage(); Serial.print(messageParser); myApplicationControl.endMessage(); }
    
            else if(canRemove(messageParser,"M485="))          { Serial1.setMode( atoi(messageParser));
                                                                 myApplicationControl.startMessage(); Serial.print(F("RS485Mode=")); Serial.print(atoi(messageParser),HEX); myApplicationControl.endMessage(); }
        }
        else
        {
            // we send this message onto RS485
            Serial1.startTransaction(myApplicationControl.getRS485Priority() );
            Serial1.print((char) USBMessageReaderClass::SOM);
            Serial1.print(messageParser);
            Serial1.print((char) USBMessageReaderClass::EOM);
            Serial1.println();
            Serial1.endTransaction();  // TODO use non blocking end transaction
        }
        
        usbMessageReader.nextMessage();
    }

    // read data from RS485 and send it to the USB Serial port
    while (Serial1.available() > 0)
    {
      char c = Serial1.read();
      Serial.write(c);
    }
}