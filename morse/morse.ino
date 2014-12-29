/* morse.ino copyright notice

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

/* Description
   Reading a serial ASCII-encoded string and converts it to morse.
   Michael Jonker 20140123
*/

#include <HardwareSerial_RS485.h>
#include <Morse.h>
#include <MessageFilter.h>

const unsigned char led1 = 12;  // defines the output LED pin for the morse codes


Morse myMorse(led1, 120);

// TODO put into a structure
unsigned char APP_TYPE;
unsigned char APP_INST;
unsigned char RS485_Priority;
unsigned char RS323_GracePeriod;
unsigned char const RS323_GraceFactor = 128;
unsigned int  RS323_GraceCredit;
char          myHeader[6] = "{M*S*";
char*         myAddress   = &myHeader[3];

#define APP_FW_VERSION F("V2.0")
#define APP_FW_DATE    F(__DATE__)
#define APP_FW_TIME    F(__TIME__)
//TODO try to make this work:
//const __FlashStringHelper* const APP_FW_VERSION = F("V2.0");
//const __FlashStringHelper* const APP_FW_DATE    = F(__DATE__); // fill in the date with a macro
//const __FlashStringHelper* const APP_FW_TIME    = F(__TIME__); // fill in the date with a macro
// TODO find a way to obtain linking date and time instead


#include "CommunicationParameters.h"
typedef CommunicationParameters<led1> comCtrl;

#include <avr/eeprom.h>

unsigned char* const EEPROM_A_APP_TYPE           = (unsigned char*) 0;
unsigned char* const EEPROM_A_APP_INST           = (unsigned char*) 1;
unsigned char* const EEPROM_A_Priority           = (unsigned char*) 2;
unsigned char* const EEPROM_A_RS323_GracePeriod  = (unsigned char*) 3;
unsigned int*  const EEPROM_A_485Stat            = (unsigned int*)  2; // = char* 4 // todo remove 485 Stat?
void EEPROM_Restore()
{
    // read our identity from the EEPROM
    APP_TYPE          = eeprom_read_byte(EEPROM_A_APP_TYPE);
    APP_INST          = eeprom_read_byte(EEPROM_A_APP_INST);
    RS485_Priority    = eeprom_read_byte(EEPROM_A_Priority);
    RS323_GracePeriod = eeprom_read_byte(EEPROM_A_RS323_GracePeriod);

    // todo: setRS485SlaveId(eeprom_read_byte(EEPROM_A_APP_TYPE));
    char code = (APP_INST&0x1f)+0x40;
    if(code<'A' or code>'Z') code='*';
    myAddress[1]=code;

    RS323_GraceCredit = RS323_GracePeriod*RS323_GraceFactor;
}

void EEPROM_Save()
{
    eeprom_write_byte(EEPROM_A_APP_INST,          APP_INST);
    eeprom_write_byte(EEPROM_A_Priority,          RS485_Priority);
    eeprom_write_byte(EEPROM_A_RS323_GracePeriod, RS323_GracePeriod);
}

void EEPROM_Save485(int st1, int st2, int st3)
{
    int count = eeprom_read_word(EEPROM_A_485Stat)+1;
    eeprom_write_word(EEPROM_A_485Stat  , count);
    eeprom_write_word(EEPROM_A_485Stat+1, st1);
    eeprom_write_word(EEPROM_A_485Stat+2, st2);
    eeprom_write_word(EEPROM_A_485Stat+3, st3);
}

void EEPROM_Print485()
{
  Serial.print(F("dump of 485 data from previous life: "));
  Serial.print( eeprom_read_word(EEPROM_A_485Stat  ), HEX); Serial.print(F("/"));
  Serial.print( eeprom_read_word(EEPROM_A_485Stat+1), HEX); Serial.print(F("/"));
  Serial.print( eeprom_read_word(EEPROM_A_485Stat+2), HEX); Serial.print(F("/"));
  Serial.print( eeprom_read_word(EEPROM_A_485Stat+3), HEX); Serial.print(F("\n"));
}


void setRS485Priority(int thePriority)
{
  RS485_Priority=thePriority;
  Serial.setPriority(RS485_Priority);
}

void setRS485SlaveId(char theSlaveId)
{
    APP_INST=theSlaveId;

    char code = (APP_INST&0x1f)+0x40;
    if(code<'A' or code>'Z') code='*';
    myAddress[1]=code;
}

// todo move to filter class
//static char* blockData[]   = {0};
//static char** passAllData = 0;
//static char* matchAllMessages[]          = {(char*)&matchAllMessages[1], 0}; // aka = {(char*)"", 0};
//static char* anySlaveAddressList[]       = {(char*)"S", 0};
//static char* slaveBroadcastAddressList[] = {(char*)"S*", 0};

char* myExactAddressList[] = {myAddress, 0};
char* myAddressList[]      = {myAddress, (char*)"S*", 0};
MessageFilter<32, MTP_readable> myMessageFilter(myAddressList);

void printAliveMessage()
{
    Serial.print(myHeader); Serial.print(F(" MORSE alive. Version ")); Serial.print(APP_FW_VERSION);
    Serial.print(F("("));                   Serial.print(APP_FW_DATE);
    Serial.print(F(") RS485_priority="));   Serial.print(RS485_Priority);
    Serial.print(F(" RS323_GraceCredit=")); Serial.print(RS323_GraceCredit);
    Serial.print(F("/"));                   Serial.print((unsigned int) RS323_GracePeriod*RS323_GraceFactor);
    Serial.println("}");
}

void doPrintTest()
{
    Serial.println(myHeader);
    #ifndef tochMaarFFNiet
    Serial.print(F(" print((uc)0x0F)           =")); Serial.println((unsigned char)0xF, DEC      ); // should print
    Serial.print(F(" print((uc)0x0F, HEX)      =")); Serial.println((unsigned char)0xF, HEX      ); // should print
    Serial.print(F(" print((uc)0x0F, HEX, 0x07)=")); Serial.println((unsigned char)0xF, HEX, 0x07); // should print
    Serial.print(F(" print((uc)0x0F, HEX, 0x27)=")); Serial.println((unsigned char)0xF, HEX, 0x27); // should print
    Serial.print(F(" print((uc)0x0F, HEX, 0x47)=")); Serial.println((unsigned char)0xF, HEX, 0x47); // should print
    Serial.print(F(" print((uc)0x0F, HEX, 0x67)=")); Serial.println((unsigned char)0xF, HEX, 0x67); // should print
    Serial.print(F(" print((sc)0x0F)           =")); Serial.println((  signed char)0xF, DEC      ); // should print
    Serial.print(F(" print((sc)0x0F, HEX)      =")); Serial.println((  signed char)0xF, HEX      ); // should print
    Serial.print(F(" print((sc)0x0F, HEX, 0x07)=")); Serial.println((  signed char)0xF, HEX, 0x07); // should print
    Serial.print(F(" print((sc)0x0F, HEX, 0x27)=")); Serial.println((  signed char)0xF, HEX, 0x27); // should print
    Serial.print(F(" print((sc)0x0F, HEX, 0x47)=")); Serial.println((  signed char)0xF, HEX, 0x47); // should print
    Serial.print(F(" print((sc)0x0F, HEX, 0x67)=")); Serial.println((  signed char)0xF, HEX, 0x67); // should print

    Serial.print(F(" print((uc)0xFF)           =")); Serial.println((unsigned char)0xF0, DEC      ); // should print
    Serial.print(F(" print((uc)0xFF, HEX)      =")); Serial.println((unsigned char)0xF0, HEX      ); // should print
    Serial.print(F(" print((uc)0xFF, HEX, 0x07)=")); Serial.println((unsigned char)0xF0, HEX, 0x07); // should print
    Serial.print(F(" print((uc)0xFF, HEX, 0x27)=")); Serial.println((unsigned char)0xF0, HEX, 0x27); // should print
    Serial.print(F(" print((uc)0xFF, HEX, 0x47)=")); Serial.println((unsigned char)0xF0, HEX, 0x47); // should print
    Serial.print(F(" print((uc)0xFF, HEX, 0x67)=")); Serial.println((unsigned char)0xF0, HEX, 0x67); // should print
    Serial.print(F(" print((sc)0xFF)           =")); Serial.println((  signed char)0xF0, DEC      ); // should print
    Serial.print(F(" print((sc)0xFF, HEX)      =")); Serial.println((  signed char)0xF0, HEX      ); // should print
    Serial.print(F(" print((sc)0xFF, HEX, 0x07)=")); Serial.println((  signed char)0xF0, HEX, 0x07); // should print
    Serial.print(F(" print((sc)0xFF, HEX, 0x27)=")); Serial.println((  signed char)0xF0, HEX, 0x27); // should print
    Serial.print(F(" print((sc)0xFF, HEX, 0x47)=")); Serial.println((  signed char)0xF0, HEX, 0x47); // should print
    Serial.print(F(" print((sc)0xFF, HEX, 0x67)=")); Serial.println((  signed char)0xF0, HEX, 0x67); // should print

    Serial.print(F(" print((ui)0x0FFF)           =")); Serial.println((unsigned int)0xF00, DEC      ); // should print
    Serial.print(F(" print((ui)0x0FFF, HEX)      =")); Serial.println((unsigned int)0xF00, HEX      ); // should print
    Serial.print(F(" print((ui)0x0FFF, HEX, 0x07)=")); Serial.println((unsigned int)0xF00, HEX, 0x07); // should print
    Serial.print(F(" print((ui)0x0FFF, HEX, 0x27)=")); Serial.println((unsigned int)0xF00, HEX, 0x27); // should print
    Serial.print(F(" print((ui)0x0FFF, HEX, 0x47)=")); Serial.println((unsigned int)0xF00, HEX, 0x47); // should print
    Serial.print(F(" print((ui)0x0FFF, HEX, 0x67)=")); Serial.println((unsigned int)0xF00, HEX, 0x67); // should print
    Serial.print(F(" print((si)0x0FFF)           =")); Serial.println((  signed int)0xF00, DEC      ); // should print
    Serial.print(F(" print((si)0x0FFF, HEX)      =")); Serial.println((  signed int)0xF00, HEX      ); // should print
    Serial.print(F(" print((si)0x0FFF, HEX, 0x07)=")); Serial.println((  signed int)0xF00, HEX, 0x07); // should print
    Serial.print(F(" print((si)0x0FFF, HEX, 0x27)=")); Serial.println((  signed int)0xF00, HEX, 0x27); // should print
    Serial.print(F(" print((si)0x0FFF, HEX, 0x47)=")); Serial.println((  signed int)0xF00, HEX, 0x47); // should print
    Serial.print(F(" print((si)0x0FFF, HEX, 0x67)=")); Serial.println((  signed int)0xF00, HEX, 0x67); // should print
 
    Serial.print(F(" print((ui)0xFFFF)           =")); Serial.println((unsigned int)0xF000, DEC      ); // should print
    Serial.print(F(" print((ui)0xFFFF, HEX)      =")); Serial.println((unsigned int)0xF000, HEX      ); // should print
    Serial.print(F(" print((ui)0xFFFF, HEX, 0x07)=")); Serial.println((unsigned int)0xF000, HEX, 0x07); // should print
    Serial.print(F(" print((ui)0xFFFF, HEX, 0x27)=")); Serial.println((unsigned int)0xF000, HEX, 0x27); // should print
    Serial.print(F(" print((ui)0xFFFF, HEX, 0x47)=")); Serial.println((unsigned int)0xF000, HEX, 0x47); // should print
    Serial.print(F(" print((ui)0xFFFF, HEX, 0x67)=")); Serial.println((unsigned int)0xF000, HEX, 0x67); // should print
    Serial.print(F(" print((si)0xFFFF)           =")); Serial.println((  signed int)0xF000, DEC      ); // should print
    Serial.print(F(" print((si)0xFFFF, HEX)      =")); Serial.println((  signed int)0xF000, HEX      ); // should print
    Serial.print(F(" print((si)0xFFFF, HEX, 0x07)=")); Serial.println((  signed int)0xF000, HEX, 0x07); // should print
    Serial.print(F(" print((si)0xFFFF, HEX, 0x27)=")); Serial.println((  signed int)0xF000, HEX, 0x27); // should print
    Serial.print(F(" print((si)0xFFFF, HEX, 0x47)=")); Serial.println((  signed int)0xF000, HEX, 0x47); // should print
    Serial.print(F(" print((si)0xFFFF, HEX, 0x67)=")); Serial.println((  signed int)0xF000, HEX, 0x67); // should print

    Serial.print(F(" print((ul)0x0FFF)           =")); Serial.println((unsigned long)0x0F000000, DEC      ); // should print
    Serial.print(F(" print((ul)0x0FFF, HEX)      =")); Serial.println((unsigned long)0x0F000000, HEX      ); // should print
    Serial.print(F(" print((ul)0x0FFF, HEX, 0x07)=")); Serial.println((unsigned long)0x0F000000, HEX, 0x07); // should print
    Serial.print(F(" print((ul)0x0FFF, HEX, 0x27)=")); Serial.println((unsigned long)0x0F000000, HEX, 0x27); // should print
    Serial.print(F(" print((ul)0x0FFF, HEX, 0x47)=")); Serial.println((unsigned long)0x0F000000, HEX, 0x47); // should print
    Serial.print(F(" print((ul)0x0FFF, HEX, 0x67)=")); Serial.println((unsigned long)0x0F000000, HEX, 0x67); // should print
    Serial.print(F(" print((sl)0x0FFF)           =")); Serial.println((  signed long)0x0F000000, DEC      ); // should print
    Serial.print(F(" print((sl)0x0FFF, HEX)      =")); Serial.println((  signed long)0x0F000000, HEX      ); // should print
    Serial.print(F(" print((sl)0x0FFF, HEX, 0x07)=")); Serial.println((  signed long)0x0F000000, HEX, 0x07); // should print
    Serial.print(F(" print((sl)0x0FFF, HEX, 0x27)=")); Serial.println((  signed long)0x0F000000, HEX, 0x27); // should print
    Serial.print(F(" print((sl)0x0FFF, HEX, 0x47)=")); Serial.println((  signed long)0x0F000000, HEX, 0x47); // should print
    Serial.print(F(" print((sl)0x0FFF, HEX, 0x67)=")); Serial.println((  signed long)0x0F000000, HEX, 0x67); // should print

    Serial.print(F(" print((ul)0xFFFF)           =")); Serial.println((unsigned long)0xF0000000, DEC      ); // should print
    Serial.print(F(" print((ul)0xFFFF, HEX)      =")); Serial.println((unsigned long)0xF0000000, HEX      ); // should print
    Serial.print(F(" print((ul)0xFFFF, HEX, 0x07)=")); Serial.println((unsigned long)0xF0000000, HEX, 0x07); // should print
    Serial.print(F(" print((ul)0xFFFF, HEX, 0x27)=")); Serial.println((unsigned long)0xF0000000, HEX, 0x27); // should print
    Serial.print(F(" print((ul)0xFFFF, HEX, 0x47)=")); Serial.println((unsigned long)0xF0000000, HEX, 0x67); // should print
    Serial.print(F(" print((ul)0xFFFF, HEX, 0x67)=")); Serial.println((unsigned long)0xF0000000, HEX, 0x67); // should print
    Serial.print(F(" print((sl)0xFFFF)           =")); Serial.println((  signed long)0xF0000000, DEC      ); // should print
    Serial.print(F(" print((sl)0xFFFF, HEX)      =")); Serial.println((  signed long)0xF0000000, HEX      ); // should print
    Serial.print(F(" print((sl)0xFFFF, HEX, 0x07)=")); Serial.println((  signed long)0xF0000000, HEX, 0x07); // should print
    Serial.print(F(" print((sl)0xFFFF, HEX, 0x27)=")); Serial.println((  signed long)0xF0000000, HEX, 0x27); // should print
    Serial.print(F(" print((sl)0xFFFF, HEX, 0x47)=")); Serial.println((  signed long)0xF0000000, HEX, 0x67); // should print
    Serial.print(F(" print((sl)0xFFFF, HEX, 0x67)=")); Serial.println((  signed long)0xF0000000, HEX, 0x67); // should print
#endif
#ifdef output
 print((uc)0x0F)           =15
 print((uc)0x0F, HEX)      =F
 print((uc)0x0F, HEX, 0x07)=       F
 print((uc)0x0F, HEX, 0x27)=0000000F
 print((uc)0x0F, HEX, 0x47)=       +F
 print((uc)0x0F, HEX, 0x67)=+0000000F
 print((sc)0x0F)           =15
 print((sc)0x0F, HEX)      =F
 print((sc)0x0F, HEX, 0x07)=       F
 print((sc)0x0F, HEX, 0x27)=0000000F
 print((sc)0x0F, HEX, 0x47)=       +F
 print((sc)0x0F, HEX, 0x67)=+0000000F
 print((uc)0xFF)           =240
 print((uc)0xFF, HEX)      =F0
 print((uc)0xFF, HEX, 0x07)=      F0
 print((uc)0xFF, HEX, 0x27)=000000F0
 print((uc)0xFF, HEX, 0x47)=      +F0
 print((uc)0xFF, HEX, 0x67)=+000000F0
 print((sc)0xFF)           =-16
 print((sc)0xFF, HEX)      =-10
 print((sc)0xFF, HEX, 0x07)=     -10
 print((sc)0xFF, HEX, 0x27)=-0000010
 print((sc)0xFF, HEX, 0x47)=      -10
 print((sc)0xFF, HEX, 0x67)=-00000010
 print((ui)0x0FFF)           =3840
 print((ui)0x0FFF, HEX)      =F00
 print((ui)0x0FFF, HEX, 0x07)=     F00
 print((ui)0x0FFF, HEX, 0x27)=00000F00
 print((ui)0x0FFF, HEX, 0x47)=     +F00
 print((ui)0x0FFF, HEX, 0x67)=+00000F00
 print((si)0x0FFF)           =3840
 print((si)0x0FFF, HEX)      =F00
 print((si)0x0FFF, HEX, 0x07)=     F00
 print((si)0x0FFF, HEX, 0x27)=00000F00
 print((si)0x0FFF, HEX, 0x47)=     +F00
 print((si)0x0FFF, HEX, 0x67)=+00000F00
 print((ui)0xFFFF)           =61440
 print((ui)0xFFFF, HEX)      =F000
 print((ui)0xFFFF, HEX, 0x07)=    F000
 print((ui)0xFFFF, HEX, 0x27)=0000F000
 print((ui)0xFFFF, HEX, 0x47)=    +F000
 print((ui)0xFFFF, HEX, 0x67)=+0000F000
 print((si)0xFFFF)           =-4096
 print((si)0xFFFF, HEX)      =-1000
 print((si)0xFFFF, HEX, 0x07)=   -1000
 print((si)0xFFFF, HEX, 0x27)=-0001000
 print((si)0xFFFF, HEX, 0x47)=    -1000
 print((si)0xFFFF, HEX, 0x67)=-00001000
 print((ul)0x0FFF)           =251658240
 print((ul)0x0FFF, HEX)      =F000000
 print((ul)0x0FFF, HEX, 0x07)= F000000
 print((ul)0x0FFF, HEX, 0x27)=0F000000
 print((ul)0x0FFF, HEX, 0x47)= +F000000
 print((ul)0x0FFF, HEX, 0x67)=+0F000000
 print((sl)0x0FFF)           =251658240
 print((sl)0x0FFF, HEX)      =F000000
 print((sl)0x0FFF, HEX, 0x07)= F000000
 print((sl)0x0FFF, HEX, 0x27)=0F000000
 print((sl)0x0FFF, HEX, 0x47)= +F000000
 print((sl)0x0FFF, HEX, 0x67)=+0F000000
 print((ul)0xFFFF)           =4026531840
 print((ul)0xFFFF, HEX)      =F0000000
 print((ul)0xFFFF, HEX, 0x07)=F0000000
 print((ul)0xFFFF, HEX, 0x27)=F0000000
 print((ul)0xFFFF, HEX, 0x47)=+F0000000
 print((ul)0xFFFF, HEX, 0x67)=+F0000000
 print((sl)0xFFFF)           =-268435456
 print((sl)0xFFFF, HEX)      =-10000000
 print((sl)0xFFFF, HEX, 0x07)=-10000000
 print((sl)0xFFFF, HEX, 0x27)=-10000000
 print((sl)0xFFFF, HEX, 0x47)=-10000000
 print((sl)0xFFFF, HEX, 0x67)=-10000000
}
#endif
    Serial.println("}");
}


int PrintMemstat()
{
    /* report on: */
    extern char *__brkval;
    extern char __heap_start;
//  extern char __heap_end;               // note __heap_end equals 0, i.e. SP is effectively used for heap end 
//  extern size_t __malloc_margin;        // actual value used by malloc (if changed by user befor first allocation)
//  extern char *__malloc_heap_start;     // idem
//  extern char *__malloc_heap_end;       // idem
    unsigned int bv   = (int) (__brkval == 0 ? &__heap_start : __brkval); // effective break value
    unsigned int free = ((unsigned int) SP) - bv;                         // subtracted from free are the 4 bytes for local variables
//  Note local variables are pre-reserved on the stack. Hence there is no difference to read the stack pointer before or after declaration of local variables

//  The SP grows towards smaller values, starting from ??.
    Serial.print(myHeader); 
    Serial.print(F("MemStat: "));         Serial.flush();
    Serial.print(F(" &__heap_start=0x")); Serial.print((int)&__heap_start, HEX, 0x23); Serial.flush();
    Serial.print(F(" __brkval=0x"));      Serial.print(bv,                 HEX, 0x23); Serial.flush();
    Serial.print(F(" stackPointer=0x"));  Serial.print(SP,                 HEX, 0x23); Serial.flush();
    Serial.print(F(" free=0x"));          Serial.print(free,               HEX, 0x3); Serial.flush();
    Serial.println("}");                                                               Serial.flush();
    return free; 
}

void setup()
{

    EEPROM_Restore();  // read our identity from the EEPROM

    Serial.setAdressFilter(myAddressList);
    Serial.setPriority(RS485_Priority);
//  initialize serial communication at 9600 bits per second:
    Serial.begin(9600);

    myMorse.send(Morse::helloWorld);

//  report to the serial port
    printAliveMessage();
    EEPROM_Print485();
    PrintMemstat();
    Serial.flush();

//  check the 232 port for local communication, if not go to RS485 communication
    if (Serial.available() == 0) comCtrl::activateRS485();

    if( comCtrl::isVerboseLevel(0x80) ) { Serial.print(myHeader); Serial.print(F("_UP: ")); Serial.println(millis(),DEC,0x6); Serial.flush(); }
}

void loop()
{
    Serial.flush();  // take a fresh start

    unsigned char state = myMessageFilter.getState(10);
    if( myMessageFilter.isMessage(state) )
    {
        char* messageParser      = myMessageFilter.getBuffer();
        char* messageDestination = myMessageFilter.getMessageDestination();
//      char  messageLength      = myMessageFilter.getMessageLength();

// todo (someday) add message reception statistics


        if( comCtrl::isVerboseLevel(0x04) )
        {
          Serial.print(F("MessageFilterState="));  Serial.print(state,HEX);  // Serial.print(myMessageFilter.getState(),HEX);
          Serial.print(F(" messageDestination=")); Serial.print(messageDestination);
          Serial.print(F(" messageParser="));      Serial.println(messageParser);
          Serial.flush();
        }

        // convenience macros for readability and to avoid typing errors, using progmem for test strings //TODO, part of MessageFilterClass??
        #define startWith( _message_,_string_)  (!strncmp_P((_message_),PSTR(_string_), sizeof((_string_))-1) )
        #define canRemove( _message_,_string_)  (!strncmp_P((_message_),PSTR(_string_), sizeof((_string_))-1) ? (_message_ += sizeof((_string_))-1, true) : false)

        /* NOTE to get rid of progmem warnings (in case you have set verbode mode in your Arduino IDE)
           edit the file ... arduino-1.0.5\hardware\tools\avr\avr\include\avr\pgmspace.h
           and replace the line # define PSTR(s) (__extension__({static char __c[] PROGMEM = (s); &__c[0];}))
           with                 # define PSTR(s) (__extension__({static char __c[] __attribute__(( section(".progmem.data") )) = (s); &__c[0];}))
           ( Alternatively you can add these two lines to your code: (which fixes the warning only for the module containing these lines)
             #undef PROGMEM
             #define PROGMEM __attribute__(( section(".progmem.data") ))
           ( Second alternatively locate the place containing #define PROGMEM, and replace the definition with: #define PROGMEM __attribute__(( section(".progmem.data") ))
         */

/* command examples 
{SA}{SAEEPROM.PRINT}{SAMemstat}
{SAEEPROM.RESTORE}
{SAmorse:Hoi ouwe rukker}
{SAVBL?}
{SARS485}
{SARS232}
*/
        if(false); // lekker puh
        else if(*messageParser=='\0')                      { Serial.print(myHeader); Serial.println(F("}")); }
        else if(startWith(messageParser,"?"))              { printAliveMessage(); }
        else if(canRemove(messageParser,"PRIO="))          { setRS485Priority(atoi(messageParser)); }
        else if(canRemove(messageParser,"GRP="))           { RS323_GracePeriod=atoi(messageParser)/RS323_GraceFactor; }
        else if(canRemove(messageParser,"SID="))           { setRS485SlaveId(messageParser[0]); }
        else if(canRemove(messageParser,"LDY="))           { comCtrl::setLoopDelay(atoi(messageParser)); }
        else if(canRemove(messageParser,"HBM="))           { comCtrl::setHartBeatMask(atoi(messageParser)); }
        else if(canRemove(messageParser,"BRM="))           { comCtrl::setBreathingMask(atoi(messageParser)); }
        else if(canRemove(messageParser,"VBL="))           { comCtrl::setVerboseLevel(atoi(messageParser)); /* Serial.print(F("VerboseLevel=")); Serial.print(atoi(messageParser)); Serial.print(F("/0x")); Serial.println( comCtrl::getVerboseLevel(),HEX); */}
        else if(startWith(messageParser,"VBL?"))           { Serial.print(F("VerboseLevel=0x")); Serial.println( comCtrl::getVerboseLevel(),HEX); }
        else if(startWith(messageParser,"Memstat"))        { Serial.print(myHeader); PrintMemstat(); Serial.println(F("}")); }
        else if(canRemove(messageParser,"EEPROM."))
        {    if(false) {} //lekker puh
             else if(startWith(messageParser,"PRINT"))     { EEPROM_Print485(); }
             else if(startWith(messageParser,"SAVE"))      { EEPROM_Save(); }
             else if(startWith(messageParser,"RESTORE"))   { EEPROM_Restore(); }
        }
        else if(canRemove(messageParser,"morse:"))         { myMorse.sendMessage(messageParser, 1); }
        else if(canRemove(messageParser,"M485="))          { comCtrl::setModeRS485( atoi(messageParser)); Serial.print(F("RS485Mode=")); Serial.println(atoi(messageParser),HEX); }
        else if(startWith(messageParser,"RS485"))          { comCtrl::activateRS485(); }
        else if(startWith(messageParser,"RS232"))          { comCtrl::activateRS232(); }
        else if(startWith(messageParser,"PTEST"))          { doPrintTest(); }
#ifdef desperate
        else if(canRemove(messageParser,"RStest"))
        {
          static int testOption=0;
          if(canRemove(messageParser,"="))testOption = atoi(messageParser);

          Serial.print(F("test=")); Serial.println(testOption);
          Serial.flush();

          Serial.reset();
          comCtrl::activateRS485();
          Serial.reset();
          if     (testOption/4 == 0) {} // niks
          else if(testOption/4 == 1) Serial.print("a");
          else if(testOption/4 == 2) Serial.print("ab");
          else if(testOption/4 == 3) Serial.print("abc");
          else if(testOption/4 == 4) Serial.println(F("{SAhello...}"));
          else if(testOption/4 == 5) { Serial.println(F("{SAhoi hoi...}")); Serial.flush(); Serial.println(F("{SAdoei, doei...}"));}
          else                       { PrintMemstat(); Serial.flush(); Serial.println(F("{SAdoei, doei...}"));}
          if( (testOption&1) == 1)   delay(100);
          if( (testOption&2) == 2)   Serial.flush();
          Serial.diagnose();
          testOption++;
          
          comCtrl::activateRS232();
        }
        #endif // desperate

//      unrecognized command:
        else if( comCtrl::isVerboseLevel(-1) ) { Serial.print(myHeader); Serial.print(F("_?:")); Serial.println(messageParser); }
        myMessageFilter.nextMessage();

        if( comCtrl::isRS232Mode() )
        {
          RS323_GraceCredit = RS323_GracePeriod*RS323_GraceFactor; // we got a message, reset the grace period for RS232 communications

          int pending_1 = Serial.pending();
          Serial.flush();
          int pending_2 = Serial.pending();
          if( comCtrl::isVerboseLevel(0x08) )
          {
            Serial.print(myHeader); Serial.print(F("pending: ")); Serial.print(pending_1); Serial.print(F(", ")); Serial.print(pending_2); Serial.println(F("}"));
            Serial.flush();
          }
        }

        return; // so we can read next message if needed
    }

    if( comCtrl::isRS232Mode() )
    {
      if(RS323_GracePeriod)
      {
        RS323_GraceCredit--;
        if(RS323_GraceCredit==0) comCtrl::activateRS485();
      }
    }

    if( comCtrl::doLoopDelay() )
    {
        Serial.print(myHeader); Serial.print(F("HB: ")); Serial.print((unsigned int)comCtrl::getLoopCount(),Print::DEC,8);Serial.println("}");
        Serial.flush();
    }
}
