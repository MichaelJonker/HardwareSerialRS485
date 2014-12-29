/* Morse.cpp copyright notice

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

#include "Morse.h"

const char Morse::helloWorld[]          = ".... . .-.. .-.. ---"; //  .-- --- .-. .-.. -.. -.-.--";
const char Morse::hoiOuweRukker[]       = ".... --- ..  --- .-- .  .-. ..- -.- -.- . .-.";
const char Morse::heiDikkeLulDrieBier[] = ".... . ..  -.. .. -.- -.- .  .-.. .. .-.. .  -.. .-. .. .  -... .. . .-.";

void Morse::send(const char *str, const int ledPin, const int quantum)
{
//  Serial.print("Mors::send(\""); Serial.print(str); Serial.print("\","); Serial.print(ledPin); Serial.print(quantum); Serial.print(")\n");
    while(*str != '\0')
    {
        if(false) ; // lekker puh
        else if(*str == ' ') { delay(2*quantum); } // space();
        else if(*str == '.') { digitalWrite(ledPin, HIGH); delay(  quantum); digitalWrite(ledPin, LOW); delay(quantum); } //dot();
        else if(*str == '-') { digitalWrite(ledPin, HIGH); delay(3*quantum); digitalWrite(ledPin, LOW); delay(quantum); } //dash();
        else break;
        str++;
    }
}

void Morse::send(const int *data, const int ledPin, const int quantum)
{
// binary encoding: morse characters (2 bits) are stored little endian in consecutive int's. 
//                  00: end; 01: space; 10: dot; 11: dash;
    while(int test = *data++)
    {
        do
        {
            switch (test&0x03)
            {
                case 0x03: dash (ledPin, quantum); break;
                case 0x02: dot  (ledPin, quantum); break;
                case 0x01: space(        quantum); break;
                default: return;
            }
            test=test>>2; // shift down to get the next nibble
        }
        while (test!=0);
    }
}


const unsigned char Morse::morseAlphabeth[] = {
//TODO find a way to F() this
0x01, // 20  (space)          
0xD7, // 21  !     -.-.--     
0xA5, // 22  "     .-..-.     
0x00, // 23  #                  
0x93, // 24  $     ...-..- note: replaced by ..-..-      
0x00, // 25  %                
0x51, // 26  &     .-...      
0xBD, // 27  '     .----.     
0x6D, // 28  (     -.--.      
0xDB, // 29  )     -.--.-     
0x00, // 2A  *                
0x55, // 2B  +     .-.-.      
0xE7, // 2C  ,     --..--     
0xC3, // 2D  -     -....-     
0xAB, // 2E  .     .-.-.-     
0x65, // 2F  /     -..-.      
0x7F, // 30  0     -----      
0x5F, // 31  1     .----      
0x4F, // 32  2     ..---      
0x47, // 33  3     ...--      
0x43, // 34  4     ....-      
0x41, // 35  5     .....      
0x61, // 36  6     -....      
0x71, // 37  7     --...      
0x79, // 38  8     ---..      
0x7D, // 39  9     ----.      
0xF1, // 3A  :     ---...     
0xD5, // 3B  ;     -.-.-.     
0x00, // 3C  <                
0x63, // 3D  =     -...-      
0x00, // 3E  >                
0x99, // 3F  ?     ..--..     
0xB5, // 40  @   .--.-.       
0x0B, // 41  A   .-           
0x31, // 42  B   -...         
0x35, // 43  C   -.-.         
0x19, // 44  D   -..          
0x05, // 45  E   .            
0x13, // 46  F   ..-          
0x1D, // 47  G   --.          
0x21, // 48  H   ....         
0x09, // 49  I   ..           
0x2F, // 4A  J   .---         
0x1B, // 4B  K   -.-          
0x29, // 4C  L   .-..         
0x0F, // 4D  M   --           
0x0D, // 4E  N   -.           
0x1F, // 4F  O   ---          
0x2D, // 50  P   .--.         
0x3B, // 51  Q   --.-         
0x15, // 52  R   .-.          
0x11, // 53  S   ...          
0x07, // 54  T   -            
0x13, // 55  U   ..-          
0x23, // 56  V   ...-         
0x17, // 57  W   .--          
0x33, // 58  X   -..-         
0x37, // 59  Y   -.--         
0x39, // 5A  Z   --..         
0x00, // 5B  [                
0x00, // 5C  '\'                
0x00, // 5D  ]                
0x00, // 5E  ^                
0x9B // 5F  _  ..--.-        
};

unsigned char Morse::getMorseCode(unsigned char input)
{
  if     (input >= ' ' and input <= 'Z') return morseAlphabeth[input-0x20];
  else if(input >= 'a' and input <= 'z') return morseAlphabeth[input-0x40];
  else                                   return 0;
}

void Morse::sendMorseCode(unsigned char theMorseCode, boolean verbose)
{
    boolean active;
    unsigned char mask;
    /* the code handles up to 7 bit characters but cannot create continuous streams *_remove_me_to_activate_code_/
    static const char maskEndValue=0x00; // */
    /* The code handles up to 6 bits characters */
    // one bit is used to control the terminating space, hence only up to 6 bit characters can be created. In
    // exchange one may create continuous streams.
    // The '$'character has been replaced by the code '..-..-', alternatively it can be represented by V-no-space U
    static const char maskEndValue=0x01; // */

    for (active=false, mask=0x80; mask!=maskEndValue; mask = mask>>1)
    {
        if(!active) active = theMorseCode&mask;
        else if(theMorseCode&mask) { dash(); if(verbose) Serial.print("-"); }
        else                       { dot();  if(verbose) Serial.print("."); }
    }
    if((theMorseCode&maskEndValue)==maskEndValue) { space(); if(verbose) Serial.print(" "); }
}

void Morse::sendMessage(char* theMessage, boolean verbose)
{
    if(verbose)
    {
      Serial.print(F("Morse::sendMessage("));
      Serial.print(theMessage);
      Serial.print(F(")\n code: "));
    }
    sendMessage(theMessage);
}
