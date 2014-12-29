/* Morse.h copyright notice

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

#ifndef Morse_h // Morse class ====================================================================
#define Morse_h

#include "HardwareSerial_RS485.h"
#include <Arduino.h>

/* morse code
http://en.wikipedia.org/wiki/Morse_code
http://nl.wikipedia.org/wiki/Morse
{41,5A} 26x4   2x1 , 4x2, 8x3, 12x4
20  (space)          40  @   .--.-.         60  `
21  !     -.-.--     41  A   .-             61  a
22  "     .-..-.     42  B   -...           62  b
23  #                43  C   -.-.           63  c
24  $     ...-..-    44  D   -..            64  d
25  %                45  E   .              65  e
26  &     .-...      46  F   ..-            66  f
27  '     .----.     47  G   --.            67  g
28  (     -.--.      48  H   ....           68  h
29  )     -.--.-     49  I   ..             69  i
2A  *                4A  J   .---           6A  j
2B  +     .-.-.      4B  K   -.-            6B  k
2C  ,     --..--     4C  L   .-..           6C  l
2D  -     -....-     4D  M   --             6D  m
2E  .     .-.-.-     4E  N   -.             6E  n
2F  /     -..-.      4F  O   ---            6F  o
30  0     -----      50  P   .--.           70  p
31  1     .----      51  Q   --.-           71  q
32  2     ..---      52  R   .-.            72  r
33  3     ...--      53  S   ...            73  s
34  4     ....-      54  T   -              74  t
35  5     .....      55  U   ..-            75  u
36  6     -....      56  V   ...-           76  v
37  7     --...      57  W   .--            77  w
38  8     ---..      58  X   -..-           78  x
39  9     ----.      59  Y   -.--           79  y
3A  :     ---...     5A  Z   --..           7A  z
3B  ;     -.-.-.     5B  [                  7B  {
3C  <                5C  \                  7C  |
3D  =     -...-      5D  ]                  7D  }
3E  >                5E  ^                  7E  ~
3F  ?     ..--..     5F  _  ..--.-                
*/

class Morse
{
    int myQuantum;
    int myLedPin;
    const static char  morse_encoding[];

    public:

    const static char helloWorld[];
    const static char hoiOuweRukker[];
    const static char heiDikkeLulDrieBier[];

    // binary encoding: Morse characters (2 bits) are packed little endian in consecutive int's - 00: end; 01: space; 10: dot; 11: dash;
    static void send(const int *data, const int ledPin, const int quantum=100);

    // character encoding: Morse characters are stored in consecutive char's - '\0': end; ' ': space; '.': dot; '-': dash;
    static void send(const char *str, const int ledPin, const int quantum=100);

    inline static void dash (const int ledPin, const int quantum=100) {digitalWrite(ledPin, HIGH); delay(3*quantum); digitalWrite(ledPin, LOW); delay(quantum);}
    inline static void dot  (const int ledPin, const int quantum=100) {digitalWrite(ledPin, HIGH); delay(  quantum); digitalWrite(ledPin, LOW); delay(quantum);}
    inline static void space(                  const int quantum=100) {delay(quantum);}
	
    inline Morse(const int ledPin, int quantum=100) { pinMode(myLedPin =    ledPin, OUTPUT); myQuantum = quantum; }
    inline void setLedPin (int newLedPin)           { pinMode(myLedPin = newLedPin, OUTPUT); }
    inline void setQuantum(int newQuantum)          { myQuantum = newQuantum; }

    inline void send(const char *str)               { send (str, myLedPin, myQuantum); }
    inline void dash()                              { dash (myLedPin, myQuantum); }
    inline void dot()                               { dot  (myLedPin, myQuantum); }

    static const unsigned char morseAlphabeth[];
    static unsigned char getMorseCode(unsigned char input);
    void sendMorseCode(unsigned char theMorseCode, boolean verbose=true);

    void sendMessage(char* theMessage, boolean verbose);
    inline void sendMessage(char* theMessage) { while(*theMessage!=0) { sendMorseCode(getMorseCode(*theMessage)); theMessage++; } }

};

#endif // Morse class ==============================================================================

