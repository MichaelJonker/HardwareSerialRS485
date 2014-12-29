/* CommunicationParameters.cpp copyright notice

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

#include "CommunicationParameters.h"

template<> CommunicationParameters<>  CommunicationParameters<>::rs232Communication(0, 0x25, 20, 0x00ff, HBM_Normal_Tonic, 0x00ff);
template<> CommunicationParameters<>  CommunicationParameters<>::rs485Communication(1, 0x11, 20, 0x0080, HBM_Normal_Soft,  0xffff);
template<> CommunicationParameters<>* CommunicationParameters<>::activeCommunicationParameters = &CommunicationParameters::rs232Communication;
template<> unsigned long CommunicationParameters<>::ourLoopCount = class_init();

/*
    0x10 setPermaRead();
    0x20 clrPermaRead();
    0x01 setWriteLocalEcho(); // write with dataCheck
    0x02 setWriteLocalEcho();
    0x03 setWriteNoLocalEcho();
    0x04 setPermaWrite();
    0x05 setPermaSilence();

    0x24 read from 232, write on 232 and 485 (Tx permanent enabled)
    0x23 read from 232, write on 232 and 485 (Tx enabled on as neeed)
    0x15 read from 485, write on 232
    0x13 read from 485, write on 485, no collision check
    0x11 read from 485, write on 485, collision check
    0x12 read from 485, write on 485, local echo, no collision check !warning, use only you have a 'solid' filter to dicard the local echo from the input.
*/

template<> size_t CommunicationParameters<>::printTo(Print& p) const
{
  Serial.print(getName());
  Serial.print(F(" communication. (485 mode="));
  Serial.print(myRS485Mode,HEX);
  Serial.print(F(")")); return 1;
}

// static initializer
template<unsigned char T_HBled> unsigned long CommunicationParameters<T_HBled>::class_init()
{
  pinMode(T_HBled, OUTPUT);      // initialize the digital pin as an output.
  return 0;
  // note: we cannot use delay in here
}

