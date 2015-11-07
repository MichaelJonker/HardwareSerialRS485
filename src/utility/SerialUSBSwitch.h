/* SerialUSBSwitch.h copyright notice

Switch implementation of HardwareSerial allowing to switch all methods between two HardwareSerial classes

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

#if !defined(SerialUSBSwitch_h)
#define      SerialUSBSwitch_h

/* Description
The SerialUSBSwitch.h provides an interface with functionality of the HardwareSerialRS485 class, which relays the methods to two underlying HardwareSerial objects, one HardwareSerialRS485, and a second stictly classical HardwareSerial.
The two underlying classes are passed by template parameter, the object of the second class are passed when the object is instantiated. (This is done as the classical HardwarSerial classes do not have a reference to the unique object)
The switching between the two underling classes is based on the mode setting of the underlying HardwareSerialRS485 in such a way that all output normally apearing on RS232 is redirected to the second class, whilst all RS485 communication is dealt with by the first class.
This class is specially created to support the HardwareSerialRS485 demo sketch to function on a Arduino Leonardo or Micro platform. In this case the first class is Serial1, while the second class is Serial_.
*/

#include <inttypes.h>


template <class T__Serial_A, class T__Serial_B>
class SerialUSBSwitch  : public Stream // ============================================================================================================
{
private:
    bool readA()  { return (mySerialA.getMode()&0xf0) == 0x10; } // i.e. if permaRead
    bool writeA() { return (mySerialA.getMode()&0x0f) != 0x05; } // i.e. if not permaSilence
    bool writeB() { return true; }

public:
//This is a singleton class (i.e. a class with only one object). We instantiate a reference here:
  static SerialUSBSwitch<T__Serial_A, T__Serial_B>& ourSerialObject;
  
  T__Serial_A& mySerialA;
  T__Serial_B& mySerialB;

  inline SerialUSBSwitch(T__Serial_B& theSerialB) : mySerialA(T__Serial_A::ourSerialObject), mySerialB(theSerialB) {}
  inline void begin(unsigned long baud)
  {
    mySerialA.begin(baud);
    mySerialB.begin(baud);
  }

  inline void begin(unsigned long baud, unsigned char config)
  {
    mySerialA.begin(baud, config);
    mySerialB.begin(baud, config);
  }

  inline void reset()
  {
    mySerialA.reset();
    mySerialB.reset();
  }

  inline void end()
  {
    mySerialA.end();
    mySerialB.end();
  }

  inline void setBaud(unsigned long baud)
  {
    mySerialA.setBaud(baud);
    mySerialB.setBaud(baud);
  }

  void startTransaction(unsigned char thePriority=0x1f)
  {
    if( writeA() ) mySerialA.startTransaction(thePriority);
  }

  unsigned char endTransaction()
  {
    if( writeA() ) return mySerialA.endTransaction();
    return 0;
  }
  
  inline void setAddressFilter(char** anAddressList, unsigned int theTimeoutValue=10)
  {
    mySerialA.setAddressFilter(anAddressList, theTimeoutValue);
  }

  inline unsigned char getInputLatency()  // returns the delay since last character read, in number of characters equivalent time.
  {
    if( readA() ) return mySerialA.getInputLatency();
    return 0; // TODO have to keep track of latest read and return time since.
  }
  virtual inline int   available()
  {
    if( readA() ) return mySerialA.available();
    else          return mySerialB.available();
 }
  virtual inline int   peek()
  {
    if( readA() ) return mySerialA.peek();
    else          return mySerialB.peek();
  }
  virtual inline int   read()
  {
    if( readA() ) return mySerialA.read();
    else          return mySerialB.read();
  }

  inline int availableForWrite()
  {
    unsigned char retVal =0;
    if( writeB() ) retVal = mySerialB.availableForWrite();
    if( writeA() ) retVal = mySerialA.availableForWrite();
    return retVal;
  }

  virtual inline void  flush()
  {
    if( writeB() ) mySerialB.flush();
    if( writeA() ) mySerialA.flush();
  }

  virtual size_t write(uint8_t c)
  {
    size_t retVal =0;
    if( writeB() ) retVal = mySerialB.write(c);
    if( writeA() ) retVal = mySerialA.write(c);
    return retVal;
  }

  void diagnose(const __FlashStringHelper* reason =0)
  {
    mySerialA.diagnose(reason, mySerialB); // TODO validate
  }

  inline void setMode(unsigned char mode)
  {
    mySerialA.setMode(mode);
  }
  inline unsigned char getMode()
  {
    return mySerialA.getMode();
  }
}; // class SerialUSBSwitch ==========================================================================================================================

#endif // !defined(SerialUSBSwitch_h)
