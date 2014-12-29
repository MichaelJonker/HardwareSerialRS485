/*
  Print.cpp - Base class that provides print() and println()
  Copyright (c) 2008 David A. Mellis.  All right reserved.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
 
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
 
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 21 December 2914 by Michael Jonker. Enhanced with fieldControl parameter, implemented many methods through templates definitions
*/

/* MJ fieldControl enhancements added by Michael Jonker (beer-ware licensed enhancements)

Enhanced the formatting capabilities of the print methods. The optional third
parameter of the print() methods allows to control the print output format in
terms of minimum field with, filling character and forced sign.

The recommended way of creating the fieldControl parameter is to use the fieldControl method, examples:
print("answer="); print( 42);                                          // answer=42
print("answer="); print( 42, 0, Print::fieldControl(4));               // answer=  42
print("answer="); print(137, 0, Print::fieldControl(4));               // answer= 137
print("answer="); print(137, 0, Print::fieldControl(4,false, true));   // answer= +137

print("answer=0x"); print((unsigned) 0xcafe, Print::Radix_HEX, Print::fieldControl(8, true)); // answer=0x0000cafe
print("answer=");   print((unsigned) 0xcafe, Print::Radix_HEX, Print::fieldControl(6 ));      // answer=  cafe
print("answer=");   print(  (signed) 0xcafe, Print::Radix_HEX, Print::fieldControl(6 ));      // answer= -3502
const unsigned char myFieldControl = fieldControl(6, false, true );
print("answer=");   print(  (signed) 0xcafe, Print::Radix_HEX, myFieldControl);               // answer=  -3502
print("answer=");   print((unsigned) 0xcafe, Print::Radix_HEX, myFieldControl);               // answer=  +cafe
*/

#ifndef Print_h
#define Print_h

#include <inttypes.h>
#include <stdio.h> // for size_t

#include "WString.h"
#include "Printable.h"

// deprecated: use PRINT::Radix_DEC etc.
#define DEC Print::Radix_DEC
#define HEX Print::Radix_HEX
#define OCT Print::Radix_OCT
#define BIN Print::Radix_BIN

// deprecated: use PRINT::fieldControl etc.
#define PFCTRL(_size, _fillZeroes, _forceSign) Print::fieldControl(_size, _fillZeroes, _forceSign)

class Print
{
  public:
    const static int Radix_HEX=16;
    const static int Radix_DEC=10;
    const static int Radix_OCT= 8;
    const static int Radix_BIN= 2;

    // fieldControl parameter layout:
    static const unsigned char fieldSizeMask  = 0x1f; /* gives the fieldsize -1 of the entry to be printed */
    static const unsigned char fillZeroes     = 0x20; /* if true fill with '0', if not fill with ' '  */
    static const unsigned char forceSign      = 0x40; /* if true force sign symbol, including for positive values */
    static const unsigned char signedItem     = 0x80; /* forces the value as a signed item (reserved for internal use) */
    static const unsigned char inline fieldControl(unsigned char _size, bool _fillZeroes=false, bool _forceSign=false)
    {
      return ( (_size==0? 0: _size>fieldSizeMask ? fieldSizeMask:_size-1) | (_fillZeroes ? fillZeroes:0) | (_forceSign ? forceSign:0) );
    }

  private:
    int write_error;
    size_t printNumber(unsigned long, uint8_t, unsigned char fieldControl=0);
    size_t printFloat(double, uint8_t);
  protected:
    void setWriteError(int err = 1) { write_error = err; }
  public:
    Print() : write_error(0) {}
  
    int getWriteError() { return write_error; }
    void clearWriteError() { setWriteError(0); }
  
    virtual size_t write(uint8_t) = 0;
    size_t write(const char *str) {
      if (str == NULL) return 0;
      return write((const uint8_t *)str, strlen(str));
    }
    virtual size_t write(const uint8_t *buffer, size_t size);

    size_t print(const __FlashStringHelper* item);
    size_t print(const String& item);
    inline size_t print(const char item[]) { return write(item);}
    inline size_t print(char       item  ) { return write(item);}

    inline size_t print(unsigned char item, unsigned char radix = Radix_DEC, unsigned char fieldControl = 0) { return printNumber((unsigned long)item, radix, fieldControl);}
    inline size_t print(unsigned int  item, unsigned char radix = Radix_DEC, unsigned char fieldControl = 0) { return printNumber((unsigned long)item, radix, fieldControl);}
    inline size_t print(unsigned long item, unsigned char radix = Radix_DEC, unsigned char fieldControl = 0) { return printNumber((unsigned long)item, radix, fieldControl);}
    inline size_t print(         int  item, unsigned char radix = Radix_DEC, unsigned char fieldControl = 0) { return printNumber((unsigned long)item, radix, fieldControl|signedItem);}
    inline size_t print(         long item, unsigned char radix = Radix_DEC, unsigned char fieldControl = 0) { return printNumber((unsigned long)item, radix, fieldControl|signedItem);}
    
    inline size_t print(double        item, unsigned char digits = 2)                                        { return printFloat(item, digits);}
    inline size_t print(const Printable& item)                                                               { return item.printTo(*this);}

                          inline size_t println(void)                                                        { return                                    print('\n'); }
    template <typename R> inline size_t println(R item                                                     ) { return print(item)                      + print('\n'); }
    template <typename R> inline size_t println(R item, unsigned char radix, unsigned char fieldControl= 0 ) { return print(item, radix, fieldControl) + print('\n'); }
//  note we cannot combine the two above templates in one by giving a default for radix, as this makes matching of print() ambiguous for R char*, and others

};

#endif
