/* BitManipulation.h

templates for bit manipulation

Copyright (c) 2015 Michael Jonker.  All right reserved.

This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free
Software Foundation; either version 2.1 of the License, or (at your option)
any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef BitManipulation_h
#define BitManipulation_h
// define templates for easy bit manipulation
template <typename R, typename V> inline void setBits     (R &item, const V set)             { item |= set;  }
template <typename R, typename V> inline void clrBits     (R &item, const V clr)             { item &= ~clr; }
template <typename R, typename V> inline void modBits     (R &item, const V set, V clr)      { item = (item &~clr ) | set; }
template <typename R, typename V> inline void setBit      (R &item, const V set)             { item = (item | (((R) 1)<<set)); }
template <typename R, typename V> inline void clrBit      (R &item, const V clr)             { item = (item &~(((R) 1)<<clr)); }
template <typename R, typename V> inline bool testBit     (const R item,  const V bit)       { return (item & (((R) 1)<<bit))!=0; }
template <typename R, typename V> inline bool testBitsAny (const R item,  const V bits)      { return (item & bits)!=0; }
template <typename R, typename V> inline bool testBitsAll (const R item,  const V bits)      { return (item & bits)==bits; }
template <typename R, typename V> inline bool testBitField(const R  item, const V FieldMask, const V FieldValue) { return (item & ~FieldMask)==(FieldValue & FieldMask); }
template <typename R, typename V> inline void setBitField (R &item,       const V FieldMask, const V FieldValue) { item = (item & ~FieldMask) |(FieldValue & FieldMask); }
template <typename R, typename V> inline V    getBitField (R &item,       const V FieldMask) { return (V)(item & FieldMask); }
template <typename R, typename V> inline void neukBits    (R &item,       const V ClrMask,   const V SetMask)    { item = (item & ~(ClrMask^SetMask)) ^SetMask; }
#endif // ifndef BitManipulation_h
