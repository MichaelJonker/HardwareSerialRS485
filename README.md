# Arduino RS485 communication software

Michael Jonker

[//]: # (This README.md file is written in github flavoured markdown. This file is visualized best by opening the project page on https://github.com/MichaelJonker/HardwareSerialRS485. Alternatively you can view - and edit -  this file with the ATOM editor https://atom.io/ )

[//]: # (for embedding comments in Markdown, see http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)
[//]: # (https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)
[//]: # (https://help.github.com/articles/github-flavored-markdown/)

This project provides a software suite to support the use of an RS485 transceiver connected to the USART (Tx/Rx pins) in a half-duplex, concurrent multi-drop (i.e. multi-master, multi-slave) environment. For this purpose, the software suite provides capabilities for message addressing and filtering as well as collision detection and collision avoidance.

Project page: https://github.com/MichaelJonker/HardwareSerialRS485

### Release history

* 20141221 V1.0

Note: To be compliant with the Arduino librarian, the link to this repository has been renamed to https://github.com/MichaelJonker/HardwareSerialRS485. Version V1.0 used to be accessible from https://github.com/MichaelJonker/Arduino_HardwareSerial_RS485 . Despite the different folder structure, version V1.0 is still in the history of this repository.


* 20151101 V2.0

Version 2 is a major release and introduces various new features:

* Fixes of non-conformant behaviour in case of message collision recovery.
* Transaction based message transmission retrial.
* Compatibility with Atmega32U4 based boards (e.g. Leonardo and Micro).
* Board specific configuration definitions.
* Library organization compliant with Arduino IDE 1.5 and up.


### Hardware connection

An example of a hardware connection is given in Figure 1, which shows the connection of the RO, DI, RE\* and DE lines of an RS485 transceiver to the Atmega/Arduino. Connecting the enable lines DE and RE\* to separate pins of the Atmega/Arduino, gives the software full control over the communication via RS485.

![Figure 1](extras/documentation/Figure1.jpg)

**Figure 1.**

The individual control of the RS485 enable lines allows for various modes of operation, such as collision detection (requiring read while write) or exclusive communication over RS232 (requiring disabled read, as enabling read from RS485 mutes the regular RS232 reception). Furthermore, the RE\* pull-up circuit ensures that the RS485 read is disabled upon reset of the Atmega/Arduino, hence allowing program downloads in the usual way from the Arduino IDE.
Note that, apart from the RE\* pull up resistor, all other extra components in the above diagram are optional. However, the presence of the LEDs to indicate the receive/transmit status of the RS485 will, be very appreciated for diagnostics.

The RS485 circuit can be implemented on a breadboard (Figure 2), soldered onto a shield (Figure 3), or included into a new PCboard design (Figures 4).

![Figure 2](extras/documentation/Figure2.JPG)

**Figure 2.** The simplest breadboard realization of the circuit shown in figure 1, based on the Arduino/Micro and a directly mounted RS485 transmitter IC.

![Figure 3](extras/documentation/Figure3.JPG)

**Figure 3.** A more sophisticated implementation with additional circuitry based on the screw-shield. The RS485 interface IC is visible in the top-middle on the shield.

![Figure 4](extras/documentation/Figure4.JPG)

**Figure 4.** A prototype chassis with a primitive backplane connects two custom designed Atmega/Arduino PC boards to the RS485 bus.


Alternatively a circuit board containing a 485 interface can be procured from http://yourduino.com/sunshop2/index.php?l=product_detail&p=323 (see Figure 5). This board does not, however, contain the LEDs to indicate the status of the enable lines. More important, this board has a DE high pullup, (in contrast to the recommended configuration shown in figure 1), with the implication that upon start-up of the Arduino (all ports tri-stated), Data Transmission is enabled. As a consequence, the MAX485 will actively drive the RS485 bus, blocking any bus transmission. Hence, it is suggested to remove the DE pullup and to add an external pulldown resistor. Finally one should be aware that these interface boards contain a 120 ohm bus termination resistor, which is not a problem if used in point to point communications, but not in a multidrop environment, where only the extremities of the bus should be terminated.

![Figure 5](extras/documentation/Figure5.JPG)

**Figure 5.** Breadboard version based on a small pc board with a premounted RS485 interface. Due to circuit failure in the RS485 PC boards (possibly following a manipulation error), this configuration was not fully tested.

Figure 6 shows the test setup used by the author to develop the HardwareSerialRS485 library, connecting together 6 RS485 devices (One RS485-USB connected to the PC, one Uno/ScrewShield, 50 m of cable, one Micro/breadboard, two custom boards, a second RS485-USB connected to the PC).

![Figure 6](extras/documentation/Figure6.JPG)

**Figure 6.**


### Software capabilities

The software is implemented on top of a rewritten `HardwareSerial` class that makes use of several included helper classes to extend its capabilities. This class is almost fully compatible with the classical HardwareSerial version provided by the Arduino distribution and hence methods like the `Serial.print()` can be used unchanged.
N.B., the qualifier 'almost' have been added in the previous sentence as the HardwareSerialRS485 class does not support internal buffers larger than 256 bytes. However, the need for a large buffers in a RS485 concurrent multi-drop environment has yet to be demonstrated.

RS485 capabilities are provided by the helper class `RS485serial` allowing one to switch between RS232 and various RS485 based operational modes (full details are given below).

- Collision avoidance is implemented through the assignment of a message priority.
A node will not start transmission before the bus has been idle for a time determined by the priority. Note that the highest priority is given to messages with the lowest priority number.

- Collision detection is implemented by read-verified-write, i.e. all data written is checked on the fly for proper echo-back.
If the software detects a frame error or a data mismatch when the echo character is received, it will disable the RS485 transmission and flag the collision status. The included transaction management will resend the message after the bus is idle again.

The helper class, `MessageFilter`, provides message recognition and address filtering at the interrupt level.
Due to a possible large data traffic between various nodes on the half duplex multi-drop RS485 bus, and given the limited capacity of the internal message buffer of the HardwareSerialRS485 class, there is a risk of message loss due to buffer flooding. This real-time filtering capability reduces this risk by pruning data not intended for the node, leaving the internal message buffer of the HardwareSerialRS485 class fully available for messages that are destined to the node.

It should be noted that message filtering in the presence of high bus occupancy will consume (real time) processing resources of the Atmega/Arduino.
In a future version may, as an option, exploit the hardware addressing capabilities of the Atmega chip (see also future developments).

Finally, a helper class Transaction provides support for message oriented transmission. Under control of this class, a message that fails during transmission, (bus occupancy, collision) will be retried after idle conditions are met. A call to endTransaction() will, if needed wait until the message is transmitted (not including acknowledgment) and report to the caller on the completion status.

#### Example code

The project provides an example/demo sketch and instructions of usages.

A simple command response sketch `RS485CommandResponse` is included in the example folder under the HardwareSerialRS485. This primitive sketch allows for switching between various communication modes and implements some rudimentary commands for echo and information printing.

#### Host interface and software

Host (PC or Android) connection to the RS485 bus can be achieved by an RS485 to USB converter. As an example, the RS485-USB converter from FTDIchip ([FTDIchip Products USBRS485](http://www.ftdichip.com/Products/Cables/USBRS485.htm)) was used for development by the author. Another possible candidate could be the Sparkfun BOB-09822 breakout board [https://www.sparkfun.com/products/9822](https://www.sparkfun.com/products/9822). However, these solutions will not provide host-based collision detection and only a limited capability of host based collision avoidance. A project for a USB RS485 interface (discussed below) may provide these facilities in a future release.

Various applications exist to communicate with the RS485 nodes on the bus. For simple command responds testing, a simple terminal emulator such as Putty ([www.putty.org](http://www.putty.org/)) or FTDI UART terminal can be sufficient. More complex applications must be developed explicitly. (In a future post, the development of an Android specific application will be published.

## Detailed software description

You may want to skip this chapter if you do not want (and need) to know these details (with a possible exception for the sections on HardwareSerialRS485, HardwareSerialRS485_configuration and RS485CommandResponse).

**Preamble:**
This project makes extensive use of template classes. The use of templates allows for a high degree of code configuration options that are resolved at compile time, leading to improved code performance. Template parameters are used in this project to enhance functionality through supplied helper classes, as well as for various configurable parameters, such as buffer sizes, buffer overrun policy, RS485 enable wirings, etc.

Another design choice is that header files contains the code for both class definition and class implementation. The majority of method implementations are given as in-line code in the class definition. However, also the implementation of non in-line methods as well as the static class variables is given in the header files. This class implementation code is activated, under `#ifdef` preprocessor control, when the header files are included from the almost empty class implementation files (see e.g. HardwareSerialRS485.cpp).

The following sections describe the files and classes that are part of the HardwareSerialRS485 library. All described files are located under the folder HardwareSerialRS485/src unless indicated otherwise.

#### HardwareSerialRS485.h (and .cpp)

This header file serves as a library wrapper and #includes all other needed header files. This header also checks for potential conflict with the classical HardwareSerial header file from the Arduino project. The header file should be included before the Arduino.h header is included (whether directly, or indirectly by another included header file). To avoid conflicts, the inclusion of this header file inhibits the loading of the classical HardwareSerial header file from the Arduino project.


#### HardwareSerialRS485_configuration.h

This header file, which is #included by the HardwareSerialRS485 header file, specifies the implementation details and options of the Serial objects. This file can be tailored to user requirements. However, the user is advised to copy this file to a user specific location in his project folder. (For more detail on tailoring the RS485 configuration, please see the section __Configuration and serial object definition__ below).

#### utility/HardwareSerialRS485_Enabled.h

This header file contains the rewritten HardwareSerial class. This class is backward compatible with the classical HardwareSerial class provided in the Arduino project in release V1.05 and up (tested up to V1.6.5). This is a template class, which by default does not provide additional functionality. Extensions are activated by passing helper class specifications through the template parameters. This particular choice was made such that the class stays lean and efficient in case no additional functionality is required.

#### utility/HardwareSerialRS485_Helper.h

This header file contains the helper class RS485serial, which can be passed by a template parameter to the HardwareSerialRS485 class to provide the RS485 capabilities. The RS485Serial class is itself implemented as a template class and takes as a parameter the `TRxControl` class (also included in this header file), which specifies the wiring of the RS485 enable lines to the Atmega/Arduino.

#### utility/HardwareSerialRS485_Tracer.h

This header file provides a diagnostics tracer function which keeps track of the most recent internal state changes. This facility is used for debugging and is inactive by default. Tracing can be activated by defining the macro RS485configuration_useTRACE (either in the HardwareSerialRS485_configuration.h or in boards.txt).

When tracing is activated, a trace history dump will be included in the output of the diagnostics() method.

#### utility/MessageFilter.h

This header file contains the helper class MessageFilter, which can be passed by a template parameter to the HardwareSerialRS485 class to provide message filtering capabilities. The MessageFilter class is itself implemented as a template class and takes as a parameter an MFP class (two versions included in this header file), which specifies the message filtering protocol.

#### MessageReader.h

This header file contains the `MessageReader` class, a client level message reader. The MessageReader class, which also does message recognition and filtering, is implemented as a sub-class of the MessageFilter class. Furthermore, this class will properly flag the user when a complete message is available without the overhead of blocking for a finite time to ascertain that no new characters have arrived (as is done by the Stream class).

#### utility/USARTdef.h

The header file `USARTdef.h` provides ‘normalized’ definitions of the USART parameters for USARTn (where n=0 … 3). The code generates for each available USART a dedicated state-less parameter class. These USART classes are used for the T__USART template parameter of the HardwareSerialRS485_Enabled template class.

#### utility/BitManipulation.h

This header file provides template functions for type independent bit manipulation functions. This is a general purpose library that may serve various purposes also outside the HardwareSerialRS485 context. (Arduino project/community).

#### utility/SerialUSBSwitch.h

This header file provides a simple wrapper class SerialUSBSwitch to support the demo sketch in the Leonardo or Micro environment. This class facilitate switching between RS485 via the USART and USB serial via the Serial_ class (as defined in USBAPI.h).

#### ../examples/RS485CommandResponse/RS485CommandResponse.ino

The example demo sketch RS485CommandResponse allows for switching between various communication modes and implements some rudimentary commands for echo and information printing. Some diagnostic facilities are provided.

This sketch makes further use of an (embedded) `ApplicationControl` class, which maintains the settings for the communication mode and provides various functions.
To support the demo sketch in the Leonardo or Micro environment, this sketch also makes use of the `SerialUSBSwitch` class described above. Note that the usage of the SerialUSBSwitch class is only required if you would like to switch dynamically between two communication channels.

#### Implementation details

Due to the implementation of the USART parameters as a static class that is passed by a template parameter to the HardwareSerialRS485 class, the HardwareSerialRS485 class handles only one USART per template instance. In case more than one Serial object is required for systems with more than one USART, each Serial object will need its own instance of the template class. While for single USART devices the template based solution will lead to efficient and compact code (as many addresses can be resolved at compile time), for devices with several USART there could be a code size penalty (however, there is no execution speed overhead). This issue will be further investigated (and if needed improved) in a future release.

## Installation and Deployment
### Installation
The software distribution contains the HardwareSerialRS485 library folder and, included in the examples sub-folder, the RS485CommandResponse demo sketch. The software can, in principle, be copied *as is* into your Arduino project libraries folder.


### Configuration and serial object definition

Due to the complex structure of the HardwareSerialRS485 template class, the specifications of the derived HardwareSerialRS485_<*n*> classes and Serial<*n*> objects are given by C preprocessor macros. Through this mechanism, declaration and implementation of the HardwareSerialRS485 classes and objects can be made consistent without having to modify multiple locations.

The header file `HardwareSerialRS485_configuration.h` provides the definitions for the HardwareSerialRS485 classes associated with the potential hardware USART devices. For a HardwareSerialRS485_<*n*> class to be created, both a RS485configuration_HardwareSerialRS485<*n*> definition and a USART<*n*> device should exists.

To further ease the tailoring, the definitions are cascaded as shown here

````cpp
#define RS485configuration_TRxBufferParameters 6, 6, ' '
#define RS485configuration_TRxControl TRxControl< 'B', 2, 3 >
#define RS485configuration_RS485helper RS485serial< RS485configuration_TRxControl >
#define RS485configuration_MessageFilterParameters MFP< '{', '}' >
#define RS485configuration_MessageFilter MessageFilter< RS485configuration_MessageFilterParameters >
#define RS485configuration_HardwareSerialRS485_0 HardwareSerialRS485< USART0, RS485configuration_TRxBufferParameters, RS485configuration_RS485helper, RS485configuration_MessageFilter >
#define RS485configuration_HardwareSerialRS485_1` HardwareSerialRS485< USART1, RS485configuration_TRxBufferParameters, RS485configuration_RS485helper, RS485configuration_MessageFilter >
// no default definition is given for RS485configuration_HardwareSerialRS485_2
// no default definition is given for RS485configuration_HardwareSerialRS485_3
//
````

These definitions reflect the default values provided by the header file HardwareSerialRS485_configuration.h. Additional code for testing if a definition already exists are, for clarity, omitted from the above extract.

In the following a short description and options for the various definitions are given.

* `RS485configuration_TRxBufferParameters` 6, 6, '  '

   defines, in three comma separated fields, the Tx (transmit) and Rx (receive) buffer sizes and the Rx-buffer overrun indicator.
   - the buffer sizes are specified in powers of 2, i.e. a value 6 specifies a buffersize of (1<<6) or 64 bytes;
   - the buffer overrun indicator specifies the character to be entered in the Rx buffer when the buffer is full and incoming characters will have to be dropped. A space character indicates no overrun indicator.

   Default definition. . . . . . : 6, 6, ' '   // Buffer size (Tx & Rx): 64, no Rx-buffer overrun signalling

   Alternative example . . .: 5, 7,'$' // Buffer size: 32 (Tx) and 128 (Rx), Rx overrun marked by a '$' sign.


* `RS485configuration_TRxControl` TRxControl< 'B', 2, 3 >

   defines the class that implements the control of the RS485 enable lines.

   This is likely **the only definition that you should adapt** to reflect how the RS485 enable pins are connected in your hardware implementation.

   Unless in special cases where a user defined class is needed, you should use the TRxControl template class.
   This template class is specialized with three parameters that give the port (by character), the TxE port number (aka DE) and the RxE\* port number (aka RE\*).

   Example: if the hardware connects output pin B2 to the TxE input and output pin B3 to the RxE\* input, you should specify TRxControl< 'B', 2, 3 >.

   **Important** the parameters refer to hardware port numbers and do not use the Arduino pin naming convention. Also note that the mapping of pin numbers to port number is different from the Atmega328P/Arduino-Uno and the Atmega32u4/Arduino-Leonardo/Micro.

   For efficiency this class only handles hardware configurations where the two enable pins of the RS485 transceiver are controlled by the same port register.
   In case this restriction does not match your hardware reality, you have to adapt the TRxControl class.

   The following definition is given as an example (and serves as default).

   Default definition. . . . . . : TRxControl< 'B', 2, 3 >

   Alternative example . . . : TheSuperDuperTRxControlClassYouWillWriteSomeday< D3, D4 >

* `RS485configuration_RS485helper` RS485serial< RS485configuration_TRxControl >

   defines the helper class to provide RS485 capabilities.
   You can define RS485configuration_RS485helper without a value to create HardwareSerialRS485 classes without RS485 capabilities.


* `RS485configuration_MessageFilterParameters` MFP< '{', '}' >

   defines the MessageFilterParameter class used by the message filter.
   Note that this definition is also used by the message reader as default MessageFilterParameter. You are advised to use the MFP template class which takes two parameters: the SOM (start of message) and EOM (end of message).

   Default definition. . . . . . :  MFP< '{', '}' >      // message filtering based on '{' and '}' message delimiting characters

   Alternative example . . . : MFP< 0x01, 0x04 >    // alternative example: message filtering based on control characters SOH: Start of header (^A), and EOT: end of transmission (^D)

* `RS485configuration_MessageFilter` MessageFilter< RS485configuration_MessageFilterParameters >

   defines the message filter class used by the HardwareSerialRS485 classes.
   If RS485configuration_MessageFilter is defined without a value, then the HardwareSerialRS485 classes will have no filtering capabilities.

* `RS485configuration_HardwareSerialRS485_0` HardwareSerialRS485< USART0, RS485configuration_TRxBufferParameters, RS485configuration_RS485helper, RS485configuration_MessageFilter >

   defines a standard HardwareSerialRS485_0 class and associated Serial0 object.

* `RS485configuration_HardwareSerialRS485_1` HardwareSerialRS485< USART1, RS485configuration_TRxBufferParameters, RS485configuration_RS485helper, RS485configuration_MessageFilter >

   defines a standard HardwareSerialRS485_1 class and associated Serial1 object.

  Note that HardwareSerialRS485_1 is the defined class on Atmega32u4 processors (Leonardo/Micro)


* `RS485configuration_HardwareSerialRS485_2`

   there is no default definition for this class. If the user would like to define this class (on a device where USART2 exists) he will have to provide this definition.

* `RS485configuration_HardwareSerialRS485_3`

  there is no default definition for this class. If the user would like to define this class (on a device where USART3 exists) he will have to provide this definition.


### User adaptations

There are several ways to tailor this configuration:

* The recommended way is to define your own configurations by adding a boards.txt file. This mechanism provides sufficient means to configure 99% of your needs. This option is explained in full detail below.

* In case the boards.txt file does not provide the full required flexibility you need, you can adapt a copy of the HardwareSerialRS485_configuration.h header file that you have placed in the same folder as the boards.txt file. In this case you should make sure that the ...build.extra_flags definitions in your boards.txt file contain the string `"-I{runtime.platform.path}"`. This will ensure that the folder containing your copy of the HardwareSerialRS485_configuration.h is part of the include search paths. Omitting this inclusion will cause the HardwareSerialRS485_configuration.h header file distributed with the library file to be included.

* Finally you could hack the HardwareSerialRS485_configuration.h version which is the folder libraries/HardwareSerialRS485/src and adapt it to you needs. This may be the fastest way, but is **not recommended**: you will not profit from multiple board configuration options and installing a new release of the library will undo your changes.

#### Making your personal copy of the configuration files
You are advised to create the following nested folder structure in your project space:

`<myArduinoProjectFolder>/hardware/HardwareSerialRS485/avr`

where you should replace <*myArduinoProjectFolder*> with the name of your project folder, i.e. the folder where all your sketch folders are located as well as the libraries folder:


Then, copy into this folder the files `HardwareSerialRS485_configuration.h` and `boards.txt`
which you can find in the folder:
`<myArduinoProjectFolder>/libraries/HardwareSerialRS485/src`


#### Adapting boards.txt
The boards.txt file can be adapted to define the c++ preprocessor macros that tailor the definition of your HardwareSerialRS485 classes. If no definitions are given, then the HardwareSerialRS485_configuration.h file provides proper default values as described above.

In the example boards.txt file that comes with the HardwareSerialRS485 library you will see the definition of two boards *Arduino UNO with RS485* and *Arduino Micro with RS485*. If you want to add other boards with RS485, then the examples should provide enough information to add this yourself. These boards will appear in the IDE under the Tools menu.
In addition, the boards.txt file defines board sub-menus. After selecting a board defined in the boards.txt file you will see a new sub-menu item right below the selected board menu item.

The predefined choices here are *RS485* and *no RS485*, but you can add as many configurations as you need (with different TxE, RxE\* connections, buffer space parameters, etc.).

As mentioned earlier, the specification of the TxE and RxE\* mapping to the output pins of the Atmega use the native form and not the Arduino convention. The mapping is different on the Atmega328p boards and the Atmega32u4. For your convenience, the file boards.txt contains a table of how these Pin numbers map to Port numbers.

### Usage of RS485CommandResponse example sketch

As the RS485CommandResponse sketch has evolved to a somewhat more complex application, some extra clarification is given here.

The RS485CommandResponse sketch is implemented as a message oriented application, i.e. incoming messages are decoded and executed. Depending on the request, the sketch may reply with zero, one or more messages. The sketch may also send unsolicited messages (in this particular case it may send out so called *breathing messages*).

All application specific parameters are maintained in the class ApplicationControl (embedded in the sketch file). Four of these parameters (ApplicationType, SlaveId, Priority, RS232ExpirationTime) are read upon start up from the EEPROM. Three of these parameters can be updated, and saved on the EEPROM with the following (case sensitive!)  user commands to be given over the active communication channel (i.e. which is either the RS485 or the RS232 based Console):
```tcl
{S*SLID=<your slave identifier character>}
{S*PRIO=<your slave priority (0...31)>}
{S*EXPL=<your slave RS232 expiration time in seconds, use zero to disable>}
{S*?}
{S*EEPROM.SAVE}
```
If, on start-up of the sketch, the EEPROM was not found to be programmed for this demo sketch application type (here 66) then the parameters will be set to the following default values: ApplicationType=66, SlaveId= '+'; Priority=0x1F; RS232 expiration time =0 (i.e. disabled). The EEPROM values can be updated with a {S...EEPROM.SAVE} command.

After programming the SlaveId with e.g. message `{S*SLID=A}` the module will respond to both the explicit slave address starting with `{SA` and to the general *broadcast* address starting with `{S*`.

Programming the RS232 expiration time with message `{S*EXPL=...}`, will set the time in seconds that the RS232 input should be idle for the slave to automatically switch to RS485 communication. Note that after start-up, the first value used for RS232ExpirationTime is the bitwise `and` of the specified value and 0x07. Setting the RS232 expiration time to zero will inhibit the automatic switch over, however, you can still switch to RS485 explicitly with the message `{S*RS485}` or `{SARS485}` from the RS232 Console. Note that whilst command input is given over either RS232 or RS485, depending on the active communication mode, any output sent to RS485, is always also transmitted to the RS232 console.

When the RS485CommandResponse sketch starts up, it will first produce a sequence of flashes on the DE and RE* LEDs (see fig 1.). Subsequently, the demo will send an ALIVE message and a memory statistics report to the RS232 console.
Depending on the value of RS232ExpirationTime, and whether data over RS232 was available or not, the demo will then switch to RS485 mode.

#### Leonardo and Micro Support
With version V2.0 support has been added for the Atmega32u4 based Leonardo and Micro boards.
In this case RS485 communications is controlled by the Serial1 object. The possibility to switch Serial1 between RS232 or RS485 mode still exists. However, as the Leonardo and Micro do not connect the Rx/Tx pins to an on-board RS232/USB converter, there is no real usage for this switching capability, unless you want to hook up an external RS232/USB interface (e.g. https://store.arduino.cc/product/A000059).

With the Atmega32u4, host communication is provided by an on-chip USB interface. To allow the RS485CommandResponse demo sketch to function identically in an Atmega32u4 environment as in the Atmega328p (UNO) environment, a small HardwareSerialRS485 compatible wrapper class is used that allows switching the Serial communication between RS485 (Serial1, which uses USART1) and the on-board USB interface (using the Serial_ class).

In case reading over RS232 is not required, one IO pin can be spared by leaving the RS485 Receive Enable always active (i.e. connecting RE* with a resistor to ground instead of connecting it to an output pin). In this case you are advised to instantiate the TRxControl template class with the same values for the TxEnable and the RxDisable template parameters. Example: if bit 6 of Port B is used to control the Transmit Enable, then parametrize the TRxControl template class as `TRxControl<'B', 6, 6 >`.

### Message oriented transmission support:

With version V2.0 support has been added for message oriented transmission through a transaction mechanism. To this purpose two new methods have been added to the HardwareSerialRS485 class. Note that these method calls have no effect if the RS485 transmission mode is not active.

````cpp

void startTransaction(unsigned char priority=0x1f);
unsigned char endTransaction();

````

The method startTransaction() will: a) flush the transmission buffer, b) reset the buffer pointer and c) set the priority based minimum RS485 bus idle time needed before the next message may be transmitted. The method endTransaction() will flush the transmission buffer and return the completion status of the transaction.

If during a transaction, a call to write(), print() or flush() produces an error of any kind, then the complete message transmission is interrupted and retried sometimes later. Message retransmission is possible, provided that the full message is still inside the buffer. If the accumulated message length has exceeded the HardwareSerialRS485 transmit buffer size, the message can no longer be retried and subsequent message data will be discarded. The user is informed in this case through the return status of endTransaction() method, leaving it up to the user to decide whether to recreate and resend the message or not.

These two methods may be used by a more message oriented approach as demonstrated in the demo sketch. (n.b. the following code, extracted from the RS485CommandResponse demo sketch, has been simplified to better illustrate the idea)

````cpp
inline void startMessage(unsigned char aPriority=0xff)
{
  if(isRS232Mode()) return;   // do not bother with message dressing-up if we are in 232 mode
  Serial.startTransaction(aPriority!=0xff ? aPriority:  myDefaultPriority);
  Serial.print('{');          // add start of message
  Serial.print(theHeader);    //
}
inline unsigned char endMessage()
{
  if(isRS232Mode()) { Serial.println(); return 0; } // do not bother with message dressing-up if we are in 232 mode
  Serial.println('}'); // add end of message
  return Serial.endTransaction();
}
//
````

**_On priority control_**: The above implementation allows a slave to control its message priority on a per message basis. This opens the possibility to boost its priority to the highest value (zero) in reply to a directly addressed request from a master. For unsolicited messages, the slave should still use is assigned priority.
Note, to avoid possible collisions between slave replies, a master should always leave some idle time between two directly addressed (i.e. non broadcast) messages requests to different slaves.

## Future developments, things to do

The current version may need some ‘aesthetic’ improvements that addresses maintainability and code efficiency. Furthermore, there is a need for related developments (either as part of this project or as an independent project). This section outlines the developments (direct or related) that are currently foreseen on short and median long term.

### Bugs

**Your input required:**, all bugs that were known to the author are resolved. Please let me know if you find any incoherent behaviour.

### Features and Limitation

There are a few features and limitations that you should be aware of:

**_RS485 Bus hugging:_** Please be aware, if one RS485 bus participant continuously occupies the bus, then all other slaves trying to emit a message, will hang without escape possible. (A time out needs to be implemented here).

**_Collision detection limits:_** Please be aware, the capability of a RS485 bus participant to detect a collisions has its limit, specially when using long cables. In this case it is not improbable that message sending nodes will not detect immediately a collision caused by a message from the remote node, because the signal from the remote node is too weak to disturb the local signal. In that case it may take one or two characters before the collision is detected by one of the nodes, while the other node remains unaware of the collision. A possible strategy to reduce the impact of such collisions (i.e. which usually corrupt the Start of Message) is to precede the start of message character by two or three dummy characters (e.g. replacing `print('{')` with `print("**{")` in the above startMessage example).

**_Message retrial limitation:_** Please be aware, messages that have exceeded the internal buffer size, cannot be retried in case of a collision (as the start of the message was overwritten). If this occurred, an error status is returned when calling `endTransmission()`. However, this method will lead to erratic behaviour if the message length exceeded 256 characters. (You may call this a known bug or a documented restriction). It will be fixed in a later release.

### Implementation improvements

Apart from the various todo comments embedded in the current software, there are a few more global developments that are discussed here.



**_RS485 as static class variable of HardwareSerialRS485_** (private development notes): Currently the HardwareSerialRS485 class subclasses the RS485serial class that is passed as a template parameter. On the one hand, this makes the public methods of the RS485serial class readily available to the users of the HardwareSerialRS485 class, most notably, the method to control the RS communication mode. On the other hand, the RS485serial object is, in this configuration, an instance field and not a class (aka static) field. Because of this the interrupt handlers of the HardwareSerialRS485 need to be instance methods and not class methods.

In case RS485serial class is implemented as a static field in the HardwareSerialRS485 class, then the later should provide ‘relay’ methods to give access to the control method of the RS485serial class.

Related to this, the linking of interrupt vectors and friendship declarations may become simpler. To be investigated: the option to dynamically link the interrupt handlers to the interrupt vectors. Should the interrupt vectors be hooked to interrupt handlers in the USART classes, which can be relayed to the static methods of HardwareSerialRS485. Option to use the now available static pointer to the instantiated *singleton* object. Should this pointer be moved to the USART class?

**_USART class definitions:_** Test for completeness of all hardware configurations.

### New developments

This subsection describes new development in relation to this project.

**_Non-blocking endTransaction() method:_** A non-blocking version of endTransaction() that can be polled to check for the message completion. A simple version will do one retry attempt per call of this method. A deluxe version will do some timer interrupt scheduling.

**_Improvement of collision avoidance:_** Currently, bus occupancy is detected after the transmission of the first character is completed. Using a level change interrupt, the bus occupancy can be detected as soon as the start bit of the first data item is transmitted onto the bus. To avoid excessive resource usage, the level change interrupt will be activated only for a short period (i.e. the duration of frame transmission) prior to the message transmission.
Such an option will reduce the likelihood for collisions; Furthermore, this opens the option to reduce the enforced priority based idle time (factor 2 or 4) as this idle time may then be based on fractional frame transmissions.

Note: if pin IPC1 is unused, one could connect IPC1 with RD and produce a snapshot of timer1 each time a level change occurs without the use of an interrupt handler.

**_Hardware implemented address filter:_** The current implementation reads all character transmitted on the bus to filter the messages that correspond to its addresses. In case of high bus occupancy, this implies a loss of processing resources for this filtering. A future implementation will (by option) use the capability of the Atmega to recognize special 9-bit address frames on the bus. However, this option can only be used if all devices on the RS485 bus, including the host devices, implement this protocol. Bus idle detection will become more complex to implement as all transmissions are filtered out. However, this may be solved by using an end message address frame. In this case the address frames can simultaneously take the function of start- and end-message delimiters.

**_Implementation of collision and filter inside RS485 – USB device:_** The current RS485 host interfaces (FTDI chip, Sparkfun), provides poor support for collision detection and collision avoidance. In essence, the host application does not have immediate knowledge of the bus occupancy, which makes collision avoidance at the character level impossible. The FTDIchip converter provides some options for collision detection as the chip can be configures in local read back mode. The host software can check whether the messages read back, match the expected echo of the messages sent. The Sparkfun BOB-09822 breakout board (based on the same FT232 chip from FTDIchip), however, does not provide individual control of the enable lines TE and RE* as these two lines are connected together in hardware. This makes collision detection impossible without modification of the hardware.

An adaptation of the local firmware in the FTDI interface could be considered. However, FTDI does not provide off-hand support for modifying the local firmware. Moreover, the adaptation of this firmware is most likely not an easy task.

The preferred solution is to make a new interface/gateway based on an Arduino Micro (which deploys the Atmega32U4 chip) in conjunction with an RS485 transceiver. The current software can be used to implement a USB - RS485 converter with collision detection and collision avoidance. A second option is to use the Atmega16U2 chip. The same chip is also used on the Arduino boards for the USB interface. The source of the firmware is available; however, the adaptation of this software is most likely more complex. Also the hardware implementation will be more complex to realize.

**_Supervisor communication application:_** A host supervisor communication application, running on Android, which can interact with the slaves and record the full network traffic, will be published as an independent project.

**_Wiki pages for documentation:_** I believe this is a good idea, as the amount of information in this readme is actually extending beyond what is reasonable and should migrate to a Wiki. However, this will need some time to set it up properly.

## - ##
>**About the author:**

> Michael Jonker, ERPID = {52.36040, 4.87001, NAP+5m, 1954.349629}

> His involvement with '*Arduino*' dates back to summer 1975, when he met Paola Arduino (from Turin) on the beach in Albinia, (near Grosseto, Toscani). Does anyone know what became of her ?
