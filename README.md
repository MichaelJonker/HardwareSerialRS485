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

For full information see the wiki pages on https://github.com/MichaelJonker/HardwareSerialRS485/wiki
