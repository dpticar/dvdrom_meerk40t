# dvdrom_meerk40tdvdrom_meerk40t is Arduino code(firmware) for small DIY laser engarver that uses DVD/CD rom drive stepper motors, and can be controled by meerk40t software.WARNING : do not use this code on CO2 (K40) lasers! I't will not work! You will burn your home!WARNING : use at your own risk!## Whats working* movment (jogging)* cut/engrave speed* cuting* engraving	* Top to Bottom and Bottom to Top (tested)	* Left to Right and Right To Left	* CrossHatch (not tested)## Whats not working / implemented* Home (you have to home manual by jog buttons)* Pause* Stop* (de)acceleration in engrave mode## Hardware requirements* Arduino Nano/Uno (atmega328p mcu)* 2 x A4899 stepper drivers* optional grbl compatibile pcb* some laser## Wiring* Arduino D2 -> A4899 X-dir* Arduino D3 -> A4899 Y-dir* Arduino D5 -> A4899 X-step* Arduino D6 -> A4899 Y-step* Arduino D11 -> Laser ttl* Arduino D8 -> A4899 X/Y enable (rail lock/unlock)## How to use1. in arduino code set "steps in mm" variable to match your stepper motors and upload code to Arduino2. in meer40t under devices, set Driver/input to LhystudiosDriver, and Output to TCP output localhost:10223. open  python TCPtoSerial program and set correct COM port of your arduino4. run TCPtoSerial  (you need python 3.7)5. home your engraver with jog buttons (upper left corner), and then press Home button in meerk40t ## Control ProtocolControl protocol is same as Lhymicro M2-Nano, it's only using serial port to transfer data.Data is sent in 34 byte packets (0xA6 + 0x00 + 30 bytes of data +0xA6 + CRC).As Arduino have more memory it's would be better to have bigger packets, but as this code is for smal DIY engravers 34 byte packets are OK.--