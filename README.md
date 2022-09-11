# Wireless Doorbell
**Wireless doorbell transmitter (and Raspberry Pi AVR prototyping HAT)**

![Screenshot of the PCB in KiCAD](doc/pcb.png)

> **Warning**  
> This project is provided as-is and likely not of use to anyone else; at least not without modifications.
> **The author of this repository takes no responsibility for any damage resulting from the electronic devices described here.**
> **You are responsible to make sure that operating these devices meets all relevant regulatory requirements in your region.**

This repository contains the PCB and Firmware for a Si4463-based (HopeRF RFM26W) wireless doorbell operating in the 433 MHz ISM band.
Doorbell button presses are transmitted wirelessly from one PCB to the same PCB serving as a RaspberryPi hat.
Here the button press events are forwarded to [OpenHAB](https://www.openhab.org/) (via SerialBridge),
where they trigger all SIP phones in the house to ring via [FemtoSIP](https://github.com/astoeckel/femtosip).

The left side of the PCB is a Raspberry Pi compatible AVR Microcontroller hat with Arduino-compatible pin headers and space
for the HopeRF RFM26W module (for the ATmega168/ATMega328 family).
The right side of the PCB is an low-voltage AC power supply with 50 Hz pulse extraction; the 
The right side of the PCB is designed to be sawed off if not in use.
Since PCB prototyping services often produce at least 5-10 PCBs, the PCB was designed with some versatility in mind,

## Overview

![Overview of the setup](doc/overview.png)

The above diagram depicts how the PCB is inteded to be used.
One instance of the PCB is directly powered by the low-voltage doorbell AC transformer; the other instance is powered by the 5V power supply of a Raspberry Pi.

Button presses are recognized by the doorbell button shorting out the yellow and black wires in the above diagram.
Correspondingly, the 50 Hz oscillations disappear on the yellow wire; this is recognized by the ATmega168 microcontroller which sends a few digital packets of the form
```
$DBG,<BUTTON PRESSED>,<SEQ>
```
to the receiver at a baud rate of 10kbps.
Each packet is repeated five times.

## Schematics and Gerber files

* **Gerber files**  
  The Gerber files used for production can be found in the `gerber` folder (the author is not aware of any bugs on the PCB).
* **Schematic and KiCAD sources**  
  The KiCAD source files are in the `kicad` folder. A printout of the schematics in [PDF form](doc/schematics.pdf) may be found in the `doc` folder.

## Firmware

The firmware is located in the `code` folder.
It uses `avr-gcc` and `avr-libc`, as well as `avrdude` with the [linuxspi patches](https://github.com/kcuzner/avrdude).

It can be compiled and flashed using [meson](https://mesonbuild.com/) as follows:
```sh
cd code; mkdir -p build
meson setup --cross-file ../avr-unknown-gcc.txt ..
ninja
ninja flash  # Requires avrdude with linuxspi when flashing directly from the RPi
```

## Manual

The firmware automatically chooses whether it acts as the receiver or transceiver depending on the presence of the
50 Hz pulse from the yellow wire in the above diagram.

### Receiver

1. Connect UART RX/TX pins and 5V power (use a baud rate of 115200) to Raspberry Pi
2. The `RX` LED should be illuminated; the LED will blink rapidly if the receiver receives any data.
3. Incoming messages will be printed as a one-line JSON-encoded string to UART
4. Shortly pressing the SMD button on the board will send a `PING` request. If any board responds with a `PONG`, both the `RX` and `TX` LED will blink rapidly for two seconds. *Note:* The transmitter is usually in energy saving mode and will not respond. Wake the transmitter up by pressing the SMD button there.
5. Pressing the SMD button for at least 3 seconds will reset the µC on the board.

### Transmitter
1. Connect the three doorbell wires as indicated in the diagram above.
2. The `RX` led will blink once every second if the 50 Hz pulse has been recognized; the board will go to a 3-4mA power-save mode (this is to prevent the linear voltage regulator from becoming too hot).
3. If the 50 Hz signal goes away, the transmitter will send a message to the receiver as described above. The `TX` LED should blink rapidly.
4. Shortly pressing the SMD button will send a `PING` request. Furthermore, the transmitter will leave power-save mode for one minute; it can now be pinged by the receiver.
5. Pressing the SMD button for 3 seconds will reset the µC on the board.

## License

Note that this project consists of both hardware and to a smaller portion of software. Both parts are made available under different licenses.

### Hardware schematics

    Copyright Andreas Stöckel 2022.
    
    This documentation describes Open Hardware and is licensed under the
    CERN OHL v. 1.2. You may redistribute and modify this documentation under
    the terms of the CERN OHL v.1.2. (http://ohwr.org/cernohl). This
    documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
    INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A
    PARTICULAR PURPOSE. Please see the CERN OHL v.1.2 for applicable conditions.

### Software (AVR firmware and host programs)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
