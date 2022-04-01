# SH72AL
Firmware for the [alternative SH72 soldering iron controller](https://oshwlab.com/AntaresLab/sh72al1) with turbo mode, two-stage sleep mode, grounding the soldering iron tip to a "-" power wire and the protection against the reverse polarity, short circuit and overvoltage.
The firmware is designed for the ATtini13(A) MCU and works with the factory-setted fuse bits.
PCB project and other more complete information can be found [here](https://oshwlab.com/AntaresLab/sh72al1).

Warning! Due to the low clock speed setting, after flashing the MCU programmer stops to communicate it. To return the ability to program the MCU, reset pin should be shorted to GND before powering the board (MCU shouldn't be started before switching MCU to the program mode). Thanks to Ignacio Vazquez-Abrams from electronics.stackexchange.com :)

Pay attention that the first commits were made for another PCB. Last commits are applicable for the ver.1 rev.C PCB.



