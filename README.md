# DW1000_HARE

Library of basic functionalities for Decawave's DW1000 chips/modules with Arduino -- edited for use with HARE lab projects. (DW1000 - https://www.decawave.com/products/dwm1000-module) Continued from DW1000 library by thotro (see https://github.com/thotro/arduino-dw1000)

## License

Apache 2.0 (see LICENSE)

## Operation Instructions for DW1000 connected to Loco Positioning Node

(Loco Positioning Node - https://www.bitcraze.io/products/loco-positioning-node/)

The DW1000 Loco Positioning Node (hereafter referred to as simply the "node") operates using an STM32 microprocessor. Interfacing with this processor can be performed within the Arduino IDE provided you install the STM32duino extension. Similarly, interfacing with this processor can be accomplished in Visual Studio Code with the Arduino extension + the STM32duino add-on.

You will also need the STM32 Cube Programmer, downloaded separately.


Settings for downloading software to the node via Arduino IDE are undertaken as follows:
1: Connect the node to your system via USB. Port connectivity can be checked in the Tools tab of the Arduino IDE, or in your dev folder if you are on a Linux platform.
2: Note the two buttons on the node. Hold the left button down and press the right button. This will transition the node to DFU mode. (NOTE: The node may no longer be seen as an active port upon entering DFU mode. This is fine -- node can still program normally)
3: On Arduino IDE: (our node is the F072CBT6 series, yours may differ)
	Tools -> Board -> STM32 boards groups -> Generic STM32F0 series
	Tools -> Board part number -> Generic F072CBTx
	Tools -> U(S)ART Support -> Enabled (generic 'serial')
	Tools -> USB Support -> CDC (generic 'Serial' supersede U(S)ART)        (Not necessary, but fixed a serial printing issue)
	Tools -> Upload Method -> STM32CubeProgrammer (DFU)
	Everything else -> Default
4: When your program is ready for upload, hit upload as you would for a regular Arduino microcontroller. If the console indicates several instances of "erasing sector" followed by "File Download Complete" you're good to go.
	

