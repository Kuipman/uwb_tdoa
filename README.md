# UWB_TDoA3

Project utilizing a Time Difference of Arrival (TDoA) system utilizing a receive-only mobile tag calculating its xyz coordinate using packet data sent from stationary base station modules.

UWB Nodes used are from the Loco Positioning System (see https://www.bitcraze.io/products/loco-positioning-node/)

Manual DW1000 source pulled from library by thotro (see https://github.com/thotro/arduino-dw1000). Library contains basic functionalities for Decawave's DW1000 chips/modules with Arduino. (DW1000 - https://www.decawave.com/products/dwm1000-module)

## License

Apache 2.0 (see LICENSE)

## Configuring the Loco Positioning System Nodes for TDoA3

Time Difference of Arrival (TDoA) is a ranging setup in which a series of static anchors exchange ranging messages and a sniffer node "sniffs" or listens to these messages.

(Configuring the LPS via Console Menu): Through a simple serial connection an LPS node can be configured on a serial console. With a device connected and its port known, you can use the command "picocom /dev/'PORT'" in your terminal. Further instructions can be found here: https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/development/anchor-low-level-config/

(TDoA3 Configuration): The anchors and sniffer must be configured to the same bitrate and preamble length. By default, these are both "normal". This can be changed in the console menu.

(For TDoA3 Long Range): The anchors need to be configured with the radio mode "low bitrate, normal preamble" as well as TX power set to max. Instructions for how to achieve this are located here: https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/user-guides/tdoa3_long_range/


## Operation Instructions for direct programming on the LPS Nodes (i.e. not using proprietary modes and software)

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
	

