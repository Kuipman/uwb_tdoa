# UWB_TDoA3

This project implements a GPS-free, autonomous position-estimation system based on Ultra-Wideband (UWB) Time Difference of Arrival (TDoA) localization. The system is specifically designed for environments in which GPS is unreliable or unavailable, including indoor spaces, dense vegetation, and cluttered or infrastructure-heavy industrial settings.

Time Difference of Arrival (TDoA) is a radio-based localization technique in which the relative arrival times of wireless signals are used to estimate position. In a traditional TDoA configuration, a mobile node (tag) periodically transmits messages that are received by multiple fixed (anchor) nodes. Each anchor records the signal arrival time, and the differences between these timestamps are used to compute the tag’s position.

![Image of Hyperbolic Localization using TDoA](/resources/Plots_Hyperbolas2.png)

This project implements a reverse TDoA architecture. Instead of the mobile node transmitting, a set of static anchor nodes periodically transmit and exchange synchronized UWB messages among themselves. The mobile tag passively intercepts these transmissions and computes its own three-dimensional position using the embedded timing information and local reception timestamps. By shifting the computational and transmission burden away from the mobile node, this approach significantly reduces power consumption and system complexity on the tag while maintaining high localization accuracy. This makes the system well suited for autonomous robotics, embedded platforms, and long-duration deployments.

This project is implemented as part of an undergraduate thesis for the department of Electrical and Computer Engineering at University of California Santa Cruz. The full thesis is available [here](/resources/A_Study_of_Ultra-Wideband_Localization_Using_Reverse_Time-Difference_of_Arrival__KuipersNS.pdf).

## Author

Nick Kuipers ([Kuipman](https://github.com/Kuipman))

## Status

Project generates a 3D position estimation of a mobile node in real-time. Undergraduate thesis associated with this repository is complete as of June 2024.

This library is currently (2024) **not actively maintained** by [Kuipman](https://github.com/Kuipman)

Areas of improvement noted by author:
<br>- Z-coordinate estimation has a large variance
<br>- Python scripts can easily be combined
<br>- TDoA tag firmware can filter out unnecessary packet information before being sent to Python scripts

## License

Apache 2.0 (see LICENSE)

## Hardware

The localization system is built using Ultra-Wideband (UWB) nodes from the Bitcraze Loco Positioning System (LPS) (see https://www.bitcraze.io/products/loco-positioning-node/), which are based on Decawave’s DW1000 UWB transceiver and provide the timing precision required for Time Difference of Arrival (TDoA) positioning.

Multiple stationary base-station nodes (anchors) were deployed at known, fixed locations within the test environment. Each anchor was mounted on a tripod using custom 3D-printed mounting brackets, allowing for flexible placement, repeatable geometry, and accurate measurement of anchor coordinates.

The mobile node (tag) was mounted on a dummy aerial platform to emulate a real-world robotic payload (e.g., a drone or mobile robot). This configuration allowed the system to be tested under realistic physical constraints without risking damage to an operational vehicle.

A Dell laptop was physically connected to the mobile platform and served as the primary computation unit. It received UWB packet timing data from the mobile node and performed real-time position estimation, including TDoA computation and multilateration to determine the mobile node’s three-dimensional position.

# Software

Loco Positioning System base firmware pulled from Bitcraze (see https://github.com/bitcraze/lps-node-firmware). The base firmware includes operational modes for Two-Way Ranging (TWR) and Transmit-Only TDoA at the mobile node. This project adds the capability for the mobile node to receive, process, and forward base station messages over serial port as part of Receive-Only TDoA. This portion of the implementation can be found in the *lpsTdoa3Tag* library in src/.

The remainder of Receive-Only TDoA is performed at the laptop-side (simulating the mobile device's OS) using a set of Python scripts. These receive base station messages over serial from the mobile LPS node, convert them to Time-of-Flight (ToF) and equivalent distance (in meters) measurements, and uses Least-Squares approximation to estimate the 3D position of the mobile node in real-time. The complete software stack operates as follows:

```
Modified LPS Firmware
    |
	| (Receive-Only ranging packets)
	|
	V
tdoa3PacketBuilder.py
    |
	| (ToF/Meter ranges)
	|
	V
tdoa3MeasurementModel_v1.py
    |
	|
	V
*3D Position Estimation*
```

## Configuring the Loco Positioning System Nodes for TDoA3

(Configuring the LPS via Console Menu): Through a simple serial connection an LPS node can be configured on a serial console. With a device connected and its port known, you can use the command "picocom /dev/'PORT'" in your terminal. Further instructions can be found here: https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/development/anchor-low-level-config/

(TDoA3 Configuration): The anchors and sniffer must be configured to the same bitrate and preamble length. By default, these are both "normal". This can be changed in the console menu.

(For TDoA3 Long Range): The anchors need to be configured with the radio mode "low bitrate, normal preamble" as well as TX power set to max. Instructions for how to achieve this are located here: https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/user-guides/tdoa3_long_range/


## LPS Node Firmware Programming (i.e. not using proprietary modes and software)

(Loco Positioning Node - https://www.bitcraze.io/products/loco-positioning-node/)

The LPS node operates using an STM32 microprocessor. Interfacing with this processor can be performed with VSCode and/or Arduino IDE provided you install the STM32duino extension. You will also need the STM32 Cube Programmer, downloaded separately.

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
