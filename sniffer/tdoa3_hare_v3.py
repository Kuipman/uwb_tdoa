"""
tdoa3_hare_v3.py

A script to parse the incoming byte stream from the LPS node in "sniffer" mode to readable
tof/meter measurements and timestamps. The latter timestamps are then used in a rough TDoA
calculation to estimate the position of the tag.

@todo
    - Set Anchor Positions. For test these will be static, need a way to easily change these or even make them dynamic
    - Timestamp-based TDoA
"""

# Note: LPS node should already be in sniffer mode. Consult LPS documentation https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/development/anchor-low-level-config/
# for instructions on how to do this in your terminal

## Library imports
# General libraries
import sys
import struct
import serial
import yaml
import time
import numpy as np
# ROS2
import rclpy    # don't forget to source your ROS2 configuration, else this will return an error
from rclpy.node import Node

## Global variables and Define values
unit = 'meters'      # we always want meters
ANTENNA_OFFSET = 154.6
LOCODECK_TS_FREQ = 499.2e6 * 128
SPEED_OF_LIGHT = 299792458.0
M_PER_TICK = SPEED_OF_LIGHT / LOCODECK_TS_FREQ

# @check -- these need to be updated with actual test conditions
anchorPos_1 = {0, 0, 0}       # coordinates are in meters
anchorPos_2 = {0, 0, 0}
anchorPos_3 = {0, 0, 0}
anchorPos_4 = {0, 0, 0}
# anchorPos_5 = {5, 0, 0}
# anchorPos_6 = {5, 0, 0}

## First argument when running this file ('...py /dev/ttyACM0 ...) should be the serial port at 9600 baud
if len(sys.argv) < 1:
    print("usage: {} <sniffer serial port> [format]".format(sys.argv[0]))
    sys.exit(1)
ser = serial.Serial(sys.argv[1], 9600)
# ser = serial.Serial('/dev/ttyACM0', 9600)   # debug line

# If more arguments are called, these are read into the outputFormat holder
if len(sys.argv) > 1:
    outputFormat = sys.argv[1].strip()

# Switch the node to binary mode (this is impossible in the terminal menu for some reason)
ser.write(b'b')

# flag sets to true when a usable binary string is read
binaryFlag = False

## With the device set up, let us sniff without end.
while True:

# Read a line from the serial input. Break from this loop when the line is fully read
    while not binaryFlag:
        c = ser.read()                                             # reads a byte from the serial port. This cycles until the header 'bc' is found
        if c == b'\xbc':                                           # Header byte (start byte) for a packet is 'bc.' Once this is read the system can parse the rest of the packet appropriately.
            ts = ser.read(5)                                       # Next five bytes are the tx timestamp of the packet
            ts += b'\0\0\0'                                        # TS is appended with 3 more bytes so this can be unpacked into a 64-bit integer
            ts = struct.unpack('<Q', ts)[0]                        # <Q denotes a little-endian 64-bit unsigned integer 'Q'
            addrFrom, addrTo = struct.unpack('<BB', ser.read(2))   # Next two bytes are the source (addrFrom) and destination (addrTo) addresses of the packet. 
            length = struct.unpack('<H', ser.read(2))[0]           # Next two bytes denote the length of the following data section of the packet
            if length > 1024:                                      # Packets with larger than 1024-byte data sections are discarded
                continue
            data = ser.read(length)                                # Reads the data section into the 'data' variable
            l2 = struct.unpack('<H', ser.read(2))[0]               # Next two bytes act as the 'checksum' and are identical to the data length read earlier - these are compared
            if length == l2:                                       # If the length and l2 are not identical the 'checksum' fails and the packet is discarded
                rxSys = time.time()                                # Records the receive timestamp (rxSys) of the packet using the local counter
                binaryFlag = True                                  # Packet is complete. Breaks the loop and continue
            else:
                print("Out of sync!")

# Decode the current information into something much more useful
    packet = {}                     # structure holding all useful information in regards to the packet
    packet['from']  = addrFrom      # Source address
    packet["data"]  = data          # Packet data
    packet['rxSys'] = rxSys         # Timestamp when packet was received at this node
    packet['to']    = addrTo        # Destination address
    packet['ts']    = ts            # Timestamp when packet was sent from source (based on source's local clock)

    packet["type"] = packet["data"][0]                        # Detect type of packet, ranging = 0x30
    if packet["type"] == 0x30:
        decoded = struct.unpack("<BBLB", packet["data"][:7])  # First seven bytes of packet data are unpacked into decoded variable
        packet["seq"] = decoded[1]                            # Sequence number of packet (wrt the destination anchor the source is communicating with)
        packet["txTimeStamp"] = decoded[2]                    # Transmission timestamp (4 bytes - coincides with long in BBLB above)

        remote_count = decoded[3]                             # number of remote anchors included in the packet
        packet["remoteCount"] = remote_count

        packet['remoteAnchorData'] = []                       # empty list to hold data about remote anchors
        anchor_data_index = 7                                 # start reading remote anchor data from the 8th byte of packet["data"]

        """
        Iterates over the number of remote anchors. Unpacks and processes each anchor's data:
            - Extracts each remote anchor's ID, sequence number, and receive timestamp (rxTimeStamp)
            - Checks if distance data is present and, if so, unpacks the distance value
            - Appends the extracted data to packet['remoteAnchorData']
        """
        for i in range(remote_count):
            decoded_anchor_data = struct.unpack(                             # unpacks 6 bytes: ID, seq, rxTimeStamp and assigns to decoded_anchor_data
                '<BBL',
                packet["data"][anchor_data_index:(anchor_data_index + 6)])
            seq = decoded_anchor_data[1] & 0x7f                              # removes the "distance presence indicator bit" and leaves behind the sequence number (0-127)
            anchor_data = {                                                  # anchor_data dictionary (think a struct in c)
                'id': decoded_anchor_data[0],
                'seq': seq,
                'rxTimeStamp': decoded_anchor_data[2],
            }
            anchor_data_index += 6                                           # move the index past the data just read so next iteration reads the next set of data

            has_distance = ((decoded_anchor_data[1] & 0x80) != 0)            # is the "distance presence indicator bit" set? If so, has_distance is true
            if (has_distance):             
                decoded_distance = struct.unpack(                            # Next two bytes unpacked as a short and assigned as distance
                    '<H',
                    packet["data"][anchor_data_index:(anchor_data_index + 2)])
                anchor_data['distance'] = decoded_distance[0]                # distance is appended to the anchor_data dictionary
                anchor_data_index += 2                                       # index is moved by 2 bytes to pass the distance value in data

            packet['remoteAnchorData'].append(anchor_data)                   # Finally, the anchor_data dictionary is moved to the remoteAnchorData element in packet

        if len(packet["data"]) > anchor_data_index:                          # Checks if there is more data after the current index position
            if packet["data"][anchor_data_index] == 0xf0 and \
                    packet["data"][anchor_data_index + 1] == 0x01:           # Checks if the next bytes in the packet data are LPP data by checking for the pattern 0xf0 and 0x01
                packet["lpp_data"] = {}
                packet["lpp_data"]['header'] = packet["data"][               # Extract and store the LPP header
                    anchor_data_index]
                anchor_data_index += 1

                packet["lpp_data"]['type'] = packet["data"][anchor_data_index]
                anchor_data_index += 1

                decoded = struct.unpack(
                    "<fff",
                    packet["data"][anchor_data_index:anchor_data_index + 3 * 6]
                )
                packet["lpp_data"]['position'] = {}
                packet["lpp_data"]['position']['x'] = decoded[0]
                packet["lpp_data"]['position']['y'] = decoded[1]
                packet["lpp_data"]['position']['z'] = decoded[2]
                anchor_data_index += 3 * 6
            else:
                packet["lpp_data"] = packet["data"][anchor_data_index:]

    """
    Finally, calculate the Time of Flight (ToF) and distances
        - Initialize an empty directory for ToF measurements
        - Iterate over each remote anchor's data, calculating the ToF based on the distance (if provided) and converting
            to meters
        - Adjusts ToF by subtracting the Antenna Offset, then stores the result in packet['tof'] keyed by the remote anchor ID
        - Prints out the resulting ToF onto the terminal
    """
    packet['tof'] = {}
    for remote in packet['remoteAnchorData']:
        if 'distance' in remote:
            tof = remote['distance']
            remote_id = remote['id']
            if unit == 'ticks':
                packet['tof'][remote_id] = tof
            if unit == 'meters':
                packet['tof'][remote_id] = tof * M_PER_TICK - ANTENNA_OFFSET

    # Check if the 'tof' dictionary is not empty
    if packet['tof']:
        print("ToF Measurements in Meters:")
        for anchor_id, tof_measurement in packet['tof'].items():
            print(f"Anchor ID {anchor_id}: {tof_measurement} meters")
    else:
        print("No ToF measurements available.")


    # print("---")
    # print(yaml.dump(packet, Dumper=yaml.Dumper))

    # Test print to console
        # print("ToF Measurements:")
        # for distance in 
        # for anchor_id, tof_measurement in packet['tof'].items():
        #     print("Anchor ID " + anchor_id + ":" + tof_measurement + "meters")

    # At end of file, reset the binary Flag so the next line can be read
    binaryFlag = False
