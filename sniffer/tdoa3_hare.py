"""
tdoa3_hare.py

A script to parse the incoming byte stream from the LPS node in "sniffer" mode to readable
tof/meter measurements usable by a separate multilateration system enabling UWB localization.
"""

# Note: LPS node should already be in sniffer mode. Consult LPS documentation https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/development/anchor-low-level-config/
# for instructions on how to do this in your terminal

# Library imports

import sys
import struct
import serial
import yaml
import time
import rclpy
from rclpy.node import Node

# Global variables and Define values
unit = 'meters'      # we always want meters
ANTENNA_OFFSET = 154.6
LOCODECK_TS_FREQ = 499.2e6 * 128
SPEED_OF_LIGHT = 299792458.0
M_PER_TICK = SPEED_OF_LIGHT / LOCODECK_TS_FREQ

# Read a line from the serial port and pass it on


if len(sys.argv) < 1:
    print("usage: {} <sniffer serial port> [format]".format(sys.argv[0]))
    sys.exit(1)

ser = serial.Serial(sys.argv[1], 9600)
# ser = serial.Serial('/dev/ttyACM0', 9600)   # remove when not debugging

if len(sys.argv) > 1:
    outputFormat = sys.argv[1].strip()

# Switch to binary mode
ser.write(b'b')

# flag sets to true when a usable binary string is read
binaryFlag = False

while True:

# Read a line from the serial input. Sets the binary flag when the line is successfully read
    while not binaryFlag:
        c = ser.read()
        if c == b'\xbc':
            ts = ser.read(5)
            ts += b'\0\0\0'
            ts = struct.unpack('<Q', ts)[0]
            addrFrom, addrTo = struct.unpack('<BB', ser.read(2))
            length = struct.unpack('<H', ser.read(2))[0]
            if length > 1024:
                continue
            data = ser.read(length)
            l2 = struct.unpack('<H', ser.read(2))[0]
            if length == l2:
                rxSys = time.time()
                binaryFlag = True      # break the loop and continue
            else:
                print("Out of sync!")

# Next step: decode the current information and transform to meters
    packet = {}
    packet['from']  = addrFrom
    packet["data"]  = data
    packet['rxSys'] = rxSys
    packet['to']    = addrTo
    packet['ts']    = ts

    packet["type"] = packet["data"][0]
    # packetType = packet["data"][0]
    # packet["type"] = packetType
    if packet["type"] == 0x30:
        decoded = struct.unpack("<BBLB", packet["data"][:7])
        packet["seq"] = decoded[1]
        packet["txTimeStamp"] = decoded[2]

        remote_count = decoded[3]
        packet["remoteCount"] = remote_count

        packet['remoteAnchorData'] = []
        anchor_data_index = 7

        for i in range(remote_count):
            decoded_anchor_data = struct.unpack(
                '<BBL',
                packet["data"][anchor_data_index:(anchor_data_index + 6)])
            seq = decoded_anchor_data[1] & 0x7f
            anchor_data = {
                'id': decoded_anchor_data[0],
                'seq': seq,
                'rxTimeStamp': decoded_anchor_data[2],
            }
            anchor_data_index += 6

            has_distance = ((decoded_anchor_data[1] & 0x80) != 0)
            if (has_distance):
                decoded_distance = struct.unpack(
                    '<H',
                    packet["data"][anchor_data_index:(anchor_data_index + 2)])
                anchor_data['distance'] = decoded_distance[0]
                anchor_data_index += 2

            packet['remoteAnchorData'].append(anchor_data)

        if len(packet["data"]) > anchor_data_index:
            if packet["data"][anchor_data_index] == 0xf0 and \
                    packet["data"][anchor_data_index + 1] == 0x01:
                packet["lpp_data"] = {}
                packet["lpp_data"]['header'] = packet["data"][
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

# Finally, calculate the distance between the two anchors in meters
    packet['tof'] = {}
    for remote in packet['remoteAnchorData']:
        if 'distance' in remote:
            tof = remote['distance']
            remote_id = remote['id']
            if unit == 'ticks':
                packet['tof'][remote_id] = tof
            if unit == 'meters':
                packet['tof'][remote_id] = tof * M_PER_TICK - ANTENNA_OFFSET

    print("---")
    print(yaml.dump(packet, Dumper=yaml.Dumper))

    # At end of file, reset the binary Flag so the next line can be read
    binaryFlag = False
