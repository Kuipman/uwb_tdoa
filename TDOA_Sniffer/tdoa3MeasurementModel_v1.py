"""
tdoa3MeasurementModel_v1.py

v1 Script parses packets from an existing yaml file, generates localization approximations
"""

## Library imports
# General libraries
import sys
import struct
import serial
import yaml
import time
# ROS2
import rclpy    # don't forget to source your ROS2 configuration, else this will return an error
from rclpy.node import Node

#####   Global variables and Define values    #####
file_path = "/home/nick_k/Documents/UWB/uwb_tdoa/TDOA_Sniffer/output20240415.yaml"
anchor1_pos = [0, 0, 0]
anchor2_pos = [0, 4.5, 0]
anchor3_pos = [4.5, 0, 0]
anchor4_pos = [4.5, 4.5, 0]

DEBUG = True        # set to True for printouts to console


#####       Classes and Functions          #####
# RemoteData object assists with tracking individual information for each anchor remote to given anchor
class RemoteData:
    def __init__(self):
        self.id = 0
        self.distance_ticks = 0
        self.tdoa = 0         # tdoa between anchor and remote anchor wrt tag
        self.rxTimeStamp = 0
        self.seq = 0

# Packet object contains necessary information from each packet for TDoA calculations.
class Packet:
    def __init__(self):
        self.remoteList = [RemoteData(), RemoteData(), RemoteData()]   # Three remote anchors from transmitting anchor
        self.remoteCount = 0
        self.seq = 0
        self.rxTimeStamp = 0     # wrt tag clock
        self.txTimeStamp = 0     # wrt transmitting anchor clock
    
    def updateRemoteData(self, remoteID, remote_data):
        self.remoteList[remoteID] = remote_data

# Anchor object tracks static information of the anchor (id, coordinates) and the previously received packet from that anchor.
class Anchor:
    def __init__(self, position = None, id = 0):
        if(position is None):
            position = [0, 0, 0]
        self.position = position      # cartesian coordinates of anchor
        self.id = id                  # id of anchor
        self.prevPacket = Packet()    # previous packet received by tag from this anchor
    
    def update_packet(self, packet):
        self.prevPacket = packet

# Function allows for reading an individual packet from the yaml file
def read_packets(file_path):
    with open(file_path, 'r') as file:
        for data in yaml.safe_load_all(file):
            yield data

## Set anchors (static and unchanging)
anchors = [Anchor(anchor1_pos, 1), Anchor(anchor2_pos, 2), Anchor(anchor3_pos, 3), Anchor(anchor4_pos, 4)]

"""
Version 1: Loads existing data from a yaml file

For each packet in the yaml file:
    - Parse data into a packet object tracking the necessary information
    - Reference with id of remote anchor and respective anchor object
        - If no prior packet exists, save packet to anchor object and continue
        - If prior packet exists but a sequence number was skipped, save packet to anchor object and continue
        - If prior packet exists and sequence number matches up, calculate TDoA between the anchor object and respective remote anchors
"""
for newPacket in read_packets(file_path):  
    # def writePacketToAnchor(anchorID):
    #     anchors[anchorID].prevPacket.remoteCount = newPacket.get('remoteCount', 'N/A')  
    #     anchors[anchorID].prevPacket.seq         = newPacket.get('seq', 'N/A')
    #     anchors[anchorID].prevPacket.rxTimeStamp = newPacket.get('ts', 'N/A')
    #     anchors[anchorID].prevPacket.txTimeStamp = newPacket.get('txTimeStamp', 'N/A')
    #     # copy over the remoteAnchors
    
    # Create placeholder packet, fill with information from newPacket
    tempPacket     = Packet()   # Placeholder packet for easier transfer to anchor objects
    tempRemoteData = RemoteData()
    tempPacket.id          = newPacket.get('from', 'N/A')     # where did this latest packet come from?
    tempPacket.remoteCount = newPacket.get('remoteCount', 'N/A')
    tempPacket.seq         = newPacket.get('seq', 'N/A')
    tempPacket.rxTimeStamp = newPacket.get('ts', 'N/A')
    tempPacket.txTimeStamp = newPacket.get('txTimeStamp', 'N/A')
    counter = 0
    for remote in newPacket['remoteAnchorData']:
        tempRemoteData.distance_ticks = remote.get('distance', 'N/A')
        tempRemoteData.id             = remote.get('id', 'N/A')
        tempRemoteData.rxTimeStamp    = remote.get('rxTimeStamp', 'N/A')
        tempRemoteData.seq            = remote.get('seq', 'N/A')
        tempPacket.remoteList[counter] = tempRemoteData
        counter += 1
    
    """
    With the placeholder packet built, now check if this packet is the next expected packet of
    the respective anchor. If not, skip TDoA calculations and just record the packet
    """
    if tempPacket.seq is not (anchors[tempPacket.id - 1].prevPacket.seq + 1):
        print("Out of sequence!")
        anchors[tempPacket.id - 1].update_packet(tempPacket)
        continue    # skips all calculations and moves to next packet
    
    """
    If this packet is indeed the next expected packet, update the TDoA calculations for each pair of anchors.
    For each remote anchor, check that the recorded remote data in tempPacket is the next sequence number from
    the remote data of the packet currently contained in the anchor object. If this is NOT the case,
    that particular tdoa calculation is skipped
    """
    # Clock Correction = ratio of differences in timestamps between current and previous packets from anchor
    for remote in newPacket['remoteAnchorData']:




    # for packet in yaml.load_all(sys.stdin, Loader=yaml.Loader):
    # if not packet:
    #     continue

    # data = {'id': packet['from'], 'tof': {}}

    # for remote in packet['remoteAnchorData']:
    #     if 'distance' in remote:
    #         tof = remote['distance']
    #         remote_id = remote['id']
    #         if unit == 'ticks':
    #             data['tof'][remote_id] = tof
    #         if unit == 'meters':
    #             data['tof'][remote_id] = tof * M_PER_TICK - ANTENNA_OFFSET
    
    #writePacket(1)
    # check if respective anchor doesn't yet have a packet
    
    #if anchors[remoteID - 1].prevPacket.id is 0:

    




# for pack in read_packets(file_path):
#     if 'remoteAnchorData' in pack:
#         print(f"Packet Seq: {pack.get('seq', 'N/A')}")
#         for anchor in pack['remoteAnchorData']:
#             print(f" Anchor ID: {anchor['id']}, rxTimeStamp: {anchor['rxTimeStamp']}")

# For Version 2 (piping output directly)
"""
for packet in yaml.load_all(sys.stdin, Loader=yaml.Loader):
    if not packet:
        continue

    data = {'id': packet['from'], 'tof': {}}
"""

