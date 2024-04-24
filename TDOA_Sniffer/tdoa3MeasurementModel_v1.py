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
import numpy as np
# ROS2
import rclpy    # don't forget to source your ROS2 configuration, else this will return an error
from rclpy.node import Node

#####   Global variables and Define values    #####
file_path = "/home/nick_k/Documents/UWB/uwb_tdoa/TDOA_Sniffer/output20240415.yaml"
anchor1_pos = [0, 0, 0]
anchor2_pos = [0, 4.5, 0]
anchor3_pos = [4.5, 0, 0]
anchor4_pos = [4.5, 4.5, 0]

DEBUG = False        # set to True for printouts to console


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
anchorList = [Anchor(anchor1_pos, 1), Anchor(anchor2_pos, 2), Anchor(anchor3_pos, 3), Anchor(anchor4_pos, 4)]


#####     Program Start     #####
"""
Version 1: Loads existing data from a yaml file

For each packet in the yaml file:
    - Parse data into a tempPacket object (math can be done directly without the object, but it helps visually)
    - Reference with id of remote anchor and respective anchor object
        - If no prior packet exists, save packet to anchor object and continue
        - If prior packet exists but a sequence number was skipped, save packet to anchor object and continue
        - If prior packet exists and sequence number matches up, calculate TDoA between the anchor object and respective remote anchors
"""
for newPacket in read_packets(file_path):  
    def calculateTdoa(alpha, d_rx, d_tx):
        return d_rx - (alpha * d_tx)
    
    # Create placeholder packet, fill with information from newPacket
    tempPacket     = Packet()   # Placeholder packet for easier transfer to anchor objects
    tempPacket.id          = newPacket.get('from', 'N/A')     # where did this latest packet come from?
    tempPacket.remoteCount = newPacket.get('remoteCount', 'N/A')
    tempPacket.seq         = newPacket.get('seq', 'N/A')
    tempPacket.rxTimeStamp = newPacket.get('ts', 'N/A')
    tempPacket.txTimeStamp = newPacket.get('txTimeStamp', 'N/A')
    counter = 0
    for remote in newPacket['remoteAnchorData']:        # record each set of remote anchor information
        tempPacket.remoteList[counter].distance_ticks = remote.get('distance', 'N/A')
        tempPacket.remoteList[counter].id             = remote.get('id', 'N/A')
        tempPacket.remoteList[counter].rxTimeStamp    = remote.get('rxTimeStamp', 'N/A')
        tempPacket.remoteList[counter].seq            = remote.get('seq', 'N/A')
        counter += 1
    
    """
    With the placeholder packet built, now check if this packet is the next expected packet of
    the respective anchor. If not, skip TDoA calculations and just record the packet.

    The lps_tdoa3 application cycles through all anchors sequentially, so ideally this statement will initially trigger the same amount of times
    as there are anchors
    """
    if tempPacket.seq is not (anchorList[tempPacket.id - 1].prevPacket.seq + 1):   # checks seq of current packet in respective anchor
        if DEBUG:
            print("Unexpected sequence number, or empty anchor. Recording packet to anchor and reading next packet.")
        anchorList[tempPacket.id - 1].update_packet(tempPacket)
        continue    # skips all calculations and moves to next packet
    
    """
    If this packet is the next expected packet for the respective anchor, update the TDoA calculations for this anchor and each remote anchor.
    For each remote anchor, check that the recorded remote data in tempPacket is the next sequence number from
    the remote data of the packet currently contained in the anchor object. If this is NOT the case,
    that particular tdoa calculation is skipped (this includes anchors that haven't yet received a packet)
    """
    # First we calculate clock correction = ratio of differences in timestamps between current and previous packets from anchor
    d_rx1 = tempPacket.rxTimeStamp - anchorList[tempPacket.id - 1].prevPacket.rxTimeStamp     # P3_rx - P1_rx
    d_tx1 = tempPacket.txTimeStamp - anchorList[tempPacket.id - 1].prevPacket.txTimeStamp     # P3_tx - P1_tx
    alpha = d_rx1/d_tx1    # clock correction variable
    if alpha < 0.999 or alpha > 1.001:     # check for clock correction outliers. These are discarded
        anchorList[tempPacket.id - 1].update_packet(tempPacket)
        continue
    if DEBUG:
        print(f"Clock Correction. Alpha = {alpha}.")
    for remote in tempPacket.remoteList:       # cycle through each remote anchor, calculate tdoa where able
        if remote.seq is not anchorList[remote.id - 1].prevPacket.seq:            # check that tempPacket is tracking the same sequence number from P2. Here, these need to be identical
            if DEBUG:
                print(f"Sequence mismatch between P2 and P3. Skipping anchor {remote.id}")
            continue            # skips this anchor, moves on to next one

        """
        Now, calculate TDoA

        d_rx = rxTimestamp of current packet - rxTimestamp of previous packet from same anchor

        d_tx = P3_tx - P2_tx. These are with respect to two different clocks, so the time of flight between the two anchors
        needs to be considered. This is given in the distance_ticks variable of the remote anchor. So,
        d_tx = P3_tx - (P2_rx - distance_ticks)
        """
        # for debug purposes these are placed here, remove later
        P3_rx       = tempPacket.rxTimeStamp
        P3_tx       = tempPacket.txTimeStamp
        P2_rx       = anchorList[remote.id - 1].prevPacket.rxTimeStamp
        P2_tx       = anchorList[remote.id - 1].prevPacket.txTimeStamp
        P2_anchorRX = remote.rxTimeStamp
        tof         = remote.distance_ticks

        d_rx = P3_rx - P2_rx
        # d_rx = tempPacket.rxTimeStamp - anchorList[remote.id - 1].prevPacket.rxTimeStamp
        if d_rx < 0:   # timer wrapped
            d_rx = d_rx + 1099511627775    # 40-bit unsigned integer max

        d_tx = P3_tx - (P2_anchorRX - tof)
        # d_tx = tempPacket.txTimeStamp - (remote.rxTimeStamp - tof)
        if d_tx < 0:   # timer wrapped
            d_tx = d_tx + 4294967296       # 32-bit unsigned integer max

        tdoa_real = np.abs(d_rx - (alpha * d_tx))     # always positive
        
        # Average the tdoa with its previous value (eventually, replace this with Kalman filtering)




        # print(f"Clock Correction: {alpha}")
        # print(f"TDoA:  {tempPacket.id} -> {remote.id}  = {tdoa_real}")
        # if(tempPacket.id == 2 and remote.id == 1):
        #     print(f"TDoA:  {tempPacket.id} -> {remote.id}  = {tdoa_real}")
        M_Tick = 0.004691764
        if(tempPacket.id == 3 and remote.id == 1):
            print(f"TDoA:  {tempPacket.id} -> {remote.id}  = {tdoa_real * M_Tick}")
        
        



        

    """
    Spare code below, ignore for now
    """


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

