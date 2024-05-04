"""
tdoa3MeasurementModel_v1.py

Last Updated: 20240502

v1 Script parses packets from an existing yaml file and generates localization approximates

1) Parses yaml file one packet at a time
2) If good packet, calculates tdoa between remote anchor and each anchor it communicated with previously
3) tdoa used in hyperbolic measurement model
"""

#####     Library imports     #####
import sys
import struct
import serial
import yaml
import time
import numpy as np
# For ROS2:
import rclpy    # don't forget to source your ROS2 configuration, else this will return an error
from rclpy.node import Node


#####   Global Constants (TUNING VALUES)   #####
FILE_PATH = "/home/nick_k/Documents/UWB/uwb_tdoa/TDOA_Sniffer/output20240415.yaml"
ANCHOR1_POS = [0, 0, 0]
ANCHOR2_POS = [0, 4.5, 0]
ANCHOR3_POS = [4.5, 0, 0]
ANCHOR4_POS = [4.5, 4.5, 0]
TDOA_BOUND_LOW  = 0.5
TDOA_BOUND_HIGH = 2
M_TICK = 0.004691764
SPEED_OF_LIGHT = 299792458.0
DEBUG = False                  # set to True for debug printouts to console


#####       Global Classes and Functions       #####
# RemoteData object assists with tracking individual information for each anchor remote to given anchor
class RemoteData:
    def __init__(self, id = 0):
        self.id             = id
        self.distance_ticks = 0
        self.tdoa           = 0         # tdoa between anchor and remote anchor wrt tag
        # self.tdoaList       = [0, 0, 0] # add this maybe later
        self.rxTimeStamp    = 0
        self.seq            = 0

# Packet object contains necessary information from each packet for TDoA calculations.
class Packet:
    def __init__(self):
        # up to three anchors remotely communicate with source anchor. For simplicity ids 1 - 4 are tracked in
        # a list, but only three are used according to their id
        self.remoteList  = [RemoteData(1), RemoteData(2), RemoteData(3), RemoteData(4)]
        self.remoteCount = 0
        self.seq         = 0
        self.rxTimeStamp = 0     # wrt tag clock
        self.txTimeStamp = 0     # wrt transmitting anchor clock
    
    def updateRemoteData(self, remoteID, remote_data):
        self.remoteList[remoteID] = remote_data

# Anchor object tracks static information of the anchor (id, coordinates) and the previously received packet from that anchor.
class Anchor:
    def __init__(self, position = None, id = 0):
        if(position is None):
            position = [0, 0, 0]
        self.position   = position      # cartesian coordinates of anchor
        self.id         = id            # id of anchor
        self.prevPacket = Packet()      # previous packet received by tag from this anchor
    
    # replaces tracked packet with latest packet pulled from bitstream
    def update_packet(self, packet):
        self.prevPacket = packet
    
    # Updates the packet, skips updating the tdoa calculations in remote anchor info
    def skinny_update_packet(self, packet):
        self.prevPacket.remoteCount = packet.remoteCount
        self.prevPacket.seq         = packet.seq
        self.prevPacket.rxTimeStamp = packet.rxTimeStamp
        self.prevPacket.txTimeStamp = packet.txTimeStamp
        for remote in packet.remoteList:
            if packet.id is not remote.id:
                id = remote.id
                self.prevPacket.remoteList[id - 1].distance_ticks = remote.distance_ticks
                self.prevPacket.remoteList[id - 1].rxTimeStamp    = remote.rxTimeStamp
                self.prevPacket.remoteList[id - 1].seq            = remote.seq


# Set anchors (positions are static and unchanging)
anchorList = [Anchor(ANCHOR1_POS, 1), Anchor(ANCHOR2_POS, 2), Anchor(ANCHOR3_POS, 3), Anchor(ANCHOR4_POS, 4)]

################################################################
#####                 Global Functions                     #####
################################################################

# # Allows for reading an individual packet from the yaml file
def read_packets(file_path):
    with open(file_path, 'r') as file:
        for data in yaml.safe_load_all(file):
            yield data

# Non-Kalman filter for tdoa outliers
# Just filters out outliers and averages the tdoa values
def tdoaLazyEstimator(anchor_id, remote_id, newTdoa):
    if anchorList[anchor_id - 1].prevPacket.remoteList[remote_id - 1].tdoa == 0:     # initial tracked tdoa is 0
        anchorList[anchor_id - 1].prevPacket.remoteList[remote_id - 1].tdoa = newTdoa  # sets tracked tdoa to new packet value
        return 0
    prevTdoa = anchorList[anchor_id - 1].prevPacket.remoteList[remote_id - 1].tdoa
    if (prevTdoa / newTdoa) < TDOA_BOUND_LOW or (prevTdoa / newTdoa) > TDOA_BOUND_HIGH:    # outside acceptable bounds
        return 0
    tdoa_avg = (prevTdoa + newTdoa) / 2
    # anchorList[anchor_id - 1].prevPacket.remoteList[remote_id - 1].tdoa = tdoa_avg
    return tdoa_avg


# def tdoaKalmanEstimator():

"""
Hyperbolic Localization infrastructure using TDoA


"""
def localizer(newTdoa, anchor_id, remote_id):
    return True


################################################################
#####                    Main Script                       #####
################################################################
"""
For each packet in the yaml file:
    - Parse data into a tempPacket object (math can be done directly without the object, but it helps visually)
    - Reference with id of remote anchor and respective anchor object
        - If no prior packet exists, save packet to anchor object and continue
        - If prior packet exists but a sequence number was skipped, save packet to anchor object and continue
        - If prior packet exists and sequence number matches up, calculate TDoA between the anchor object and respective remote anchors
"""
for newPacket in read_packets(FILE_PATH):  
    # Create placeholder packet, fill with information from newPacket
    tempPacket             = Packet()   # Placeholder packet for easier transfer to anchor objects
    tempPacket.id          = newPacket.get('from',        'N/A')     # where did this latest packet come from?
    tempPacket.remoteCount = newPacket.get('remoteCount', 'N/A')     # how many remote anchors tracked by sender
    tempPacket.seq         = newPacket.get('seq',         'N/A')     # sequence number of this packet
    tempPacket.rxTimeStamp = newPacket.get('ts',          'N/A')     # timestamp of receipt at tag
    tempPacket.txTimeStamp = newPacket.get('txTimeStamp', 'N/A')     # timestamp of transmission from sender
    # records and sequentially organizes remote anchors
    for remote in newPacket['remoteAnchorData']:
        id = remote.get('id', 'N/A')
        tempPacket.remoteList[id - 1].distance_ticks = remote.get('distance', 'N/A')
        tempPacket.remoteList[id - 1].rxTimeStamp    = remote.get('rxTimeStamp', 'N/A')
        tempPacket.remoteList[id - 1].seq            = remote.get('seq', 'N/A')
    """
    With the placeholder packet built, now check if this packet is the next expected packet of
    the respective anchor. If not, skip TDoA calculations and just record the packet.

    The lps_tdoa3 application cycles through all anchors sequentially, so ideally this statement will initially trigger the same amount of times
    as there are anchors
    """
    if tempPacket.seq is not (anchorList[tempPacket.id - 1].prevPacket.seq + 1):   # checks seq of current packet in respective anchor
        if DEBUG:
            print("Unexpected sequence number, or empty anchor. Recording packet to anchor and reading next packet.")
        anchorList[tempPacket.id - 1].skinny_update_packet(tempPacket)
        continue    # skips all further calculations and moves to next packet
    
    """
    If this packet is the next expected packet for the respective anchor, update the TDoA calculations for this anchor and each remote anchor.
    For each remote anchor, check that the recorded remote data in tempPacket is the next sequence number from
    the remote data of the packet currently contained in the anchor object. If this is NOT the case,
    that particular tdoa calculation is skipped (this includes anchors that haven't yet received a packet)
    """
    # First we calculate clock correction = ratio of differences in timestamps between current and previous packets from anchor
    alpha = (tempPacket.rxTimeStamp - anchorList[tempPacket.id - 1].prevPacket.rxTimeStamp)\
          / (tempPacket.txTimeStamp - anchorList[tempPacket.id - 1].prevPacket.txTimeStamp)    # (P3_rx - P1_rx)/(P3_tx - P1_tx)
    if alpha < 0.999 or alpha > 1.001:     # check for clock correction outliers, these will screw up the tdoa
        anchorList[tempPacket.id - 1].skinny_update_packet(tempPacket)   # adjust to update without changes to tdoa values
        continue

    for remote in tempPacket.remoteList:                                 # cycle through each remote anchor, calculate tdoa where able
        if remote.seq is not anchorList[remote.id - 1].prevPacket.seq:   # check that tempPacket is tracking the same sequence number from P2. Here, these need to be identical
            if DEBUG:
                print(f"Sequence mismatch between P2 and P3. Skipping anchor {remote.id}")
            continue                    # skips this anchor, moves on to next one
        """
        Now, calculate TDoA

        d_rx = rxTimestamp of current packet - rxTimestamp of previous packet from same anchor
        d_tx = P3_tx - P2_tx. These are with respect to two different clocks, so the time of flight between the two anchors
        needs to be considered. This is given in the distance_ticks variable of the remote anchor. So,
        d_tx = P3_tx - (P2_rx - distance_ticks)
        """
        d_rx = tempPacket.rxTimeStamp - anchorList[remote.id - 1].prevPacket.rxTimeStamp
        if d_rx < 0:     # timer has wrapped
            d_rx = d_rx + 1099511627775     # 40-bit unsigned integer max
        
        d_tx = tempPacket.txTimeStamp - (remote.rxTimeStamp - remote.distance_ticks)
        if d_tx < 0:     # timer has wrapped
            d_tx = d_tx + 4294967296        # 32-bit unsigned integer max
        
        tdoa_raw = np.abs(d_rx - (alpha * d_tx))   # The great tdoa calculation
     
        # Now, filter for outliers. For brevity, take ratio of previous tdoa with current tdoa. If > 1, discard tdoa
        # @todo better filtration -- issues with a fast-moving device are foreseen. Kalman filter will be useful
        """
        Now that we have a raw tdoa value, we need to filter this and make sure it's a usable value (no outliers!)
        Two options considered:
            1) Lazy filter -- ratio calculation between raw tdoa and previously-recorded values
            2) Kalman filter (?)
        """
        tdoa_estimated = tdoaLazyEstimator(tempPacket.id, remote.id, tdoa_raw)
        # tdoa_estimated = tdoa_kalmanEstimator()

        """
        Here is where the localization code will go. For now, let's just use this to update the tdoa
        """
        # for debug, delete later vvv
        previousTDOA = anchorList[tempPacket.id - 1].prevPacket.remoteList[remote.id - 1].tdoa

        test = localizer(previousTDOA, tdoa_estimated, ANCHOR1_POS)

        # after localization is done, replace old tdoa value with updated value
        if tdoa_estimated != 0:   # a constant 0 will loop endlessly
            anchorList[tempPacket.id - 1].prevPacket.remoteList[remote.id - 1].tdoa = tdoa_estimated

        """ Debug print out of tdoa distance measurements """
        if((tempPacket.id == 2 and remote.id == 1) or (tempPacket.id == 1 and remote.id == 2)):
            print(f"TDoA:  {tempPacket.id} -> {remote.id}  = {tdoa_estimated * M_TICK} meters")
            print(f"TDoA Raw: {tdoa_raw}  | Previous TDoA: {previousTDOA}  | TDoA Estimated: {tdoa_estimated}")
        # if(tempPacket.id == 2 and remote.id == 1):
        #     print(f"TDoA:  {tempPacket.id} -> {remote.id}  = {tdoa_raw * M_Tick} m")
        # if(tempPacket.id == 1 and remote.id == 2):
        #     print(f"TDoA:  {tempPacket.id} -> {remote.id}  = {tdoa_raw * M_Tick} m")
        



        

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

