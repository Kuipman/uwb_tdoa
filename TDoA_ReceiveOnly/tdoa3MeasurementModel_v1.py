"""
tdoa3MeasurementModel_v1.py

Last Updated: 20240604

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
import math
# import pymap3d as pm
from collections import deque     # for moving window
from scipy.optimize import least_squares   # optimization
import matplotlib.pyplot as plt            # plotting (optional)
from mpl_toolkits.mplot3d import Axes3D    # more plotting
# For ROS2:
# import rclpy    # don't forget to source your ROS2 configuration, else this will return an error
# from rclpy.node import Node

#####   Global Constants (TUNING VALUES)   #####
# FILE_PATH = "/home/nick_k/Documents/UWB/uwb_tdoa/TDOA_Sniffer/output20240415.yaml"
FILE_PATH = "/home/nick_k/Documents/UWB/uwb_tdoa/TDOA_Sniffer/finalTest_static.yaml"
INITIAL_GUESS = [2, 3, 2]
BASE_STATION_POS = np.array([
    [0, 0, 0.9144],
    [0, 4.5, 0.9144],
    [4.5, 0, 1.2192],
    [4.5, 4.5, 1.524]
])
MOVING_WINDOW_SIZE = 10
MINIMUM_PACKET_ITERATIONS = 50   # number of complete cycles before localization starts
MINIMUM_LOCALE_ITERATIONS = 10   # after localization starts, minimum number of complete cycles between each localization function call
TDOA_BOUND_LOW  = 0.5
TDOA_BOUND_HIGH = 2
DEBUG = True                  # set to True for debug printouts to console

#####   Static Global Constants (Do not tune)   #####
M_TICK = 0.004691764
SPEED_OF_LIGHT = 299792458.0

################################################################
#####                 Global Classes                       #####
################################################################
"""
MovingWindow -- Acts as an array of preset size. Once array is full of
objects and a new object is received, the old object is removed and the new
object is added (think of a visual window moving to the right).

Also includes functions for calculating the average and median of the
tracked values. Used to track the latest TDoA values for each base station pair
"""
class MovingWindow:
    def __init__(self, window_size):
        self.window = deque(maxlen=window_size)
        self.window_size = window_size
        self.weighted_average = None  # Initialize weighted average

    def add_value(self, value):
        self.window.append(value)
        self.update_weighted_average(value)  # Update weighted average

    def get_average(self):
        return np.mean(self.window) if len(self.window) > 0 else float('nan')

    def get_median(self):
        if len(self.window) > 0:
            return np.median(self.window)
        else:
            return float('nan')

    def get_values(self):
        return list(self.window)

    def update_weighted_average(self, new_value, alpha=0.2):
        # Update the weighted average with the new value
        if self.weighted_average is None:
            self.weighted_average = new_value
        else:
            self.weighted_average = (1 - alpha) * self.weighted_average + alpha * new_value

    def get_weighted_average(self):
        return self.weighted_average if self.weighted_average is not None else float('nan')
    
"""
Tracks remote anchor information received as part of an incoming packet
    - Used as part of the Packet object
"""
class RemoteData:
    def __init__(self, id = 0):
        self.id             = id
        self.distance_ticks = 0
        self.tdoa           = 0         # tdoa between anchor and remote anchor wrt tag
        self.rxTimeStamp    = 0
        self.seq            = 0

"""
Tracks all information from an incoming packet, including remote anchor information
"""
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

"""
Anchor object tracks static information of the given anchor (id, coordinates), as
well as the previously-received packet from that anchor
"""

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

"""
Tracks a given anchor pair's MovingWindow of TDoA values, and keeps track of the
latest TDoA estimate based on the values in the window
"""
class TDOA:
    def __init__(self, anchor1, anchor2):
        self.anchor1  = anchor1
        self.anchor2  = anchor2
        self.mWindow   = MovingWindow(MOVING_WINDOW_SIZE)
        self.estimate = 0
        self.mean     = 0     # for ending graphs/comparisons
        self.std      = 0     # for ending graphs
        self.count    = 0     # iterative

    def addNewValue(self, newValue):
        self.count += 1
        
        # calculate new mean
        delta = newValue - self.mean
        self.mean += delta / self.count

        # calculate new sum of squared differences
        sum_squared_diff += delta * (newValue - self.mean)

        return self.mean, self.get_std(sum_squared_diff)
    
    def get_variance(self, sum_squared_diff):
        if self.count < 2:
            return float('nan')   # Need more data to compute variance
        return self.sum_squared_diff / (self.count - 1)
    
    def get_std(self, sum_squared_diff):
        return math.sqrt(self.get_variance(sum_squared_diff))


################################################################
#####                 Global Variables                     #####
################################################################

anchorList = [Anchor(BASE_STATION_POS[0], 1), Anchor(BASE_STATION_POS[1], 2), Anchor(BASE_STATION_POS[2], 3), Anchor(BASE_STATION_POS[3], 4)]
estimatedPosition = INITIAL_GUESS     # estimated xyz coordinate of tag
tdoa_roster = [TDOA(1, 2), TDOA(1, 3), TDOA(1, 4), TDOA(2, 3), TDOA(2, 4), TDOA(3, 4)]  # tdoa_12, tdoa_13, tdoa_14, tdoa_23, tdoa_24, tdoa_34

################################################################
#####                 Global Functions                     #####
################################################################

# # Allows for reading an individual packet from the yaml file
def read_packets(file_path):
    with open(file_path, 'r') as file:
        for data in yaml.safe_load_all(file):
            yield data

"""
updateTdoaValues(anchor_Id, remote_id, newTdoa)
Purpose: Adds calculated raw tdoa value to the respective moving window (filtering for outliers) Calculate the median of the window as the new tdoa estimate
Note: A positive TDoA indicates the tag is closer to the latter base station (i.e. tdoa_12 = 1.02, closer to base station 2)
"""

def updateTdoaValues(anchor_id, remote_id, newTdoa):
    if remote_id == anchor_id:  # id outlier filtering
        return None
    if remote_id < anchor_id:  # Flip these if needed
        tmp = anchor_id
        anchor_id = remote_id
        remote_id = tmp
        newTdoa *= -1       # if ids are flipped, tdoa value also needs to be flipped

    # iterate through list until correct TDOA object found
    counter = 0
    for element in tdoa_roster:
        if element.anchor1 == anchor_id and element.anchor2 == remote_id:
            break
        counter += 1
    else:
        print("No matching TDOA object found.")
        return

    # set temporary value as pointer to the moving window
    window = tdoa_roster[counter].mWindow

    # Calculate IQR for outlier detection
    window_values = window.get_values()
    # The dumb way that works
    # if len(window_values) == MOVING_WINDOW_SIZE:
    #     avg = window.get_weighted_average()
    #     if newTdoa > 0:
    #         lower_bound = avg - (0.5 * avg)
    #         upper_bound = avg * 2
    #     else:
    #         lower_bound = avg * 2
    #         upper_bound = avg - (0.5 * avg)
    if len(window_values) == 5:
        q1 = np.percentile(window_values, 5)
        q3 = np.percentile(window_values, 95)
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr

        if newTdoa < lower_bound or newTdoa > upper_bound:
            if DEBUG:
                print(f"Outlier detected: {newTdoa} is outside the range [{lower_bound}, {upper_bound}]")
            return  # Do not add the outlier value to the window

    # Add new TDoA data to moving window and update weighted average
    window.add_value(newTdoa)

    # update mean and other values in TDOA object
    tdoa_obj = tdoa_roster[counter].addNewValue(newTdoa)


    if len(window_values) == MOVING_WINDOW_SIZE:
        tdoa_roster[counter].estimate = window.get_median()
        return tdoa_roster[counter].estimate
    

"""
Hyperbolic Localization infrastructure using TDoA

The estimated tdoa value for each base station pair and the previously-estimated location are
parameters for this function.

This function states a series of six hyperboloids based on the base station pairs, then estimates
their intersection at which the tag is most likely to be positioned.

Returns the new location estimate
"""
def localizer(estimatedPosition, tdoa_12, tdoa_13, tdoa_14, tdoa_23, tdoa_24, tdoa_34):
    """
    Objective function is needed for least-squares optimization

    Calculates euclidean distance between last estimated position and base station positions. Subtracts
    the calculated tdoa value to determine how close new value is to estimated position
    """
    # def objective_function(position):
    #     differences = []

    #     for tdoa in tdoa_roster:
    #         i = tdoa.anchor1 - 1  # Adjust for 0-based indexing
    #         j = tdoa.anchor2 - 1  # Adjust for 0-based indexing
    #         estimate = tdoa.estimate
    #         dist_i = np.linalg.norm(position - BASE_STATION_POS[i])
    #         dist_j = np.linalg.norm(position - BASE_STATION_POS[j])
    #         differences.append((dist_i - dist_j) - estimate)
    
    #     return differences

    threshold = 0.01  # Example threshold, adjust as necessary
    weight = 2  # Example weight, adjust as necessary
    def objective_function(position):
        differences = []
        for tdoa in tdoa_roster:
            i = tdoa.anchor1 - 1
            j = tdoa.anchor2 - 1
            estimate = tdoa.estimate
            dist_i = np.linalg.norm(position - BASE_STATION_POS[i])
            dist_j = np.linalg.norm(position - BASE_STATION_POS[j])
            difference = (dist_i - dist_j) - estimate
            # Apply a higher weight to the z-axis if needed
            if np.abs(difference) > threshold:  # define a suitable threshold
                difference *= weight  # adjust weight as necessary
            differences.append(difference)
        return differences

    
    # Least squares calculation
    result = least_squares(objective_function, estimatedPosition)

    optimized_position = result.x
    # print("Optimized Position:", optimized_position)
    return optimized_position


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

initializationCounter = 0   # iterates with each complete cycle until MINIMUM_ITERATIONS is reached
localeCounter         = 0

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
    if (tempPacket.seq is not (anchorList[tempPacket.id - 1].prevPacket.seq + 1) or (tempPacket.seq == 0 and (anchorList[tempPacket.id - 1].prevPacket.seq + 1) != 127)):   # checks seq of current packet in respective anchor
        # if DEBUG:
        #     print("Unexpected sequence number, or empty anchor. Recording packet to anchor and reading next packet.")
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
            # if DEBUG:
            #     print(f"Sequence mismatch between P2 and P3. Skipping anchor {remote.id}")
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
        
        tdoa_raw = d_rx - (alpha * d_tx)   # The great tdoa calculation
        """
        We need to ensure this new tdoa value isn't an outlier (i.e. it's actually useful)
        
        updateTdoaValues() function cross-checks with previously-tracked tdoa values and decides whether or not to update
        """
        tdoa_estimated = updateTdoaValues(tempPacket.id, remote.id, tdoa_raw * M_TICK)
        # tdoa_estimated = updateTdoaValues_TEMP(tempPacket.id, remote.id, tdoa_raw * M_TICK)

        """
        Debug printout: TDoA values
        """
        if DEBUG and (tempPacket.id != remote.id):
            if tdoa_estimated is None:
                # test = True
                print(f"TDoA Raw:  {tempPacket.id} -> {remote.id}  = {tdoa_raw * M_TICK} meters")
            else:
                if((tempPacket.id == 1) and (remote.id == 2)):    #  or ((tempPacket.id == 2) and (remote.id == 1)
                    if ((tdoa_estimated < 0) and (tdoa_raw > 0)) or ((tdoa_estimated > 0) and (tdoa_raw < 0)):
                        print(f"TDoA Estimated:  {tempPacket.id} -> {remote.id}  = {-tdoa_estimated} meters")
                    else:
                        print(f"TDoA Estimated:  {tempPacket.id} -> {remote.id}  = {tdoa_estimated} meters")
        # previousTDOA = anchorList[tempPacket.id - 1].prevPacket.remoteList[remote.id - 1].tdoa
        # if((tempPacket.id == 2 and remote.id == 1) or (tempPacket.id == 1 and remote.id == 2)):
        #     print(f"TDoA:  {tempPacket.id} -> {remote.id}  = {tdoa_estimated * M_TICK} meters")
            # print(f"TDoA Raw: {tdoa_raw}  | Previous TDoA: {previousTDOA}  | TDoA Estimated: {tdoa_estimated}")

    """
    Once all remote anchors have been cycled through and TDoAs calculated, check if the first MINIMUM_ITERATIONS
    number of packets has been processed (this allows for all needed data to be populated and condensed for localization).
    If MINIMUM_ITERATIONS has not yet been reached, increment the counter and carry on.
    If MINIMUM_ITERATIONS has been reached, run the localizer to update the estimated location
    """
    if initializationCounter < MINIMUM_PACKET_ITERATIONS:
        initializationCounter += 1
    else:
        localeCounter += 1
        if localeCounter > MINIMUM_LOCALE_ITERATIONS:
            localeCounter = 0
            localizer(estimatedPosition, tdoa_roster[0].estimate, tdoa_roster[1].estimate, tdoa_roster[2].estimate, 
                                     tdoa_roster[3].estimate, tdoa_roster[4].estimate, tdoa_roster[5].estimate)

"""
Here, we reached the end of the yaml file.

Produce results for TDoA Noise measurements (standard deviation and variance).

Produce results for Location estimates x, y, z (standard deviation and variance).
"""






###########################
##### END OF DOCUMENT #####
###########################


"""
# Going pure averaging to try something here
"""
# def updateTdoaValues_TEMP(anchor_id, remote_id, newTdoa):
#     if remote_id == anchor_id:  # id outlier filtering
#         return None
#     if remote_id < anchor_id:  # Flip these if needed
#         tmp = anchor_id
#         anchor_id = remote_id
#         remote_id = tmp
#         newTdoa *= -1       # if ids are flipped, tdoa value also needs to be flipped

#     # iterate through list until correct TDOA object found
#     counter = 0
#     for element in tdoa_roster:
#         if element.anchor1 == anchor_id and element.anchor2 == remote_id:
#             break
#         counter += 1
#     else:
#         print("No matching TDOA object found.")
#         return

#     # set temporary value as pointer to the moving window
#     window = tdoa_roster[counter].mWindow

#     # Calculate IQR for outlier detection
#     window_values = window.get_values()

#     # Get weighted average and add to the window
#     if len(window_values) == MOVING_WINDOW_SIZE:
#         avg = window.get_weighted_average()

#     # Add new TDoA data to moving window and update weighted average
#     window.add_value(newTdoa)

#     if len(window_values) == MOVING_WINDOW_SIZE:
#         tdoa_roster[counter].estimate = window.get_median()
#         return tdoa_roster[counter].estimate