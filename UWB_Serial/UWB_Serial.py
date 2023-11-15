## UWB_Serial.py
## 
## Purpose: Read serial input from a TDoA sniffer node, record and parse payload from each respective
##      TDoA anchor node, and output data into a ROS2 message
## Author: Nicholas Kuipers
##
##

###########      Library Imports       #############
import serial           # read serial port
import datetime         # get system time for outgoing TDoA packet to ROS2
import sys              # debugging - reading memory consumption
from time import sleep  # allows for setting rate of TDoA packet gathering

###########      Adjustable Values     #############
Debug    = True         # Set this to true for additional console outputs and data
arr_cols = 10000        # notional size of array storing TDOA objects, can be adjusted as needed
slp_millis = 50         # number of milliseconds between each time a TDoA object is recorded and stored


###########         Classes            #############
class TDOA_Range:
    sys_time       = 0       # system time when sniffer picked up the packet
    source_address = 0       # address of packet origin
    dest_address   = 0       # address of packet destination
    tx_timestamp   = 0       # local timestamp of transmission at source address
    payload        = 0       # payload of range packet

###########      Operational Start     #############
# Set up serial port and assign to 'ser'
ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/ttyACM0'    # @todo assign this dynamically
try:
    ser.open()
except:
    print("Serial Assignment Failed!")
    exit()
else:
    print("Serial Assignment Successful.")
    print(ser)   # verify serial port settings and activation   

# REMOVE LATER
newTDOA = TDOA_Range()

# Main loop - skips over certain amount of messages before recording valid TDoA
while True:
    # newTDOA = TDOA_Range()
    currentLine = ser.readline()
    newLine = currentLine.split()
    if len(newLine) < 6:
        continue                  # discards all bad packets
    
    # parse the line into the new TDOA_Range object
    newTDOA.sys_time       = datetime.datetime.now()
    newTDOA.source_address = newLine[1]
    newTDOA.dest_address   = newLine[3]
    newTDOA.payload        = newLine[5]

    # clean up the TX Timestamp
    TX = str(newLine[4], encoding='utf-8')
    TX = TX.replace("@", "")
    TX = TX.replace(":", "")
    newTDOA.tx_timestamp = bytes(TX, 'utf-8')

    # give python a brief nap
    # this sacrifices precision (i.e. not collecting every TDoA) message in exchange for memory savings
    sleep(slp_millis / 1000)

    if Debug:
    # DEBUG print TDOA object
        print(newTDOA.sys_time)
        print(newTDOA.source_address + newTDOA.dest_address)
        print(newTDOA.tx_timestamp)
        print(newTDOA.payload)
        print(sys.getsizeof(newTDOA))       # prints size of object in memory



# close the serial port -- end program
ser.close()


