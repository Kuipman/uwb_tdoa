## UWB_Serial.py
## 
## Purpose: Read serial input from a TDoA sniffer node, record and parse payload from each respective
##      TDoA anchor node, and output data into a ROS2 message
## Author: Nicholas Kuipers
##
##

###########      Library Imports       #############
import serial

###########      Global Variables      #############
Debug = True    # Set this to true for additional console outputs and data


###########         Code Start         #############
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

# DEBUG LINE
# Read lines from the serial port, print directly to console
while Debug:
    print(ser.readline())    # need to specify print



# close the serial port -- end program
ser.close()


