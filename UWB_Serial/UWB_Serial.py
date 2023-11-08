## UWB_Serial.py
## 
## Purpose: Read serial input from a TDoA sniffer node, record and parse payload from each respective
##      TDoA anchor node, and output data into a ROS2 message
## Author: Nicholas Kuipers
##
##

# Library Imports
import serial

# Debug Variable, set to true when you want to see all outputs
Debug = True


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
# Read a line from 
#while Debug:
#    print(ser.readline())    # need to specify print

# Read lines ad infinitum
# Reads a line from the console and assigns byte data to currentLine
# currentLine is parsed for the RX anchor ID and payload of 




# close the serial port -- end program
ser.close()


