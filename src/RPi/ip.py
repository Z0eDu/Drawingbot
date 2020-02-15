import socket
import fcntl
import struct
import time
import serial
ser = serial.Serial ("/dev/serial0",timeout=1)    # Open named port 
ser.baudrate = 38400                     # Set baud rate to 38400

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
# wait until the pi actually acquires an IP address
time.sleep(20)
# construct message
msg = "I %s\n" % (get_ip_address('wlan0'))
ser.write(msg)
while (ser.readline().strip("\n\r\x00") != msg.strip("\n\r")):
    ser.write(msg)
ser.close()
