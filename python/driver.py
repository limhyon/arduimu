#!/usr/bin/env python
import serial
import struct
import math
import binascii

class arduimuDrv:

	NO_DATA_PACKET       = 0x00

	def __init__(self, port, cb=None):
		self.ser = serial.Serial(port, 
                                    baudrate=115200, 
                                    bytesize=8, parity='N', 
                                    stopbits=1, timeout=None)
        	self.data_callback = cb

	def __del__(self):
        	self.ser.close()

	def update(self):
       		pkt = self.readPacket()
        	if (pkt != arduimuDrv.NO_DATA_PACKET):       
            		self.decodePacket(pkt)

	def syncToHeader(self):
		while (self.ser.inWaiting() > 0):
		    if (self.ser.read()=='S' and
		        self.ser.read()=='T' and
		        self.ser.read()=='X'):
		        return 1 
		    return 0 

	def readPacket(self):
		if (not self.syncToHeader()):
           		return arduimuDrv.NO_DATA_PACKET

		packet = []

		# Read BID
		bid = self.chToByte(self.ser.read())
		packet.append(bid)
		# Read MID
		mid = self.chToByte(self.ser.read())
		packet.append(mid)
		# Read LEN
		pkt_len = self.chToByte(self.ser.read())
		packet.append(pkt_len)
		# Read Data by LEN
		for i in range(0,pkt_len):
			byte = self.chToByte(self.ser.read())
			packet.append(byte)
		# Read Chksum
		chksum = self.chToByte(self.ser.read())

		# Verify chksum

		return packet;

	def decodePacket(self, pkt):
		# Check BID(BusID)
		bid = pkt[0];
		# Check MID
		mid = pkt[1]
		if mid == 0x01:
			#print "IMU Data"
			roll_raw = ''.join([chr(val) for val in pkt[3:7]])
			roll = 180*self.decode_float(roll_raw)/math.pi
			pitch_raw = ''.join([chr(val) for val in pkt[7:11]])
			pitch = 180*self.decode_float(pitch_raw)/math.pi
			yaw_raw = ''.join([chr(val) for val in pkt[11:15]])
			yaw = 180*self.decode_float(yaw_raw)/math.pi
			print "[Bus:%d] roll %2.2f pitch %2.2f yaw %2.2f"%(bid,roll,pitch,yaw)

	def chToByte(self, ch):
        	# convert single char string to unsigned byte
        	return struct.unpack("B", ch)[0]

	
	def decode_float(self,s):
		"""Other possible implementation. Don't know what's better
		#from ctypes import *
		s = s[6:8] + s[4:6] + s[2:4] + s[0:2] # reverse the byte order
		i = int(s, 16)                   # convert from hex to a Python int
		cp = pointer(c_int(i))           # make this into a c integer
		fp = cast(cp, POINTER(c_float))  # cast the int pointer to a float pointer
		return fp.contents.value         # dereference the pointer, get the float
		"""
		return struct.unpack('<f', s)[0]

if __name__ == '__main__':

	def data_cb(data): print "rx pkt"

	arduimu = arduimuDrv('/dev/ttyUSB0', data_cb) #3 = COM4 in windows, use '/dev/ttyUSB' style in unix

	while(True):
        	arduimu.update()
