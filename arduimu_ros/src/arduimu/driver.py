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
		if mid == 0x04:
			#print "ROS Data"
			hexdata = ''.join([chr(val) for val in pkt[3:7]])
			self.q0 = self.decode_float(hexdata)
			hexdata = ''.join([chr(val) for val in pkt[7:11]])
			self.q1 = self.decode_float(hexdata)
			hexdata = ''.join([chr(val) for val in pkt[11:15]])
			self.q2 = self.decode_float(hexdata)
			hexdata = ''.join([chr(val) for val in pkt[15:19]])
			self.q3 = self.decode_float(hexdata)
			hexdata = ''.join([chr(val) for val in pkt[19:23]])
			self.ang_vel_x = self.decode_float(hexdata)
			hexdata = ''.join([chr(val) for val in pkt[23:27]])
			self.ang_vel_y = self.decode_float(hexdata)
			hexdata = ''.join([chr(val) for val in pkt[27:31]])
			self.ang_vel_z = self.decode_float(hexdata)
			hexdata = ''.join([chr(val) for val in pkt[31:35]])
			self.lin_acc_x = self.decode_float(hexdata)
			hexdata = ''.join([chr(val) for val in pkt[35:39]])
			self.lin_acc_y = self.decode_float(hexdata)
			hexdata = ''.join([chr(val) for val in pkt[39:43]])
			self.lin_acc_z = self.decode_float(hexdata)
			#print "[Bus:%d] roll %2.2f pitch %2.2f yaw %2.2f"%(bid,self.roll,self.pitch,self.yaw)
			#print "%2.2f %2.2f %2.2f"%(self.ang_vel_x,self.ang_vel_y,self.ang_vel_z)
			



		self.quaternion = []
		self.quaternion.append(self.q0)
		self.quaternion.append(self.q1)
		self.quaternion.append(self.q2)
		self.quaternion.append(self.q3)

		results = {}
		results['DATA_QUATERNION'] = self.quaternion
		results['DATA_ANGULAR_VEL'] = [self.ang_vel_x, 
                                               self.ang_vel_y, 
                                               self.ang_vel_z]
		results['DATA_LINEAR_ACCEL'] = [self.lin_acc_x, 
                                                self.lin_acc_y, 
                                                self.lin_acc_z]

		if (self.data_callback is not None):
                	self.data_callback(results)

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

	def data_cb(data):
		q = [data['DATA_QUATERNION'][0],
		     data['DATA_QUATERNION'][1],
		     data['DATA_QUATERNION'][2],
		     data['DATA_QUATERNION'][3]]
		print q

	arduimu = arduimuDrv('/dev/ttyUSB1', data_cb) #3 = COM4 in windows, use '/dev/ttyUSB' style in unix

	while(True):
        	arduimu.update()
