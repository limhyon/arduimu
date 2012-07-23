#!/usr/bin/env python
import serial
import struct
import math

class arduimuDrv:

	NO_DATA_PACKET       = 0x00

	def __init__(self, port, cb=None):
		self.ser = serial.Serial(port, 
                                    baudrate=38400, 
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
		    if (self.ser.read()=='s' and
		        self.ser.read()=='n' and
		        self.ser.read()=='p'):
		        return 1 
		    return 0 

	def readPacket(self):
		if (not self.syncToHeader()):
           		return arduimuDrv.NO_DATA_PACKET

		packet = []

		# check 0x06 (imu) or 0x13 (gps) or 0x11 (performance)
		pkt_kind = self.chToByte(self.ser.read())

		print pkt_kind

		print "readPacket"

		return packet;

	def decodePacket(self, pkt):
		print "decodePacket"

	def chToByte(self, ch):
        	# convert single char string to unsigned byte
        	return struct.unpack("B", ch)[0]

if __name__ == '__main__':

	def data_cb(data): print "rx pkt"

	arduimu = arduimuDrv('/dev/ttyUSB0', data_cb) #3 = COM4 in windows, use '/dev/ttyUSB' style in unix

	while(True):
        	arduimu.update()
