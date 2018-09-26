#!/usr/bin/python
#
# A sample application to read bluetooth data over serial port
# Usage: read_serial device_name baudrate outfile numbytes
# (e.g. python read_serial.py /dev/ttyACM0 115200 test.txt)
import rospy
import roslib;roslib.load_manifest('mobile_robot')
from std_msgs.msg import Float32
from std_msgs.msg import String
from mobile_robot.srv import ObluDistanceService,ObluDistanceServiceResponse
# from oblu.msg import Coordinates

import threading

import sys
import serial
import struct
import math

import numpy as np
import time

MAX_CMD_LENGTH = 6
btserial=''

x_coord=0.0
y_coord=0.0



def open_device(device, rate):
	print "Opening Serial device", device
	btserial = serial.Serial(device,rate)
	print btserial
	return btserial

def read_device(device,length):
	buffer = []
	device.flushInput()
	buffer = device.read(length)
	return buffer

def write_device(device, buffer):
	if(len(buffer) < MAX_CMD_LENGTH):
		#for row in buffer:
		x=bytearray(buffer)
		# print(type(buffer))
		# print(buffer)
		device.write(x)
		device.flushOutput()
	else:
		print "Error: Was trying to write too long command"

def parse_pkt(buffer):
	pkt_info = ()
	payload = []
        #print buffer
	(start_code, pkt_num1, pkt_num2, payload_length) = struct.unpack("!BBBB",buffer[0:4])

	if (start_code != 0xAA):
		print "Error: Failed to detect header at packet start"
		valid = False

	else :
		valid = True
		(step_count, checksum) = struct.unpack("!HH",buffer[60:64])
		pkt_info = (pkt_num1, pkt_num2, step_count, checksum)
		payload = struct.unpack("!ffffffffffffff",buffer[4:60])
	return valid, pkt_info, payload

def create_ack(pkt_num1, pkt_num2):
	ack = []
	ack.append(1)
	ack.append(pkt_num1)
	ack.append(pkt_num2)
	ack.append((1 + pkt_num1 + pkt_num2) / 256)
	ack.append((1 + pkt_num1 + pkt_num2) % 256)
	return ack

def calc_disp(sensor_data, theta):
	d0 = sensor_data[0]
	d1 = sensor_data[1]
	d2 = sensor_data[2]
	d3 = sensor_data[3]

	sin_theta = math.sin(theta)
	cos_theta = math.cos(theta)

	dx = d0 * cos_theta - d1 * sin_theta
	dy = d0 * sin_theta + d1 * cos_theta
	dz = d2
	dp = d3

	disp = math.sqrt(dx * dx + dy * dy + dz * dz)
	return dx,dy,dz,dp,disp

def calc_angle(x,y):
	if (y !=0) and (x != 0 ):
		angle = math.atan(x/y)
		angle = math.degrees(angle)
	else:
		if (x ==0): angle = 0
		else: angle = 90
	return angle

def calc_dist(x,y,z):
	r = x*x + y*y + z*z
	return math.sqrt(r)


#if len(sys.argv) < 1 :
#	print "Usage: read_serial.py serial_dev_name baudrate outfile No_of_packet"
#else :
def oblu_dist():
	device = "/dev/ttyACM0"
	baudrate = 115200
	outfile = "test.txt"
#	numbytes = 6400
#	print device, baudrate, outfile, numbytes
	data = []

	btserial = open_device(device,baudrate)

	#write command to start dead step reckoning
	cmd = [0x34,0x00,0x34]
	write_device(btserial,cmd)
	count = 0
	pkt_len = 64
	num_pkts = 0
	curr_pkt = 0
	prev_pkt = -1

	xpos = 0.0   # x-coord in user's reference frame
	ypos = 0.0   # y-coord in user's reference frame
	zpos = 0.0   # z-coord in user's reference frame
	phi  = 0.0   # Angular position around Z-axis in user's reference frame
	read_device(btserial, 4)

	x = [0]
	y = [0]

	pkts = 0
	st = 0
	min = 0

	# mfile = open("test.txt","w")
	# mfile.write("X  : Y")
	# mfile = open("test.txt","a")

	#oblu_pub = rospy.Publisher('oblu_bane',Coordinates)
	#rospy.init_node('oblu_bane_publisher',anonymous=True)
	#r= rospy.Rate(
	try:
		while (True):
			#print("going to give distance")
			buffer = read_device(btserial, pkt_len)
			valid, packet_info, payload = parse_pkt(buffer)

			if (valid == False): continue

			curr_pkt = packet_info[0] * 256 + packet_info[1]
			# print "Read packet %d...Sending Ack" % curr_pkt
			ack = create_ack(packet_info[0],packet_info[1])
			write_device(btserial,ack)

			if (curr_pkt == prev_pkt): continue

			dx,dy,dz,dp,disp = calc_disp(payload,phi)

			xpos += dx
			ypos += dy
			zpos += dz
			phi  += dp

			radial_dist = calc_dist(xpos,ypos,0) #for now do not factor in zpos

			num_pkts += 1
			count += pkt_len
			#print count
			prev_pkt = curr_pkt

			data.append(xpos)
			data.append(ypos)
			data.append(zpos)
			data.append(phi)
			data.append(disp)
			x.append(float(format(xpos, '.4f')))
			y.append(float(format(ypos, '.4f')))
			# print(float(format(xpos, '.4f')),float(format(ypos, '.4f')))
			global x_coord
			global y_coord

			x_coord = float(format(xpos, '.4f'))
			y_coord = float(format(ypos, '.4f'))
			rospy.loginfo("location %f %f\n"%(x_coord,y_coord))
	except KeyboardInterrupt:
		exit()
		# ax.plot(np.array(x), np.array(y), 'r')
		# plt.pause(0.0001)
		# plt.draw()
		# mfile.write(str(float(format(xpos, '.4f')))+" "+str(float(format(ypos, '.4f'))))
		# mfile.write("\n")
		# time.sleep(0.001)
		# pkts += 1
	# mfile.close()
# #command to stop processing
# cmd = ['0x32','0x00','0x32']
# write_device(btserial,cmd)

# #command to stop all outputs
# cmd = ['0x22','0x00','0x22']
# write_device(btserial,cmd)
# for i in range(0, num_pkts*5, 5):
# 	out.write(', '.join(str(x) for x in data[i:i+5]))
# 	out.write("\n")
# #out.flush()
# btserial.close()
#print "Written into file"
#out.close()
def giveDistance(req):
	global x_coord
	global y_coord
	resp = ObluDistanceServiceResponse()
	resp.x = x_coord
	resp.y = y_coord
	resp.time = time.time()
	print resp
	return resp
def server():
	global x_coord
	global y_coord
	print x_coord,y_coord
	try:
	    	s = rospy.Service('oblu_distance_service_bane', ObluDistanceService, giveDistance)
    		# print("Ready to give distances")
    		rospy.spin()
	except KeyboardInterrupt:
		exit()

if __name__ == '__main__':
	oblu_calc = threading.Thread(target=oblu_dist, args=())
	service = threading.Thread(target=server, args=())

	rospy.init_node('oblu_distance_server_bane', anonymous = True)

	service.start()
	oblu_calc.start()
