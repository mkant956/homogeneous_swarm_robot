#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import sys
import time
from time import sleep
import sys, tty, termios
import math
from mobile_robot.srv import *
import cmath
import RPi.GPIO as GPIO # always needed with RPi.GPIO


class MotionServer():
	def __init__(self):
		rospy.init_node('oblu_server_bane', anonymous = True)
		#-----------------------------------PID CONTROL--------------------------------------------
		self.k_p = 0.8 				#coefficient of proportional term in pid controller
		self.k_d = 0.5				#coefficient of derivative term
		self.k_i = 0.0				#coefficient of integral term
		self.freq = 10  			#frequency = (1/dt)
		self.avg_pwm = 90			#average pwm of motors = (l_pwm + r_pwm)/2
		self.diff_pwm = 0			#difference in pwm of motors = (l_pwm - r_pwm)
		self.diff_bearing = 0			#difference in current bearing and required bearing = (cur_bear - target_bear)
		self.prev_diff_bearing = 0		#value of diff_bearing in the previous iteration
		self.sum_diff_bearing = 0		#cumulative sum of diff_bearing*(1/freq), represents integral
		self.pid_control = 0			#the pid control expression value
		
		self.turn_power = 50
		self.pwm_turn_l_l = self.turn_power
		self.pwm_turn_l_r = self.turn_power
		self.pwm_turn_r_l = self.turn_power
		self.pwm_turn_r_r = self.turn_power
		#...................................................................................................
		self.flag = 0				#To ensure that target coordinates and nature of motion primitive are decided only once
		self.execute = 0			#To check if motion primitive has been executed
		self.isRotate = True			#True if motion primitive is Rotation CW or CCW, False otherwise
		self.isTranslate = False		#True if motion primitive is Forward Translation, False otherwise
		self.rotateSubscribe = 0		#Subscriber for execution of motion primitive
		self.xt  = 0				#Target x coordinate
		self.yt = 0				#Target y coordinate
		self.n = 0				#Target orientation in 0,1,2, and 3
		self.xc = 0
		self.yc = 0
		self.loc_timestamp = -1
		self.cur_bear = 0
		self.targets = [0,90,180,-90]
	
		self.buf_len = 10
		self.xc_buffer = [0]*self.buf_len
		self.yc_buffer = [0]*self.buf_len
		self.cur_bear_buffer = [0]*self.buf_len		

		self.front_dist=-100
		#---------------------------------------------------------------------------------------------------

		GPIO.setmode(GPIO.BCM)  # choose BCM or BOARD numbering schemes. I use BCM
		GPIO.setwarnings(False)
		self.moto1l=18
		self.moto1r=23
		self.moto2l=24
		self.moto2r=25
		self.power=1.2
		self.pwml=100.0
		self.factor=(299.0/325.0)*(653.0/670.0) #full battery
		self.pwmr=self.pwml*(self.factor**self.power)

		GPIO.setup(self.moto1r, GPIO.OUT)# set GPIO 23 as output for white led
		GPIO.setup(self.moto1l, GPIO.OUT)# set GPIO 18 as output for red led

		GPIO.setup(self.moto2r, GPIO.OUT)# set GPIO 25 as output for white led
		GPIO.setup(self.moto2l, GPIO.OUT)# set GPIO 24 as output for red led

		self.motor1l = GPIO.PWM(18, 55)
		self.motor1r = GPIO.PWM(23, 55)
		self.motor2l = GPIO.PWM(24, 55)
		self.motor2r = GPIO.PWM(25, 55) 

		self.motor1l.start(100)
		self.motor1r.start(100)
		self.motor2l.start(100)
		self.motor2r.start(100)
		
	def forward(self, tmp_pwml,tmp_pwmr):
    		#print "forward"
    		self.motor1l.ChangeDutyCycle(tmp_pwmr)
    		self.motor1r.ChangeDutyCycle(0)
    		self.motor2l.ChangeDutyCycle(tmp_pwml)
    		self.motor2r.ChangeDutyCycle(0)

	def backward(self):
    		#print "backward"
    		self.motor1l.ChangeDutyCycle(0)
    		self.motor1r.ChangeDutyCycle(self.pwmr)
    		self.motor2l.ChangeDutyCycle(0)
    		self.motor2r.ChangeDutyCycle(self.pwml)

	def left(self, tmp_pwml,tmp_pwmr):
    		#print "left"
    		self.motor1l.ChangeDutyCycle(tmp_pwmr)
    		self.motor1r.ChangeDutyCycle(0)
    		self.motor2l.ChangeDutyCycle(0)
    		self.motor2r.ChangeDutyCycle(tmp_pwml)

	def right(self, tmp_pwml, tmp_pwmr):
    		#print "right"
    		self.motor1l.ChangeDutyCycle(0)
    		self.motor1r.ChangeDutyCycle(tmp_pwmr)
    		self.motor2l.ChangeDutyCycle(tmp_pwml)
    		self.motor2r.ChangeDutyCycle(0)

	def stop(self):
    		#print "stop"
    		self.motor1l.ChangeDutyCycle(100)
    		self.motor1r.ChangeDutyCycle(100)
    		self.motor2l.ChangeDutyCycle(100)
    		self.motor2r.ChangeDutyCycle(100)
    		
    	def commandExecutioner(self):
    		start = time.time()
		rospy.wait_for_service('oblu_distance_service_bane')
    		try:
        		execute_command_client = rospy.ServiceProxy('oblu_distance_service_bane', ObluDistanceService)
			rospy.loginfo("received coordinates: ")
        		resp1 = execute_command_client(1)
        		#print "ok"
        		#print(resp1)
			#tt = time.time() - start
        		return resp1
    		except rospy.ServiceException, e:
        		print "Service call failed: %s"%e
    		
    	#def getDistance(self):
    		
		
	def moveRobot(self, req):
		self.xt=req.x
		self.yt=req.y
		#self.loc_timestamp = time.time()
		self.n=req.n
		
		rospy.loginfo("going to execute ")
		coords = self.commandExecutioner()
		(self.xc, self.yc, self.loc_timestamp) = (coords.x, coords.y, coords.time)
		print (coords)
		#(self.xc, self.yc, self.loc_timestamp) = (coords[0], coords[1], coords[2])
		while(True):
			print time.time(),self.loc_timestamp
			if(self.yc > 1 or self.xc >0.8 or self.xc < -0.8 or self.yc < -1):
				self.stop()
				break
			if(time.time() - self.loc_timestamp > 2):
				self.stop()
				time.sleep(1)
				coords = self.commandExecutioner()
				print (coords)
				(self.xc, self.yc, self.loc_timestamp) = (coords.x, coords.y, coords.time)
				rospy.loginfo("location coordinates are : " + str(self.xc) + "	" + str(self.yc))
				
			else:
				self.forward(self.avg_pwm, self.avg_pwm)
				time.sleep(0.5)
				
		return ExecuteMotionPrimitiveResponse(1)
		
	def execute_command_server(self):
    		s = rospy.Service('execute_command_bane', ExecuteMotionPrimitive, self.moveRobot)
    		rospy.spin()
    		print "Ready to execute plans."
    		self.flag=0
		

#if __name__ == "__main__":
  #  execute_command_server()
if __name__ == '__main__':
    
	motionserver = MotionServer()
	motionserver.stop()
    
	try:
		motionserver.execute_command_server()
	except rospy.ROSInterruptException:
		pass
