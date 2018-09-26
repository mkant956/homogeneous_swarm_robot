#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import paramiko
import sys
import time
from time import sleep
import sys, tty, termios
import math
from mobile_robot.srv import *
import cmath
import RPi.GPIO as GPIO # always needed with RPi.GPIO
import yaml

class MotionServer():
	def __init__(self):
		self.robot_config = yaml.load(open('/home/pi/catkin_ws/src/mobile_robot/scripts/robot_config.yaml'))
		rospy.init_node(self.robot_config['node_name'], anonymous = True)
		rospy.Subscriber(self.robot_config['dist_topic']['front'],Float32,self.Callback_distance)
		#-----------------------------------PID CONTROL--------------------------------------------
		self.k_p = 1.5 				#coefficient of proportional term in pid controller
		self.k_d = 0.5				#coefficient of derivative term
		self.k_i = 0.0				#coefficient of integral term
		self.freq = 10  			#frequency = (1/dt)
		# if rospy.has_param("robot_"+str(self.robot_config['robot_id'])):
		# 	self.avg_pwm = rospy.get_param("robot_0")			#average pwm of motors = (l_pwm + r_pwm)/2
		# else:	
		self.avg_pwm = self.robot_config['pwm']['forward']
		self.diff_pwm = 0			#difference in pwm of motors = (l_pwm - r_pwm)
		self.diff_bearing = 0			#difference in current bearing and required bearing = (cur_bear - target_bear)
		self.prev_diff_bearing = 0		#value of diff_bearing in the previous iteration
		self.sum_diff_bearing = 0		#cumulative sum of diff_bearing*(1/freq), represents integral
		self.pid_control = 0			#the pid control expression value
		
		self.turn_power = self.robot_config['pwm']['rotation']
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
 

   	def Callback_distance(self,data):
		self.front_dist=data.data

	def setTarget(self, tmp_xc, tmp_yc, tmp_xt, tmp_yt):
		var  = (math.atan2((tmp_yt-tmp_yc+0.5),(tmp_xt-tmp_xc + 0.5)))*180.0/3.141592653
		return var

	def distance(self, tmp_xc, tmp_yc, tmp_xt, tmp_yt, n):
		#return abs(tmp_yc-tmp_yt-0.5) +abs(tmp_xc-tmp_xt-0.5)
		return ((n+1)%2)*abs(tmp_xc-tmp_xt-0.5)+(n%2)*abs(tmp_yc-tmp_yt-0.5)

	# Converts degree into 0,1,2, or 3 depending on the degrees
	def computeN(self, cur_pose):
		if(cur_pose<45 and cur_pose>-45):
			return 0
		elif(cur_pose<135 and cur_pose>45):
			return 1
		elif(cur_pose>135 or cur_pose<-135):
			return 2
		elif(cur_pose<-45 and cur_pose>-135):
			return 3


	# Estimate the direction of the rotation: left or right
	def decideDir(self, complex_cur_bear, complex_target_bear):
        	#return true if left
    		if (cmath.phase(complex_target_bear/complex_cur_bear) > 0):
    		    	return True
    		else: 
    			return False

	
	# Chaeck the deviation from target bearings
	def hasDeviated(self, complex_cur_bear, complex_target_bear):
		rot = abs(complex_cur_bear-complex_target_bear)>10*math.pi/180
		return rot



	# Execute for completing a forward or rotate command
	def callback(self, data):
		read = data.data
		s1=read.find(' ')
		self.xc = float(read[:s1])
		read=read[s1+1:]
		s2=read.find(' ')
		self.yc = float(read[:s2])
		read = read[s2+1:]
		self.cur_bear=float(read)
	
		self.xc_buffer.insert(0,self.xc)
		self.yc_buffer.insert(0,self.yc)
		self.cur_bear_buffer.insert(0,self.cur_bear)

		self.xc_buffer.pop()
		self.yc_buffer.pop()
		self.cur_bear_buffer.pop()
	
		xc_sortedbuffer = (self.xc_buffer[:])
		xc_sortedbuffer.sort()
		yc_sortedbuffer = (self.yc_buffer[:])
		yc_sortedbuffer.sort()
		cur_bear_sortedbuffer = (self.cur_bear_buffer[:])
		cur_bear_sortedbuffer.sort()
	
        	if (self.cur_bear_buffer[len(self.cur_bear_buffer)-1] <> 0):
        	    self.xc = sum(xc_sortedbuffer)/float(len(xc_sortedbuffer))
        	    self.yc = sum(yc_sortedbuffer)/float(len(yc_sortedbuffer))
        	    self.cur_bear = sum(cur_bear_sortedbuffer)/float(len(cur_bear_sortedbuffer))

#        	rospy.loginfo(self.xc_buffer)
#       	rospy.loginfo("pose : " + str(self.xc) + " " + str(self.yc) + " " + str(self.cur_bear))
	
		target  = self.setTarget(self.xc, self.yc, self.xt, self.yt)

        	complex_cur_bear = cmath.exp(self.cur_bear*math.pi*1j/180)
        	complex_target_bear = cmath.exp(self.targets[self.n]*math.pi*1j/180)
	
		
		if(self.flag==0):
			self.flag=1
			
			if(abs(cmath.phase(complex_cur_bear/complex_target_bear) )< math.pi/4):
				self.isRotate=False
			else: 
				self.isRotate=True
			if(int(self.xc)==int(self.xt) and int(self.yc)==int(self.yt)):
				self.isTranslate=False
			elif(self.isRotate == False):
				self.isTranslate=True
	
		#if((abs(cur_bear-targets[n])<5 or abs(abs(cur_bear-targets[n])-360 )<5) and distance(xc,yc,xt,yt)<=0.05):
		if(self.isRotate == False and self.isTranslate == False):
		        self.stop()
        	        print("subscriber unresgistered 1111111111111\n")
			self.rotateSubscribe.unregister()
			self.flag = 0
        	        self.execute = 1
		#elif(abs(cur_bear-targets[n])%360 >= 3):# and distance(xc,yc,xt,yt)<=0.05):
		elif(self.isRotate):
			#rospy.loginfo("rotating : c_a =  %lf n = %d\n",self.cur_bear, self.n)
			complex_target_bear = cmath.exp(self.targets[self.n]*math.pi*1j/180.0)
		

		#target=targets[n]
               		if(self.decideDir(complex_cur_bear,complex_target_bear)):
                	       	self.left(self.pwm_turn_l_l, self.pwm_turn_l_r)
                	else:
                	        self.right(self.pwm_turn_r_l, self.pwm_turn_r_r)

			if(abs(complex_cur_bear - complex_target_bear) < 5*math.pi/180):
	
			   	self.stop()
	        	        print("subscriber unresgistered 222222222222222\n")
	               	        self.rotateSubscribe.unregister()
        			self.flag = 0
			        self.execute = 1
                	        #stop()

		elif(self.isTranslate):
			target  = self.setTarget(self.xc, self.yc, self.xt, self.yt)
			complex_target_bear = cmath.exp(target*math.pi*1j/180.0)
			#if(abs(cur_bear-target)>9 and abs(abs(cur_bear-target)-360 )>9  and distance(xc,yc,xt,yt)>0.05):
			if(self.distance(self.xc,self.yc,self.xt,self.yt,self.n)>0.10):
				#print("going forward(straight) : ",cur_bear,target,xc,yc,xt,yt)
	
				self.prev_diff_bearing = self.diff_bearing
				self.diff_bearing=cmath.phase(complex_target_bear/complex_cur_bear)*180/math.pi
			
				d_diff_bearing = (self.diff_bearing-self.prev_diff_bearing)*self.freq
				self.sum_diff_bearing = self.sum_diff_bearing + self.diff_bearing/self.freq
				
				self.pid_control = self.k_p * self.diff_bearing + self.k_i * self.sum_diff_bearing + self.k_d * d_diff_bearing
				self.diff_pwm = (-1.0)*self.pid_control
				
				l_pwm = max(0,min(self.avg_pwm + self.diff_pwm/2,100))
				r_pwm = max(0,min(self.avg_pwm- self.diff_pwm/2,100))
				if(self.front_dist>5):
					self.forward(l_pwm,r_pwm)				
				else:
					self.stop()
			else:
				self.stop()
				self.isRotate = True

		else:
                	self.stop()
                	print("subscriber unresgistered 33333333333333333333333\n")
                	self.rotateSubscribe.unregister()
			self.flag = 0
                	self.execute = 1
	
	# Move a robot according to the request 
	def moveRobot(self,req):
		self.xt=req.x
		self.yt=req.y
		self.n=req.n
    		print("target : ",self.xt, self.yt, self.n, self.execute)
		self.rotateSubscribe = rospy.Subscriber(self.robot_config['position_topic'], String, self.callback)
		while(self.execute==0):
			#rospy.loginfo("skipped")
			time.sleep(0.1)
		if(int(self.xc)<>int(self.xt) or int(self.yc)<>int(self.yt) or self.computeN(self.cur_bear)<>self.n):
			rospy.loginfo("skipped corrected")
			self.execute=0
			self.flag=0
			self.rotateSubscribe = rospy.Subscriber(self.robot_config['position_topic'], String, self.callback)
		self.execute = 0
		self.rotateSubscribe.unregister()
		self.flag = 0
		print("goForward executed")
		return ExecuteMotionPrimitiveResponse(1)    

	# Wait for command(move/rotate) request
	def execute_command_server(self):
    		s = rospy.Service(self.robot_config['command_service_name'], ExecuteMotionPrimitive, self.moveRobot)
    		print "Ready to execute plans."
    		self.flag=0
		rospy.spin()

#if __name__ == "__main__":
  #  execute_command_server()
if __name__ == '__main__':
    
	motionserver = MotionServer()
	motionserver.stop()
    
	try:
		motionserver.execute_command_server()
	except rospy.ROSInterruptException:
		pass
