#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import Float32
from ultrasonic import Ultrasonic
import yaml
def distPublish(frontUltrasonic):#,leftUltrasonic='null',frontUltrasonic='null'):
	
		
	#if(leftUltrasonic <> 'null'):
	pubf = rospy.Publisher(frontUltrasonic.topic, Float32, queue_size=10)
	rospy.init_node(frontUltrasonic.node, anonymous=True)
	rate=rospy.Rate(10)
	
	#if(frontUltrasonic <> 'null'):
		#pubf = rospy.Publisher(frontUltrasonic.topic, String, queue_size=10)
		#rospy.init_node(frontUltrasonic.node, anonymous=True)
		#rate=rospy.Rate(10)
	buffer = [0]*15
	while not rospy.is_shutdown():
	
	    dist = frontUltrasonic.distance()
	    buffer.insert(15,dist)
	    buffer.pop(0)
	    sortedbuffer = buffer[:]
	    sortedbuffer.sort()
	    trimmed = sortedbuffer[5:10]
	    result = sum(trimmed)/float(len(trimmed))
	    rospy.loginfo('front: '+str(result))
#	    pubr.publish(rdist)
#	    publ.publish(ldist)
	    pubf.publish(result)
	    rate.sleep()


if __name__ == '__main__':
	#rightUltrasonic = Ultrasonic(19,26,'rightdistance','publishultrasonicr')
	#leftUltrasonic = Ultrasonic(21,20,'leftdistance','publishultrasonicl')
	robot_config = yaml.load(open('/home/pi/catkin_ws/src/mobile_robot/scripts/robot_config.yaml'))

	frontUltrasonic = Ultrasonic(17,27,robot_config['dist_topic']['front'],robot_config['dist_node_name']['front'])
	try:
		#distPublish(rightUltrasonic,leftUltrasonic,frontUltrasonic)
		distPublish(frontUltrasonic)
		#distPublish(frontUltrasonic)
	except rospy.ROSInterruptException:
		pass
