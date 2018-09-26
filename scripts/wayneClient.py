#!/usr/bin/env python

# license removed for brevity
import threading
import math
from collections import namedtuple
import rospy
import roslib; roslib.load_manifest('mobile_robot')
#import roslib; roslib.load_manifest('mobile_robot')
from std_msgs.msg import String
from std_msgs.msg import Float32
from mobile_robot.srv import *
import time
import yaml
#from beginner_package.srv import *


global position
global distance
distance = (0,0,0)
global flag
flag=0
global check_left
check_left=0

global check_right
check_right=0
global check_front
check_front=0
def callback_location(data):
	global position
	global flag
	position = data.data
	
	flag=1
	#rospy.loginfo("calling "+str(data)+"\t"+str(flag)+"\n")
	#return
def callback_front_ultrasonic(data):
	global distance
	global check_front
	check_front = 1
	#rospy.loginfo("f "+str(distance))
	distance=(data.data,distance[1],distance[2])
	
def callback_left_ultrasonic(data):
	global check_left
	check_left=1
	global distance
	#rospy.loginfo("l "+str(distance))
	distance=(distance[0],data.data,distance[2])
	
def callback_right_ultrasonic(data):
	global distance
	global check_right
	check_right = 1
	distance=(distance[0],distance[1],data.data)
	#rospy.loginfo("r "+str(distance)+str(data.data))
	
class voltbot():
    Position = namedtuple('Position', ['x', 'y', 'value'])
    PlanInstance = namedtuple('PlanInstance',['robot_id','horizon_instance','x','y','theta'])

    def __init__(self):
        self.robot_config = yaml.load(open('/home/pi/catkin_ws/src/mobile_robot/scripts/robot_config.yaml'))

        rospy.init_node('voltbot_controller', anonymous=True)
        self.rate = rospy.Rate(10)
        self.robot_id = 0
	self.robot_x = -100
	self.robot_y = -100
	self.robot_theta = -100
	self.workspace_x = self.robot_config['workspace_dim']['x']
        self.workspace_y = self.robot_config['workspace_dim']['y']
        self.workspace = [[-1 for y in range(self.workspace_y)] for x in range(self.workspace_x)]
        self.current_horizon = 0
        self.share_plan_service = 0

        self.forward_avg = 0.0
        self.right_avg = 0.0
        self.left_avg = 0.0

        self.forward_var = 0.0
        self.right_var = 0.0
        self.left_var = 0.0

        self.forward_count = 0
        self.left_count = 0
        self.right_count = 0

        self.forward_square_dist = 0
        self.left_square_dist = 0
        self.right_square_dist = 0

        self.prev_primitive = None

        self.getPlanServer()
        
        self.myfile = open("prim_times.txt","a")
        
	rospy.Subscriber(self.robot_config['position_topic'],String,self.callback_location)
	rospy.Subscriber(self.robot_config['dist_topic']['front'],Float32,callback_front_ultrasonic)
	rospy.Subscriber(self.robot_config['dist_topic']['left'],Float32,callback_left_ultrasonic)
	rospy.Subscriber(self.robot_config['dist_topic']['right'],Float32,callback_right_ultrasonic)

	#rospy.spin()
	# Getting the location	
    ## Up the service which recives plan from the global planner   
    def getPlanServer(self):
        plan_service_name = "robot_";
        plan_service_name += str(self.robot_id);
        plan_service_name += "/share_plan";
        self.share_plan_service = rospy.Service(plan_service_name, mobile_robot.srv.PlanForHorizon, self.getPlan)
        #print("Service Name " + plan_service_name)
        print("Robot %d ready to receive plan from global planner"%self.robot_id)

    def callback_location(self, data):
	position = data.data
	#rospy.spinonce()
	s1=position.find(' ')
	self.robot_x = float(position[:s1])
	position=position[s1+1:]
	s2=position.find(' ')
	self.robot_y = float(position[:s2])
	position = position[s2+1:]
	self.robot_theta=float(position)*math.pi/180.0
	
    def printWorkspace(self):
        print("Print local view of Robot:%d"%self.robot_id)
        print
        for j in range(self.workspace_y-1, -1, -1):
            print("%d \t" %j),
            for i in range(self.workspace_x):
                print("%lf \t" % self.workspace[i][j]),
            print
        print("")
        for i in range(self.workspace_x):
                print("\t %d \t" % i),
        print
    
    # Estimate the intended theta from the actual location
    def check_intended_theta(self, theta):
       if(theta < 0):
           theta = 2*math.pi + theta
       intended_theta = (int(round(theta/(math.pi/2))))%4
       #print("Actual Theta: %lf, Intended Theta: %lf"%(theta,intended_theta))
       return intended_theta

    def get_rotated_location(self,temp_x,temp_y,cell_x,cell_y,theta):
        tmp_theta = theta*(math.pi/2)
        next_x = round(((temp_x * math.cos(tmp_theta) - temp_y * math.sin(tmp_theta))))
        next_y = round(((temp_x * math.sin(tmp_theta) + temp_y * math.cos(tmp_theta))))
        sensed_x =  next_x + cell_x
        sensed_y =  next_y + cell_y
        #print("X: %lf Y: %lf Theta: %lf"%(next_x,next_y,tmp_theta))
        return (sensed_x,sensed_y)
        
    def get_motion_prim(self, loc_x, loc_y, loc_theta, prim_x, prim_y, prim_theta):		#1 for forward, 2 for left, 3 for right
    	if(prim_theta == (int(round(loc_theta/(math.pi/2))))%4):
    		return 1
    	else:
    		if(prim_theta - (int(round(loc_theta/(math.pi/2))))%4 == 1 or prim_theta - (int(round(loc_theta/(math.pi/2))))%4 == -3):
    			return 2
    		else:
    			return 3
    			
    		

    # Get the visible cells
    def get_visible_cells(self,cell_x,cell_y,theta):
    	time.sleep(0.5)
        [front,left,right] = self.getUltraSonicValue()
        print("Front: %lf Left: %lf Right: %lf"%(front,left,right))
        visible_cells = []
        # checking the front 
        [loc_x,loc_y] = self.get_rotated_location(1,0,cell_x,cell_y,theta)
        if((loc_x >= 0 and loc_x < self.workspace_x) and ((loc_y >= 0 and loc_y < self.workspace_y))):
        	if(front >= 50):
        		temp_position = self.Position(loc_x,loc_y,0.5)
        	else:
        		temp_position = self.Position(loc_x,loc_y,0.0)
        	visible_cells.append(temp_position)
        		
        # checking the left 
        [loc_x,loc_y] = self.get_rotated_location(0,1,cell_x,cell_y,theta)
        if((loc_x >= 0 and loc_x < self.workspace_x) and ((loc_y >= 0 and loc_y < self.workspace_y))):
        	if(left >= 50):
        		temp_position = self.Position(loc_x,loc_y,0.5)
        	else:
        		temp_position = self.Position(loc_x,loc_y,0.0)
        	visible_cells.append(temp_position)
        
        # checking the right 
        [loc_x,loc_y] = self.get_rotated_location(0,-1,cell_x,cell_y,theta)
        if((loc_x >= 0 and loc_x < self.workspace_x) and ((loc_y >= 0 and loc_y < self.workspace_y))):
        	if(right >= 50):
        		temp_position = self.Position(loc_x,loc_y,0.5)
        	else:
        		temp_position = self.Position(loc_x,loc_y,0.0)
        	visible_cells.append(temp_position)
        
        return visible_cells

    def check_visible_region(self, loc_x,loc_y,theta):
        cell_x = int(loc_x)
        cell_y = int(loc_y)
        intended_theta = self.check_intended_theta(theta)
        print("Actual Pose: X: %lf Y: %lf Theta: %lf Cell Pose: X: %d, Y: %d, Theta: %d"%(loc_x,loc_y,theta,cell_x,cell_y,intended_theta))
        visible_cells = self.get_visible_cells(cell_x,cell_y,intended_theta)
        total_visible_cells = len(visible_cells)
        for i in range(total_visible_cells):
            print("Visible Cell: %d Location: X: %d Y: %d Value: %lf"%(i,visible_cells[i].x,visible_cells[i].y,visible_cells[i].value))
        return visible_cells
            
     # Update the workspace based on the current location and visibility           
    def updateWorkspace(self):
    	
        [loc_x, loc_y, theta] = self.getPose()
        print("Location: X: %lf Y: %lf Theta: %lf"% (loc_x, loc_y, theta))
        cell_x = int(loc_x)
        cell_y = int(loc_y)
        self.workspace[cell_x][cell_y] = 1.0
        visible_cells = self.check_visible_region(loc_x,loc_y,theta)
        total_visible_cells = len(visible_cells)
        for i in range(total_visible_cells):
            temp_x = int(visible_cells[i].x)
            temp_y = int(visible_cells[i].y)
            temp_value = visible_cells[i].value
            #if((self.workspace[temp_x][temp_y] < temp_value)and (self.workspace[temp_x][temp_y]!=0.0)):
	    if((self.workspace[temp_x][temp_y]<0) or (abs(self.workspace[temp_x][temp_y]-0.5)<0.01)):	
                self.workspace[temp_x][temp_y] = temp_value


    def shareWorkspaceInformation(self):
        rospy.wait_for_service('share_workspace')
        [loc_x,loc_y,theta] = self.getPose()
        
        try:
            share_local_information = rospy.ServiceProxy('share_workspace', ShareLocalInformation)
            temp_request = mobile_robot.srv.ShareLocalInformation()
            temp_request.robot_id = self.robot_id
            temp_request.horizon = self.current_horizon
            temp_request.x = loc_x
            temp_request.y = loc_y
            temp_request.theta = self.check_intended_theta(theta)
            temp_request.workspace = []
            for i in range(self.workspace_x):
                for j in range(self.workspace_y):
                    temp_request.workspace.append(self.workspace[i][j])
            #temp_request.robot_pose = Pose2D(loc_x,loc_y,theta)
            #resp = share_local_information(temp_request.robot_id,temp_request.horizon, temp_request.workspace, temp_request.robot_pose)
            resp = share_local_information(temp_request.robot_id,temp_request.horizon, temp_request.x, temp_request.y, temp_request.theta, temp_request.workspace)
            print("Robot Id: %d Current Horizon: %d"% (self.robot_id, resp.next_horizon))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def getPlan(self,req):
        plan_for_horizon = []
        for i in range(len(req.plans)):
          print("Robot id: %d, Horizon_instance: %d, X: %lf Y: %lf, Theta: %lf"%(req.plans[i].robot_id,req.plans[i].horizon_step, req.plans[i].x,req.plans[i].y,req.plans[i].theta))
          if(req.plans[i].robot_id == self.robot_id):
            temp_plan_instance = self.PlanInstance(req.plans[i].robot_id,req.plans[i].horizon_step, req.plans[i].x,req.plans[i].y,req.plans[i].theta)
            plan_for_horizon.append(temp_plan_instance)
        try:
            threading.Thread(target=self.executeTotalPlan, args=(plan_for_horizon,)).start()
        except:
            print "Error: unable to start thread"
        return mobile_robot.srv.PlanForHorizonResponse(self.current_horizon)

    def executeTotalPlan(self,plan_for_horizon):
        # [prev_x, prev_y, prev_theta] = [cur_x, cur_y, cur_theta]
    	[cur_x, cur_y, cur_theta] = self.getPose()
        cell_x = int(cur_x)
        cell_y = int(cur_y)
        cell_theta = (int(round(cur_theta/(math.pi/2))))%4
        
        datapt = str(self.robot_id) + "\t" + str(len(plan_for_horizon)) + "\t"
        times = ""
        
        horizon_start_time = time.time()
        
        print("Robot: %d Ready to execute received plan in Horizon %d"%(self.robot_id,self.current_horizon))
        if(len(plan_for_horizon) == 0):
            print("Coverage Completed.......Robot %d is shutting down.")
        for i in range(1, len(plan_for_horizon)):
            prim_start_time = time.time()
            datapt = datapt + str(self.get_motion_prim(cell_x, cell_y, cell_theta,  plan_for_horizon[i].x,plan_for_horizon[i].y,plan_for_horizon[i].theta))
            print("Robot id: %d, Horizon_instance: %d, X: %lf Y: %lf, Theta: %lf"%(plan_for_horizon[i].robot_id,plan_for_horizon[i].horizon_instance, plan_for_horizon[i].x,plan_for_horizon[i].y,plan_for_horizon[i].theta))
	    tt = self.commandExecutioner( plan_for_horizon[i].x,plan_for_horizon[i].y,plan_for_horizon[i].theta)          
	    print('time taken in execution : ',tt)
            print('forward average: ', self.forward_avg,' forward variance: ', self.forward_var)
            print('left average: ', self.left_avg,' left variance: ', self.left_var)
            print('right average: ', self.right_avg,' right variance: ', self.right_var)
            if(self.prev_primitive == None):
                self.prev_primitive = self.getPose()
                self.prev_primitive = [int(x) for x in self.prev_primitive]
                
            else:
                temp_pr = self.getPose()
                print("-"*22, temp_pr, self.prev_primitive)
                temp_pr = [int(x) for x in temp_pr]
                if(temp_pr[0] == self.prev_primitive[0] and temp_pr[1] == self.prev_primitive[1]):
                        if( (temp_pr[2]-self.prev_primitive[2])%4 == 1 ):
                                # left movement
                                self.left_count += 1
                                delta = tt - self.left_avg
                                self.left_avg += delta/self.left_count
                                delta2 = tt - self.left_avg
                                self.left_square_dist += delta*delta2

                                self.left_var = self.left_square_dist/self.left_count

                        elif( (temp_pr[2] - self.prev_primitive[2])%4 == 3 ):
                                #right movement
                                self.right_count += 1
                                delta = tt - self.right_avg
                                self.right_avg += delta/self.right_count
                                delta2 = tt - self.right_avg
                                self.right_square_dist += delta*delta2

                                self.right_var = self.right_square_dist/self.right_count
                else:
                        # forward movement
                        self.forward_count += 1
                        delta = tt - self.forward_avg
                        self.forward_avg += delta/self.forward_count
                        delta2 = tt - self.forward_avg
                        self.forward_square_dist += delta*delta2

                        self.forward_var = self.forward_square_dist/self.forward_count
                self.prev_primitive = temp_pr
                        




            self.updateWorkspace()
            self.printWorkspace()
            prim_end_time = time.time()
            prim_time = prim_end_time - prim_start_time
            times = times + str(prim_time) + "\t"
            time.sleep(1.5) 
        datapt = datapt + "\t"
        datapt = datapt + times
        
        horizon_end_time = time.time()
        horizon_time = horizon_end_time - horizon_start_time
        datapt = datapt + str(horizon_time) + "\n"
        
        self.myfile.write(datapt)
        
        self.updateHorizonInformation()  
        self.shareWorkspaceInformation()
        

    def updateHorizonInformation(self):
        self.current_horizon = self.current_horizon + 1
        print("Next Horizon: %d"%self.current_horizon)
        
    def commandExecutioner(self,x, y, theta):
    	start = time.time()
	rospy.wait_for_service(self.robot_config['command_service_name'])
    	try:
        	execute_command_client = rospy.ServiceProxy(self.robot_config['command_service_name'], ExecuteMotionPrimitive)
		print("Going to execute : ",x,y,theta)
        	resp1 = execute_command_client(x,y,theta)
        	#print(resp1)
		tt = time.time() - start
        	return tt
    	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e

    def getPose(self):
        
	time.sleep(0.1)
        while((self.robot_x <= -100) or (self.robot_y <= -100) or (self.robot_theta <= -100)):
        	time.sleep(0)	
		print("waitiing for position \n")
        return (self.robot_x, self.robot_y,self.robot_theta)

    def getUltraSonicValue(self):
        global distance
	#print(distance)
	global check_front
	global check_left
	global check_right		
	while(check_front+check_left+check_right < 3):
		time.sleep(0)
	time.sleep(1)
        return distance


    
if __name__ == '__main__':
   # try:
  #  	global robot
    	
    	#command=[[1,2,1], [1,3,1],[1,3,2],[0,3,2],[0,3,3]]
    	command=[ [2,2,1],[2,2,2],[1,2,2],[1,2,1],[1,3,1]]
	#command=[[1,2,1],[1,2,0],[1,2,3],[1,2,0]]
	#command=[[4,2,0]]
    	#command=[[4,,2,0], [4,2,1],[4,3,1],[4,4,1],[4,4,2]]
    	
        try:
       		robot = voltbot()
        	robot.updateWorkspace()
        	robot.printWorkspace()

        	robot.shareWorkspaceInformation()
        	rospy.spin()
        except rospy.ROSInterruptException:
        	pass
 #       for i in range (len(command)):
#		robot.updateWorkspace()
  #              robot.printWorkspace()
 ##               
#		robot.shareWorkspaceInformation()	
    #    	robot.commandExecutioner(command[i][0], command[i][1], command[i][2])
   #     	robot.updateWorkspace()
  #      	robot.printWorkspace()
 #       	robot.shareWorkspaceInformation()
#		time.sleep(0.5)
#	robot.updateWorkspace()
   #     robot.printWorkspace()
  #      robot.shareWorkspaceInformation()
 #   except rospy.ROSInterruptException:
#       pass


