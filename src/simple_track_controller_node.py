#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, BoolStamped,ObstacleProjectedDetectionList
class SimpleStopControllerNode:

   def __init__(self):
        self.name = 'simple_stop_controller_node'
        rospy.loginfo('[%s] started', self.name)
	self.testtime = rospy.Time(0)
        #self.sub_close = rospy.Subscriber("/obstacle_safety_node/object_too_close", BoolStamped, self.cbBool, queue_size=1)
	self.projection = rospy.Subscriber("/obstacle_safety_node/detection_list_proj",ObstacleProjectedDetectionList,self.cbdata_transfer,queue_size = 1)
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)

	def fuck(self,v, omega, duration):
		#rospy.sleep(0.3)
    	car_cmd = Twist2DStamped()
    	car_cmd.v = v
   		car_cmd.omega = omega
   		self.pub_car_cmd.publish(car_cmd)
		print"fuck"
		rospy.sleep(duration)	
		#print "fuck"
		#car_cmd.v = 0.0
		#car_cmd.omega = 0.0
		#self.pub_car_cmd.publish(car_cmd)

	def choose(self,cat):
		for i in range(len(cat)):
			pri = cat[i]
			if pri == 'S': #forward
				self.fuck(4.0,0,1)
			if pri == 'B': #back
				self.fuck(-0.5,0,2)
			if pri == 'R': #right
				self.fuck(0,-8,1.0)
			if pri == 'L': #left
				self.fuck(0,8,1.125)
			if pri == 'O': #left+forward,90 degrees
				self.fuck(0.50,8.9,2)
				self.fuck(0,8,0.5)
			if pri == 'P': #right+forward, 90degrees
				self.fuck(0.5,-8,2)
			if pri == 'G': #right+forward, 30degrees
				self.fuck(0.5,-8,1.2)
				self.fuck(0.5,0,0.5)
			if pri == 'F': #left+forward,30 degrees
				self.fuck(0.45,8,2)
			if pri == 'M': #stop!
				self.fuck(0,0,2)
				
   def cbdata_transfer(self, data):
	stamp = rospy.Time.now()
	tmp1 = stamp.to_sec()
	tmp2 = self.testtime.to_sec()
        self.testtime = stamp
        x = data.list[0].location.x
        y = data.list[0].location.y
        dist = data.list[0].distance
        rospy.loginfo("here is x, y, dist")
        rospy.loginfo(str(x))
        rospy.loginfo(str(y))
        rospy.loginfo(str(dist))
	
	#if tmp2 != 0.0:
		#time = tmp1-tmp2
		#rospy.loginfo("time is")
		#rospy.loginfo(str(time))
       		 # for omega, right is negative
        theta = math.atan(y/x)
        	#omega = theta / time
		rospy.loginfo("theta is")
		rospy.loginfo(str(math.degrees(theta)))
        	#rospy.loginfo("omega is")
        	#rospy.loginfo(str(omega))

        # control decision	
		kinect = Twist2DStamped()
		if dist > 0.3:
			self.choose("S")
		elif dist < 0.1:
			self.choose("M")
		else:
			if math.degrees(theta) > 0:
				self.choose("G")
			else:
				self.choose("F")

		#rospy.loginfo("v,omega is")
		#rospy.loginfo(str(kinect.v))
		#rospy.loginfo(str(kinect.omega)) 
		#self.pub_car_cmd.publish(kinect)

	

	

if __name__=="__main__":
    rospy.init_node('simple_stop_controller_node')
    node = SimpleStopControllerNode()
    rospy.spin()
