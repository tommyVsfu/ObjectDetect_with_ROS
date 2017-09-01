#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped,Vector2D
class qti(object):
	def __init__(self):
	
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
			if pri == 'S':
				self.fuck(4.0,0,1)
			if pri == 'B':
				self.fuck(-0.5,0,2)
			if pri == 'R':
				self.fuck(0,-8,1.0)
			if pri == 'L':
				self.fuck(0,8,1.125)
			if pri == 'O':
				self.fuck(0.50,8.9,2)
				self.fuck(0,8,0.5)
			if pri == 'P':
				self.fuck(0.5,-8,2)
			if pri == 'M':
				self.fuck(0,0,2)


if __name__ == '__main__':
	rospy.init_node('simple_stop_controller_node')
	ss = qti()
	rospy.sleep(0.5) #need to delay
	ss.choose("LM")	
	print "hi"


    
