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

   def cbdata_transfer(self, data):
	stamp = rospy.Time.now()
	tmp1 = stamp.to_sec()
	tmp2 = self.testtime.to_sec()
        self.testtime = stamp
        x = data.list[0].location.x
        y = data.list[0].location.y
        rospy.loginfo(str(x))
        rospy.loginfo(str(y))
	
	if tmp2 != 0.0:
		time = tmp1-tmp2
		rospy.loginfo("time is")
		rospy.loginfo(str(time))
       		 # for omega, right is negative
        	theta = math.atan(y/x)
        	omega = theta / time
		rospy.loginfo("theta is")
		rospy.loginfo(str(math.degrees(theta)))
        	#rospy.loginfo("omega is")
        	#rospy.loginfo(str(omega))

		kinect = Twist2DStamped()
		#kinect.header = data.list[0].header
		kinect.v = 0.3
		if omega>0:
			kinect.omega = omega*30+0.5
		else :
			kinect.omega = omega*30-0.5
		rospy.loginfo("v,omega is")
		rospy.loginfo(str(kinect.v))
		rospy.loginfo(str(kinect.omega)) 
		self.pub_car_cmd.publish(kinect)

	

	

if __name__=="__main__":
    rospy.init_node('simple_stop_controller_node')
    node = SimpleStopControllerNode()
    rospy.spin()
