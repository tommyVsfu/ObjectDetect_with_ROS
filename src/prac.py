#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped,Vector2D, BoolStamped,ObstacleProjectedDetectionList
class SimpleStopControllerNode:

   def __init__(self):
        self.name = 'simple_stop_controller_node'
        rospy.loginfo('[%s] started', self.name)
	#self.testtime = rospy.Time(0)
        #self.sub_close = rospy.Subscriber("/obstacle_safety_node/object_too_close", BoolStamped, self.cbBool, queue_size=1)
	self.projection = rospy.Subscriber("~test",Vector2D,self.cbdata_transfer,queue_size = 1)
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)

   def cbdata_transfer(self, data):
	car_cmd = Twist2DStamped()
	car_cmd.v = data.x
	car_cmd.omega = data.y
	self.pub_car_cmd.publish(car_cmd)
	rospy.sleep(1.0)
	
	car_cmd.v = 0.0
	car_cmd.omega = 0.0
	self.pub_car_cmd.publish(car_cmd)

	

	

	

if __name__=="__main__":
    rospy.init_node('simple_stop_controller_node')
    node = SimpleStopControllerNode()
    rospy.spin()
