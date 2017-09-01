#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
class SimpleStopControllerNode:

   def __init__(self):
        self.name = 'simple_stop_controller_node'
        rospy.loginfo('[%s] started', self.name)
        self.sub_close = rospy.Subscriber("/obstacle_safety_node/object_too_close", BoolStamped, self.cbBool, queue_size=1)
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)

   def cbBool(self,data):
	rospy.loginfo(str(data.data))
	if data.data:
		rospy.loginfo("it's too close ~~~~~")
                stop = Twist2DStamped()
                stop.header = data.header
                stop.v = 0.0
                stop.omega = 0.0
                self.pub_car_cmd.publish(stop)
	if not data.data:
		rospy.loginfo("it's safe")
		move = Twist2DStamped()
		move.header = data.header
		move.v= 0.3
		move.omega = 0.0
		self.pub_car_cmd.publish(move)


if __name__=="__main__":
    rospy.init_node('simple_stop_controller_node')
    node = SimpleStopControllerNode()
    rospy.spin()
