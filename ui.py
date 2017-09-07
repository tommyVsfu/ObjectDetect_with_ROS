#!/usr/bin/env python
import rospy
import math
import threading
from duckietown_msgs.msg import Twist2DStamped, BoolStamped,ObstacleProjectedDetectionList, Dmp, Vector2D
class SimpleStopControllerNode:

   	def __init__(self):
        	self.name = 'simple_stop_controller_node'
        	rospy.loginfo('[%s] started', self.name)
		self.raw = 0.0
		self.dataDegrees = Dmp()
		self.testtime = rospy.Time(0)
		self.cmd = 'A'
        #self.sub_close = rospy.Subscriber("/obstacle_safety_node/object_too_close", BoolStamped, self.cbBool, queue_size=1)
		self.projection = rospy.Subscriber("/obstacle_safety_node/detection_list_proj",ObstacleProjectedDetectionList,self.cbdata_transfer,queue_size = 1)
		self.projection = rospy.Subscriber("Filter",Dmp,self.cbimudata,queue_size = 1)
		self.pp = rospy.Subscriber("test",Vector2D,self.cbpp,queue_size = 1)
        	self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
      		self.t1 = threading.Thread(target = self.f1, name='control')
      		self.t2 = threading.Thread(target = self.f2, name='imu')
      		self.t3 = threading.Thread(target = self.f3, name = 'ScheduleThread')
		self.threadC = threading.Condition()

	def fuck(self,v, omega):
		#rospy.sleep(0.3)
    		car_cmd = Twist2DStamped()
    		car_cmd.v = v
   		car_cmd.omega = omega
   		self.pub_car_cmd.publish(car_cmd)
		#print"fuck"
		
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
    		if dist != 0:
        		rospy.loginfo("here is x, y, dist")
        		rospy.loginfo(str(x))
        		rospy.loginfo(str(y))
        		rospy.loginfo(str(dist))
        		theta = math.atan(y/x)
        	
			rospy.loginfo("theta is")
			rospy.loginfo(str(math.degrees(theta)))
        	#rospy.loginfo("omega is")
        	#rospy.loginfo(str(omega))

        # control decision	
			#kinect = Twist2DStamped()
			#if math.degrees(theta) > 0 and dist > 0.3:
			#	self.choose("F")
			#elif math.degrees(theta) < 0 and dist > 0.3:
			#	self.choose("G")
			#elif dist < 0.3 :
				#self.choose("S")
			if math.degrees(theta) > 30.0:
				self.threadC.acquire()
				if self.t1.isAlive():
					self.t1.start()
				self.cmd = 'L'
				self.threadC.release()
			elif math.degrees(theta) < -30.0:
				self.threadC.acquire()
				if self.t1.isAlive():
					self.t1.start()
				self.cmd = 'R'
				self.threadC.release()
			elif :
				self.threadC.acquire()
				self.cmd = 'S'
				self.threadC.release()
				self.desire(self.cmd)
		

		else:
			kinect = Twist2DStamped()
			self.choose("B")


	def cbpp(self,dd):
		if dd.x == 10.0:
			self.t1.start()

	def f1(self):
		self.threadC.acquire()
		self.threadC.wait()
		self.threadC.release()

	def cbimudata(self,theta):
		tmp = Dmp()
                tmp.thetax = theta.thetax
                tmp.thetay = theta.thetay
                tmp.thetaz = theta.thetaz
		print ("t1 alive?",self.t1.isAlive())
		print ("t2 alive?",self.t2.isAlive())
		if self.t1.isAlive() and not self.t2.isAlive():
			self.t2.start()
			self.dataDegrees.thetax = theta.thetax
			self.dataDegrees.thetay = theta.thetay
			self.dataDegrees.thetaz = theta.thetaz
			self.raw = self.dataDegrees.thetay
			print(str(self.raw),"raw")
		elif self.t1.isAlive() and self.t2.isAlive():
			self.acquire()
			command = self.cmd
			self.release()					
			self.continuous(tmp,command)		
			
			

	def f2(self):
		self.threadC.acquire()
		self.threadC.wait()
		self.threadC.release()

	def desire(self,cc):
		for i in range(len(cc)):
			pri = cc[i]
			if pri == 'S': #forward
				self.fuck(0.5,0)
			if pri == 'B': #back
				self.fuck(-0.5,0)
			if pri == 'R': #right
				self.fuck(0,-5)
			if pri == 'L': #left
				self.fuck(0,5)
			if pri == 'M':
				self.fuck(0,0)

	def continuous(self, dataDegrees,command):
		#rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@")
		if command == 'R':
			self.desire('R')
		elif command == 'L':
			self.desire('L')
		error = 0.1	
		xr = math.cos(self.raw)
		yr = math.sin(self.raw)
		x = math.cos(dataDegrees.thetay)
		y = math.sin(dataDegrees.thetay)
		d = math.sqrt( (x - xr)**2 + (y - yr)**2 )
		value = (2 - (d**2)) / float(2)
		desire = math.degrees(math.acos(value))
		if desire >= 30.0:
			self.t3.start()
			self.desire('M')
			rospy.loginfo("-----------------------------------------------------------------------------------------")

	def f3(self):
		self.threadC.acquire()
		self.threadC.notifyAll()
		print self.t1.isAlive()
		print self.t2.isAlive()
		self.threadC.release()

	

if __name__=="__main__":
    rospy.init_node('simple_stop_controller_node')
    node = SimpleStopControllerNode()
    rospy.spin()

