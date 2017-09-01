#!/usr/bin/env python
import rospy
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import time
import threading
class face_detector_wama(object):
	def __init__(self):
		self.node_name = rospy.get_name()

		self.thread_lock = threading.Lock()
		self.active = True

		# to do: initial no-faces-detected as face_detected senario
		self.face_detected = True  # I don't know what is this parameter do, so I set it true first

		self.bridge = CvBridge()

		# Publicaiton


		# To do : publish ros message topic: /node_name/image_with_face, datatype: Image 
		self.pub_image_face = rospy.Publisher("imagef", Image, queue_size=1)

        # Subscription

		# To do : subscribe ros message topic: /node_name/image, datatype: CompressedImage, callback function: self.cbImage
		self.sub_image_origin = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.cbImage, queue_size=1)

		# safe shutdown
		#rospy.on_shutdown(self.custom_shutdown)

		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))



	def cbImage(self, image_msg):
		if not self.active:
			return

		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()

	def processImage(self, image_msg):
		if not self.thread_lock.acquire(False):
			return

		try:
			self.theTest(image_msg)
		finally:
			self.thread_lock.release()


        def theTest(self, image_msg):
		# Decompress image and convert ROS image message to cv image
		narr = np.fromstring(image_msg.data, np.uint8)
		image = cv2.imdecode(narr, 3)
		cv2.imshow("image",image)
		cv2.waitKey(0)
		#print len(narr)
		

if __name__ == "__main__":

	# to do: initial a node named "face_detector_X", X= you duckiebot name
	rospy.init_node("face_detector_vsung",anonymous=False)

	face_detector_wama_node = face_detector_wama()
	rospy.spin()
