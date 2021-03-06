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
		self.face_detected = True

		self.bridge = CvBridge()

		# Publicaiton

		# To do : publish ros message topic: /node_name/image_with_face, datatype: Image 
		self.pub_image_face = rospy.Publisher("XXX", XXX, queue_size=1)

        # Subscription
		# To do : subscribe ros message topic: /node_name/image, datatype: CompressedImage, callback function: self.cbImage
		self.sub_image_origin = rospy.Subscriber("XXX", CompressedImage, self.cbImage, queue_size=1)

		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)

		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)

		# Send stop command
		car_control_msg = Twist2DStamped()
		car_control_msg.v = 0.0
		car_control_msg.omega = 0.0
		self.publishCmd(car_control_msg)
		rospy.sleep(0.5) #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" %self.node_name)

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
			self.cbFacedetect(image_msg)
		finally:
			self.thread_lock.release()


	def cbFacedetect(self, image_msg):
		# Decompress image and convert ROS image message to cv image
		narr = np.fromstring(image_msg.data, np.uint8)
		image = cv2.imdecode(narr, cv2.CV_LOAD_IMAGE_COLOR)
        
        # Initial opencv CascadeClassifier class to detect objects and import face detection module
		faceCascade = cv2.CascadeClassifier('/home/ubuntu/duckietown/catkin_ws/src/spring2016_nctu/wama/face_detector/src/haarcascade_frontalface_default.xml')
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		# Detect faces in the image
		faces = faceCascade.detectMultiScale(gray,scaleFactor=2,minNeighbors=5,minSize=(10, 10),flags = cv2.cv.CV_HAAR_SCALE_IMAGE)
		print "Found {0} faces!".format(len(faces))

        # Draw face detections region proposals in the image
		for (x, y, w, h) in faces:
			cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
	
		# Convert cv image to ROS image message
		image_msg_out = self.bridge.cv2_to_imgmsg(image, "bgr8")
		image_msg_out.header.stamp = image_msg.header.stamp

		# to do: using pub_image_face publisher we initialed at line 27 to publish image with face region proposals
		self.pub_image_face.publish(image_msg_out)


if __name__ == "__main__":

	# to do: initial a node named "face_detector_X", X= you duckiebot name
	rospy.init_node("faceProcessing",anonymous=False)

	face_detector_wama_node = face_detector_wama()
	rospy.spin()
