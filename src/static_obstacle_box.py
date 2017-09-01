#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import ObstacleImageDetection, ObstacleImageDetectionList, ObstacleType, Rect, BoolStamped,ObstacleImageDetectionwithimageList
import sys
import threading
from rgb_led import *


class Matcher:
    CONE = [np.array(x, np.uint8) for x in [[0,80,80], [22, 255,255]] ]
    DUCK = [np.array(x, np.uint8) for x in [[25,100,150], [35, 255, 255]] ]
    terms = {ObstacleType.CONE :"cone", ObstacleType.DUCKIE:"duck"}
    def __init__(self):
            rospy.loginfo("build Matcher")
            self.bridge1 = CvBridge()

    def get_filtered_contours(self,img):
                hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                frame_threshed = cv2.inRange(hsv_img, np.array([0,130,46]), np.array([10,255,255]))  #color map 
                ret,thresh = cv2.threshold(frame_threshed,46,255,0)

                im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

                contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
                
                tmp = np.array([(contour_area[i][0])  for i in range(len(contour_area))])
                maxindex = np.argmax(tmp)

                area, cnt = contour_area[maxindex]
                print area
                x,y,w,h = cv2.boundingRect(cnt)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(img, "ObjectBox", (x,y-5),font,0.5,[0,255,0],2)
                cv2.rectangle(img,(x,y),(x+w,y+h),[0,255,0],2)
                return x,y,w,h,img


    def contour_match(self, img):
        '''
        Returns 1. Image with bounding boxes added
                2. an ObstacleImageDetectionList
        '''

        object_list = ObstacleImageDetectionwithimageList()
        object_list.list = []

        height,width = img.shape[:2]
        object_list.imwidth = width
        object_list.imheight = height
        
        # get filtered contours
        x,y,w,h,img= self.get_filtered_contours(img)
        object_list.im = self.bridge1.cv2_to_imgmsg(img,"bgr8") 
                            

                
        r = Rect()
        r.x = x
        r.y = y
        r.w = w
        r.h = h
	rospy.loginfo("the x,y,h,w is")
	rospy.loginfo(str(x))
	rospy.loginfo(str(y))
	rospy.loginfo(str(w))
	rospy.loginfo(str(h))
                #t = ObstacleType()
                #t.type = 

        d = ObstacleImageDetection()
        d.bounding_box = r
                #d.type = t
        object_list.list.append(d);
        return img, object_list

class StaticObjectDetectorNode:
    def __init__(self):
        self.name = 'static_object_detector_node'
        
        self.tm = Matcher()
        self.active = True
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/decoder_low_freq/image/raw", Image, self.cbImage, queue_size=1)
        #self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        self.pub_image = rospy.Publisher("~cone_detection_image", Image, queue_size=1)
        #self.pub_detections_list = rospy.Publisher("~detection_list", ObstacleImageDetectionList, queue_size=1)
        self.pub_detections_list = rospy.Publisher("~detection_list", ObstacleImageDetectionwithimageList, queue_size=1)
        self.bridge = CvBridge()
	#turn_off_LEDs(speed=5)

        rospy.loginfo("[%s] Initialized." %(self.name))

    def cbImage(self,image_msg):
        if not self.active:
            return
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):

            return
        try:
            image_cv=self.bridge.imgmsg_to_cv2(image_msg,"bgr8")
            print "you"
        except CvBridgeErrer as e:
            print "fuck1"
            print e

        img, detections = self.tm.contour_match(image_cv)
        detections.header.stamp = image_msg.header.stamp
        detections.header.frame_id = image_msg.header.frame_id
        self.pub_detections_list.publish(detections)
        height,width = img.shape[:2]
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print("fuck2")
            print(e)

        self.thread_lock.release()

if __name__=="__main__":
	rospy.init_node('static_object_detector_node')
	node = StaticObjectDetectorNode()
	rospy.spin()
