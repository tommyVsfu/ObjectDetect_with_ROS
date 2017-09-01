#!/usr/bin/env python
import rospy
from ground_projection.srv import *
from duckietown_msgs.msg import Vector2D, ObstacleImageDetection, ObstacleImageDetectionList, ObstacleType, Rect, ObstacleProjectedDetectionList, ObstacleProjectedDetection, BoolStamped,ObstacleImageDetectionwithimageList
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

class ObstacleSafetyNode:
    def __init__(self):
        self.name = 'obstacle_safety_node'
        rospy.loginfo('[%s] started', self.name)
        self.bridge = CvBridge()
        self.sub_ = rospy.Subscriber("/static_object_detector_node/detection_list", ObstacleImageDetectionwithimageList, self.cbDetectionsList, queue_size=1)
        ###############################this may have problem in path variable
        rospy.wait_for_service('ground_projection/get_ground_coordinate')
        rospy.loginfo("service success")
        self.ground_proj = rospy.ServiceProxy('ground_projection/get_ground_coordinate',GetGroundCoord)

        self.pub_too_close = rospy.Publisher("~object_too_close", BoolStamped, queue_size=1)
        self.pub_projections = rospy.Publisher("~detection_list_proj", ObstacleProjectedDetectionList, queue_size=1)
        self.pub_drawimg = rospy.Publisher("~drawImagewithdist",Image,queue_size = 1)
        self.pub_markers = rospy.Publisher("~object_detection_markers", MarkerArray, queue_size=1)
        self.maxMarkers = 0

        #self.veh_name = rospy.get_namespace().strip("/")
        self.veh_name = "alphaduck"
        ################################this may have problem in situation when other node need the yaml value 0.2
        #self.closeness_threshold = self.setupParam("~closeness_threshold", 0.2)
        self.closeness_threshold = 0.3


    def cbDetectionsList(self, detections_msg):
        #For ground projection uncomment the next lines
        marker_array = MarkerArray()
        img = self.bridge.imgmsg_to_cv2(detections_msg.im, "bgr8")
        p = Vector2D()
        p2 = Vector2D()
        count = 0;

        projection_list = ObstacleProjectedDetectionList()
        projection_list.list = []
        projection_list.header = detections_msg.header

        minDist = 999999999999999999999999.0
        dists = []
        width = detections_msg.imwidth
        height = detections_msg.imheight
        too_close = False
        for obstacle in detections_msg.list: 
            marker = Marker()
            rect = obstacle.bounding_box
	    # the x,y need to corrected
	    if rect.x > rect.h:
		rospy.loginfo("this is x,y,w,h")		
	    #rospy.loginfo(str(rect.x))
	    #rospy.loginfo(str(rect.y))
	    #rospy.loginfo(str(rect.w))
	    #rospy.loginfo(str(rect.h))
	    #rect.x = rect.x - rect.w
	    rect.y = rect.y + rect.h 
		#rect.y = rect.y - rect.w
	    #rospy.loginfo(str(rect.x))
	    #rospy.loginfo(str(rect.y))
            p.x = float(rect.x)/float(width)
            p.y = float(rect.y)/float(height)
            #rospy.loginfo("p.x,p.y")
	    #rospy.loginfo(str(p.x))
	    #rospy.loginfo(str(p.y))
            p2.x = float(rect.x + rect.w)/float(width)
            p2.y = p.y

            projected_point = self.ground_proj(p)
            projected_point2 = self.ground_proj(p2)
	    rospy.loginfo("p.x p.y is")
            rospy.loginfo(projected_point.gp.x)
            rospy.loginfo(projected_point.gp.y)
	    rospy.loginfo("p.x2 p.y2 is")
            rospy.loginfo(projected_point2.gp.x)
            rospy.loginfo(projected_point2.gp.y)
	    #rospy.loginfo("p2.x p2.y is",projected_point2.gp.x,projected_point2.gp.y)
            obj_width = (projected_point2.gp.y - projected_point.gp.y)**2 + (projected_point2.gp.x - projected_point.gp.x)**2
            obj_width = obj_width ** 0.5
            rospy.loginfo("[%s]Width of object: %f" % (self.name,obj_width))
            projection = ObstacleProjectedDetection()
            projection.location = projected_point.gp
            #   projection.type = obstacle.type
	    #if abs(projected_point.gp.y) <= abs(projected_point2.gp.y):
		#projection.location.y = projected_point.gp.y
	    #else :
		#projection.location.y = projected_point2.gp.y
            projection.location.y = (projected_point.gp.y + projected_point2.gp.y) / float(2)
            rospy.loginfo("the location.y is")
	    rospy.loginfo(str(projection.location.y))
	    dist = projected_point.gp.x**2 + projected_point.gp.y**2 + projected_point.gp.z**2
            rospy.loginfo("the projected point of z is")
	    rospy.loginfo(projected_point.gp.z)
	    dist = dist ** 0.5
            rospy.loginfo("distance is ")
            rospy.loginfo(dist)
            # draw
            dst = cv2.line(img,(0,480),(200,250),[0,0,255],3)
            dst = cv2.line(dst,(100,250),(200,250),[0,0,255],3)
            font = cv2.FONT_HERSHEY_SIMPLEX
            dst = cv2.putText(dst,str(dist),(50,400),font,2,[0,255,0],2)
            
            #if dist<minDist:
            #    minDist = dist
            if dist<self.closeness_threshold:
                # Trying not to falsely detect the lane lines as duckies that are too close
                too_close = True
                #if projected_point.gp.y < 0.18:
                    # rospy.loginfo("Duckie too close y: %f dist: %f" %(projected_point.gp.y, minDist))
                    #too_close = True

            projection.distance = dist
            projection_list.list.append(projection)
            
            #print projected_point.gp
            marker.header = detections_msg.header
            marker.header.frame_id = self.veh_name
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = -0.7071
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 0.7071
            marker.pose.position.x = projected_point.gp.x
            marker.pose.position.y = projected_point.gp.y
            marker.pose.position.z = projected_point.gp.z 
            marker.id = count
            count = count +1

            marker_array.markers.append(marker)
                         
        if count > self.maxMarkers:
            self.maxMarkers = count

        while count <= self.maxMarkers:
            marker = Marker()
            marker.action = 2
            marker.id = count
            marker_array.markers.append(marker)
            count = count+1

        b = BoolStamped()
        b.header = detections_msg.header
        b.data = too_close
        self.pub_drawimg.publish(self.bridge.cv2_to_imgmsg(dst,"bgr8"))
        self.pub_too_close.publish(b)
        self.pub_projections.publish(projection_list)
        self.pub_markers.publish(marker_array)

if __name__=="__main__":
    rospy.init_node('obstacle_safety_node')
    node = ObstacleSafetyNode()
    rospy.spin()
