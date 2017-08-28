# 採用 fixed color map 把 object 之外的東西濾掉，然後畫出contour，並且計算出Rect的x,y,w,h 
import cv2
import numpy as np
from matplotlib import pyplot as plt
plt.rcParams['figure.figsize'] = (10, 10)  # large images
plt.rcParams['image.interpolation'] = 'nearest' 

def ImageProcess():
    img = cv2.imread("output2.jpg")
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    frame_threshed = cv2.inRange(hsv_img, np.array([0,130,46]), np.array([10,255,255]))  #color map 我有親手調過

    ret,thresh = cv2.threshold(frame_threshed,46,255,0)

    im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
    # 抓出最大contour_area 的就是object_box (但這不是一個robust 的辦法)
    tmp = np.array([(contour_area[i][0])  for i in range(len(contour_area))])
    maxindex = np.argmax(tmp)

    area, cnt = contour_area[maxindex]
    print area
    x,y,w,h = cv2.boundingRect(cnt)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "ObjectBox", (x,y-5),font,0.5,[0,255,0],2)
    cv2.rectangle(img,(x,y),(x+w,y+h),[0,255,0],2)
    #cv2.drawContours(img, contours, -1, (0,255,0), 3)
    cv2.imshow("img",img)
    cv2.waitKey(0)
    #cv2.imshow("img",img)
    #cv2.waitKey(0)
if __name__ == '__main__':
    ImageProcess()