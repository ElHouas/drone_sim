#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import cv2

#import keras
from math import *
import numpy as np
import time

#from helpers.openpose import OpenPose
#openpose = OpenPose()

from helpers.control import Control
control = Control()

from helpers.trtpose import TrtPose
trtpose = TrtPose()


class Tracking(object):
    def __init__(self):
        rospy.init_node('yaw_node', anonymous=True)
        self.rate = rospy.Rate(10)

        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_simulation()

        rospy.Subscriber("/drone/front_camera/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        control.takeoff()
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            if self.frame is not None:
                start_time = time.time()
                #frame = deepcopy(self.frame)
                frame = self.frame
                
                #points = openpose.detect(frame)
                #for i in range(len(points)):
                #    if points[i] is not None:
                #        frame = cv2.circle(frame, (int(points[i][0]), int(points[i][1])), 3, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                #cv2.imshow("", frame)
                #cv2.waitKey(1)

                object_counts, objects, normalized_peaks, topology = trtpose.detect(frame)
                height = frame.shape[0]
                width = frame.shape[1]
                
                K = topology.shape[0]
                count = int(object_counts[0])
                K = topology.shape[0]
                for i in range(count):
                    color = (0, 255, 0)
                    obj = objects[0][i]
                    C = obj.shape[0]
                    for j in range(C):
                        k = int(obj[j])
                        if k >= 0:
                            peak = normalized_peaks[0][j][k]
                            x = round(float(peak[1]) * width)
                            y = round(float(peak[0]) * height)
                            cv2.circle(frame, (x, y), 3, color, 2)

                    for k in range(K):
                        c_a = topology[k][2]
                        c_b = topology[k][3]
                        if obj[c_a] >= 0 and obj[c_b] >= 0:
                            peak0 = normalized_peaks[0][c_a][obj[c_a]]
                            peak1 = normalized_peaks[0][c_b][obj[c_b]]
                            x0 = round(float(peak0[1]) * width)
                            y0 = round(float(peak0[0]) * height)
                            x1 = round(float(peak1[1]) * width)
                            y1 = round(float(peak1[0]) * height)
                            cv2.line(frame, (x0, y0), (x1, y1), color, 2)
                                
                cv2.imshow("", frame)
                cv2.waitKey(1)
                #print("%s seconds" % (time.time() - start_time))

            self.rate.sleep()
    
    def camera_callback(self,data):
        try:
            cv_img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_img

    def shutdown(self):
        cv2.destroyAllWindows()
        control.land()

def main():
    try:
        Tracking()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()