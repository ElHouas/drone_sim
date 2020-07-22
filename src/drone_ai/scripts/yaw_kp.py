#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

from cv_bridge import CvBridge, CvBridgeError
import cv2

from copy import deepcopy
from math import *
import numpy as np
import time

#from helpers.openpose import OpenPose
#openpose = OpenPose()
pose = Pose()
x_fpv, y_fpv = [960, 720]

from helpers.trtpose import TrtPose
trtpose = TrtPose()

from helpers.control import Control
control = Control()
kp = 0.5

class Yaw(object):
    def __init__(self):
        rospy.init_node('yaw_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.current_yaw = 0.0

        rospy.Subscriber("/tello/camera/image_raw/", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        rospy.Subscriber ('/tello/odom', Pose, self.pose_callback)
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.move_msg = Twist()

        control.takeoff()
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            if self.frame is not None:
                start_time = time.time()
                frame = self.frame
                
                current_yaw = deepcopy(self.current_yaw)
                
                # To-do: multithread or service node
                # https://answers.ros.org/question/287178/multithreading-vs-ros-nodes/
                object_counts, objects, normalized_peaks, topology = trtpose.detect(frame)
                height = frame.shape[0]
                width = frame.shape[1]
                             
                K = topology.shape[0]
                count = int(object_counts[0])
                K = topology.shape[0]
                poses = []
                for i in range(count):
                    color = (0, 255, 0)
                    obj = objects[0][i]
                    C = obj.shape[0]
                    pose = []
                    for j in range(C):
                        k = int(obj[j])
                        if k >= 0:
                            peak = normalized_peaks[0][j][k]
                            x = round(float(peak[1]) * width)
                            y = round(float(peak[0]) * height)
                            body_part = [x,y]
                            pose.append(body_part)
                            cv2.circle(frame, (x, y), 3, color, 2)
                            
                    poses.append(pose)

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
                
                
                # # To-do: multithread or action node
                print('poses', len(poses))
                if is not poses: # nose point
                    continue
                else:
                    x, y = poses[0][0]
                    yaw_angle = trtpose.calcYawAngle([x, y])
                    print('YAW', yaw_angle)
                    # To-do: Investigation on proportional controller
                    # https://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/
                    self.move_msg.angular.z = kp * (yaw_angle*pi/180 - current_yaw)
                    #self.pub_cmd_vel.publish(self.move_msg)
                
                    
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

    def pose_callback(self, data):
        orientation = data.pose.orientation
        print('orientation', orientation)
        self.current_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]

    def shutdown(self):
        cv2.destroyAllWindows()
        control.land()

def main():
    try:
        Yaw()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()
