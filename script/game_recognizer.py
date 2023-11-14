#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

class GameRecognizer:
    def __init__(self):
        self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.class_names = ['f1', 'fifa', 'mariokart', 'nba']
        self.images = [cv2.imread('Games/f1.jpg', 0), cv2.imread('Games/fifa.jpg', 0), cv2.imread('Games/mariokart.jpg', 0), cv2.imread('Games/nba.jpg', 0)]
        self.desList = self.find_des(self.images)
        self.bridge = CvBridge()

    def find_des(self, images):
        desList = []
        for img in images:
            kp, des = self.orb.detectAndCompute(img, None)
            desList.append(des)
        return desList

    def findID(self, img, thres=20):
        kp2, des2 = self.orb.detectAndCompute(img, None)
        bf = cv2.BFMatcher()
        matchList = []
        # list_kp2 = []
        finalID = -1
        try:
            for des in self.desList:
                matches = bf.knnMatch(des, des2, k=2)
                good = []
                for i, j in matches:
                    if i.distance < 0.75 * j.distance:
                        good.append([i])
                matchList.append(len(good))
        except:
            pass
        if len(matchList) != 0:
            if max(matchList) > thres:
                finalID = matchList.index(max(matchList))
                # list_kp2 = [kp2[mat].pt for mat in matchList]

        return finalID

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        img2 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ID = self.findID(img2)

        # duration = rospy.Duration.from_sec(5)
        # start_time = rospy.Time.now()

        if ID != 'f1':
            move_turn = Twist()
            move_turn.angular.z = 0.2
            self.move_pub.publish(move_turn)
            
        if self.class_names[ID] == 'f1':
            duration = rospy.Duration.from_sec(5)
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < duration:
                move_msg = Twist()
                move_msg.linear.x = 0.2
                self.move_pub.publish(move_msg)
                rospy.sleep(0.1)
            rospy.sleep(20)
                # if rospy.Time.now() - start_time >= duration:
                #     stop_cmd = Twist()
                #     self.move_pub.publish(stop_cmd) 
                #     break
            stop_cmd = Twist()
            self.move_pub.publish(stop_cmd) 
            cv2.putText(cv_image, self.class_names[ID], (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
                
         

        cv2.imshow('img2', cv_image)
        cv2.waitKey(1)

    def run(self):
        rospy.init_node('image_processing_node')
        image_sub = rospy.Subscriber('camera_topic', Image, self.image_callback)
        rospy.spin()

if __name__ == '__main__':
    game_recognizer = GameRecognizer()
    game_recognizer.run()
