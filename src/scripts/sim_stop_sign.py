#!/usr/bin/env python3
import rospy
import time
import roslaunch
import numpy as np
from std_msgs.msg import Int16, Int8
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes

class stopsign:
    def __init__(self):
        print("Initialized")
        rospy.init_node('stopsignprobability', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Publishes when stop sign is detected
        self.stop_sign_detect_pub = rospy.Publisher('/detect_stop', Int16, queue_size=10)
        # Subscribes to the bounding box messages
        self.yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.newprediction)
        # Subscribes to the detect_line topic which will indicate when a line is detectied
        self.detect_line_sub = rospy.Subscriber("/detect_line",Int16,self.line_detection)
        # This is not working currently
        self.object_detect_sub = rospy.Subscriber("/darknet_ros/found_object/count", Int8, self.object_counter)
        self.rate = rospy.Rate(10)

    # This is not working currently
    def object_counter(self, msg):
        #print(msg)
        #global object_counter 
        #print(msg)
        if msg == 0:
            stop_sign_detect = 0
            self.stop_sign_detect_pub.publish(stop_sign_detect)

    # Callback function from detect_line_sub which assigns the line_detection variable
    def line_detection(self,msg):
        global line_detection
        line_detection = msg.data

    # Recieves the bounding_boxes messages and processes it to identify stop sign detection
    def newprediction(self,bounding_box):
        print("Callback working")
        self.rate.sleep()
        # variable that records stop sign detection
        global stop_sign_detect
        #print("Function executing")
        # All the predictions detected by YOLO
        prediction = bounding_box.bounding_boxes
        
        # Loop over all the predictions and detect the bounding boxes that are stop signs
        for box in prediction:
            identified_class=box.Class
            probability = float(box.probability)
            #area = abs(box.xmax-box.xmin)*abs(box.ymax-box.ymin)
            # If the identified class is the stop-sign with 0.2 or higher probabilty then stop after some delay
            if ((identified_class == 'stop sign') and (probability >= 0.2)): #and (area >=3000)):        #change based on the calibration
                #print("Stop sign detected")
                now = time.time()
                diff=0
                # Creating the delay
                while diff<7:
                    #print("Stop sign detected")
                    current = time.time()
                    diff = current - now
                stop_sign_detect = 1
                self.stop_sign_detect_pub.publish(stop_sign_detect)
            #else:
             #   self.stop_sign_detect_pub.publish(0)
                break
            rospy.sleep(0.1)
# Initialization
# Initializing the class
stopsign()
vel_msg = Twist()
line_detection = 0
stop_sign_detect = 0
object_counter = 0
while True:
    x = 1

rospy.spin()
    #print(stop_sign_detect)
    # does nothing until line is detected
  #  if line_detection ==1:
   #     stopsign()
        #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)
        #launch = roslaunch.parent.ROSLaunchParent(uuid,["~/catkin_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"])
        #launch.start()
     #   while stop_sign_detect==0:
    #        rospy.sleep(0.1)
      #      stopsign()
    #if stop_sign_detect==1:
     #  vel_msg.linear.x = 0
      # vel_msg.linear.y = 0
       #vel_msg.linear.z = 0
    #   vel_msg.angular.x = 0
     #  vel_msg.angular.y = 0
      # vel_msg.angular.z = 0
    #   self.velocity_publisher.publish(vel_msg)
     #  rospy.sleep(3)