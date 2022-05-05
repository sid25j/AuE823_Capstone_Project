#!/usr/bin/env python3
import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
#from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import LaserScan


class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.detect_line_publisher = rospy.Publisher('/detect_line', Int16, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.camera_callback)
        self.tag_detection_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        #self.image_sub = rospy.Subscriber("/camera/image/compressed",CompressedImage,self.camera_callback)
        self.stop_sign_detect = rospy.Subscriber("/detect_stop",Int16,self.stop_detection)
        #self.stop_sign_detect_pub = rospy.Publisher('/detect_stop', Int16, queue_size=10)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_data)
        self.switch = 0
        self.delta = 0
        self.flag = 0 
        self.end_of_lane = 1
        self.tag_detected = 0

        #self.tag_detection_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        # Publish when a tag is detected
        #self.tag_pub = rospy.Publisher('/detect_apriltag', Int8, queue_size = 10)
        self.angular = 0
        self.linear = 0.06
        self.rate = rospy.Rate(10)

    def stop_detection(self,msg):
        global stop_detected
        #stop_detected_old = stop_detected
        #if self.flag != 1:
        stop_detected = msg.data
        #self.delta = stop_detected_old - stop_dete
        cted

    def tag_callback(self,data):
        """Callback function which is called when a tag_detection is
        received by the subscriber."""
        ax = []
        if len(data.detections) > 0:
            print("detected the tag")
            data_received = data.detections[0].pose.pose.pose.position 
            self.angular = data_received.x    # helps with angular velocity
            self.linear = data_received.z # helps with linear velocity
            # If april tag is detected
            #self.tag_pub.publish(1)
            self.tag_detected = 1
        else:
            # If april tag is not detected
            #self.tag_pub.publish(0)
            self.linear = 0
            self.angular = 0
            self.tag_detected = 0
        
        self.linear = round(self.linear, 4)
        self.angular = round(self.angular, 4)

    def move2tag(self):
        vel_msg = Twist()
        
        while self.tag_detected == 1:
            if self.linear > 0.05:
                print("TagDetected")
                # Porportional controller.
                # Linear velocity in the x-axis.
                vel_msg.linear.x = 0.06
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = -min(self.angular, 0.2)

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                # Publish at the desired rate.
                self.rate.sleep()
            else:
                self.tag_detected == 0
                self.angular = 0
                self.linear = 0.06



        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()


    def scan_data(self,distance):
        #Create Twist message object for velocities
        vel_msg = Twist()
        #Laser scan threshold for obstacles
        #threshold_dist = distance.range_max/2
        threshold_dist = 0.13 #Approximatley 45cm
        rate1 = rospy.Rate(10)
        #Publisher Object
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 100)
        
        # Controller
        #print(self.switch)
        if self.switch == 0 and self.tag_detected == 0:
            self.end_of_lane = 1
            distance.ranges = [x if x>0 else 3.5 for x in distance.ranges]
            left = distance.ranges[25:110]
            right = distance.ranges[250:335]
            front = distance.ranges[0:25] + distance.ranges[335:360]
            #front = [i for i in front if i !="inf"]
            front = min(front)
            #print("front: ", front)
            avg_left = np.mean(sorted(left)[:5])
            #print("average left: ", avg_left)
            avg_right = np.mean(sorted(right)[:5])
            #print("average right: ", avg_right)
            #front_left and front_right is used with the p-controller to control angular velocity
            front_left = np.min(distance.ranges[25:50])
            front_right = np.min(distance.ranges[310:335])
            #print("Obstacle avoidance")
            if front > threshold_dist and front_left > threshold_dist and front_right > threshold_dist:
                # if every range is above threshold
                vel_msg.linear.x = front/7.5
                vel_msg.angular.z = (front_left - front_right)*0.55
            elif front > threshold_dist and front_left > threshold_dist and front_right <threshold_dist:
                # if front_right is below threshold
                vel_msg.linear.x = front/10
                vel_msg.angular.z = (front_left - front_right)*0.75
            elif front > threshold_dist and front_left < threshold_dist and front_right > threshold_dist:
                # if front_left is below threshold
                vel_msg.linear.x = front/10
                vel_msg.angular.z = (front_left - front_right)*0.75
            elif front < threshold_dist:
                # if front distance is below threshold, use a backing maneuver
                vel_msg.linear.x = -front/0.5
                if avg_left > avg_right:
                    vel_msg.angular.z = 0.8
                elif (avg_right - avg_left) > 0.05:
                    vel_msg.angular.z = -0.8
                else:
                    vel_msg.angular.z = 0.8

                rate1.sleep()

            #if front > 1.5:
             #   vel_msg.linear.x = 0.1
                
            pub.publish(vel_msg)
            vel_msg.linear.x = 0.05
            vel_msg.angular.z = 0.0

        elif self.switch == 0 and self.tag_detected == 1:
            self.move2tag()


    def camera_callback(self, data):
        global stop_detected
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #np_arr = np.fromstring(data.data, np.uint8)
        #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        crop_start=(height/2)+200
        crop_end=(height/2)+240
        #crop_start=450
        #crop_end=500
        # crop_start=(height/2)+200
        # crop_end=(height/2)+400
        crop_img = cv_image[int(crop_start):int(crop_end)][1:int(width)]
        #crop_img = cv_image[340:360][1:640]

        # photo center
        centerx, centery = width/2, ((crop_start+crop_end)/2-crop_start)

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV.
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([25,3,100])
        upper_yellow = np.array([85,100,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        global line_detection
        line_detection=0

        if m['m00'] == 0:
            self.switch = 0
        else:
            self.switch = 1

        #print(self.switch)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            line_detection=1
            self.detect_line_publisher.publish(line_detection)             # to let other nodes know that line is detected
        except ZeroDivisionError:
            cx, cy = 10000, 10000
            self.detect_line_publisher.publish(line_detection)             # to let other nodes know that line is not detected


        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        vel_msg = Twist()

        #calculating the difference between the center of the image and the centroid
        diff_y= centery-cy
        diff_x= centerx-cx

        # Proportional controller value calculation for linear speed
        if self.switch == 1:
            self.end_of_lane = 0
            self.velocity_publisher.publish(vel_msg)
            #print("Line following")
            if diff_y < -8000 :
                vel_msg.linear.x = 0
            else:
                # vel_msg.linear.x = 0
                vel_msg.linear.x = 0.1


            # Proportional controller value calculation for angular speed
            if diff_x < -8000 :
                vel_msg.angular.z = 0
            else:
                # vel_msg.angular.z = 0
                vel_msg.angular.z = 0.3*diff_x/100

            # print("centerioid",cx,cy)
            # print("center",centerx,centery)
            # print(vel_msg.linear.x)

            global stop_detected
            if ((line_detection==1) and (stop_detected==0)):
                #vel_msg.linear.y = 0
                #vel_msg.linear.z = 0
                #vel_msg.angular.x = 0
                #vel_msg.angular.y = 0
                self.velocity_publisher.publish(vel_msg)
            elif stop_detected==1:
                vel_msg.linear.x = 0
                #vel_msg.linear.y = 0
                #vel_msg.linear.z = 0
                #vel_msg.angular.x = 0
                #vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)

                #stop for 3 seconds at stop sign
                now = time.time()
                diff=0
                while diff<3:
                    current = time.time()
                    diff = current - now
                stop_detected=0
                #self.flag = 1
                self.stop_sign_detect_pub.publish(stop_detected)
        else:
            self.end_of_lane = 1
            

    def clean_up(self):
        cv2.destroyAllWindows()
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

def main():
    rospy.init_node('line_following_node', anonymous=True)
    #stop_detected =0
    line_follower_object = LineFollower()
    rate = rospy.Rate(10)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

stop_detected =0
if __name__ == '__main__':
        main()
