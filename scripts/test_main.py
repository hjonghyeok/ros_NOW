#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


pubmsg= Twist()
rospy.loginfo("Starting teleop node")
rospy.init_node('teleop', anonymous=True)
# publisher
publisher = rospy.Publisher('/main', Twist, queue_size=1)

spin_msg = ""


#cam
def process_cam(msg_cam):
    global spin_msg
    spin_msg = msg_cam.data

noise_count = 0
# lidar
def process_lidar(msg_lidar) :
    # motor pub
    global noise_count,spin_msg
    distance = msg_lidar.ranges
    for i in range(0, len(distance), 4):
        degree = np.rad2deg(msg_lidar.angle_min + msg_lidar.angle_increment * i)
        if (-50 < degree < 50) and distance[i] != 0.0:
            if distance[i] < 0.5:
                noise_count += 4
                if noise_count > 20:
                    noise_count = 100
                    if spin_msg == "Right":
                        pubmsg.linear.x = 1
                        pubmsg.angular.z = -1
                        publisher.publish(pubmsg)

                    elif spin_msg == "Left":
                        pubmsg.linear.x = -1
                        pubmsg.angular.z = 1
                        publisher.publish(pubmsg)

                    else :
                        pubmsg.linear.x = 0
                        pubmsg.angular.z = 0
                        publisher.publish(pubmsg)
                        
            else :
                noise_count -= 4
                if noise_count < 10:
                    noise_count = 0
                    if spin_msg == "Right":
                        pubmsg.linear.x = 1
                        pubmsg.angular.z = -1
                        publisher.publish(pubmsg)

                    elif spin_msg == "Left":
                        pubmsg.linear.x = -1
                        pubmsg.angular.z = 1
                        publisher.publish(pubmsg)
                    else :
                        pubmsg.linear.x = 1
                        pubmsg.angular.z = 1
                        publisher.publish(pubmsg)
                    
# lidar sub
sub_scan = rospy.Subscriber('/scan', LaserScan, callback = process_lidar)
sub_cam = rospy.Subscriber('/cam', String, callback = process_cam)
rospy.spin()