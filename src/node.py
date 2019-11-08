#!/usr/bin/python

import numpy as np
from datetime import datetime
from math import pi, cos, sin
#from __future__ import print_function
from dronekit import connect, VehicleMode
from vehicle_connector import VehicleConnector
import time

import tf
import rospy
from gps_driver.msg import GPSQual
from std_msgs.msg import Float64
from gps_common.msg import GPSFix
from geometry_msgs.msg import Quaternion, Twist

class Node:

    def __init__(self):

        rospy.init_node("GPS")

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Starting node...")

        self.rate = 1.0

	    self.heading = 0.0

	    self.latitude = 0.0

	    self.longitude = 0.0

        self.pub_gps_fix = rospy.Publisher('gps_fix', GPSFix, queue_size = 1)

	    self.sub_gps_navsat = rospy.Subscriber('gps_navsat', NavSatFix, self.gps_navsat_callback)

	    self.sub_odom = rospy.Subscriber('odom_inertial', Odometry, self.odom_callback)
        
        self.gps_msg = GPSFix()


    def run(self):

        rospy.loginfo("Starting GPS broadcast")

        r_time = rospy.Rate(self.rate)

        while not rospy.is_shutdown():

            # building the GPS Fix message

            self.gps_msg.header.stamp = rospy.Time.now()

            self.gps_msg.header.frame_id = 'base_link'   

            self.gps_msg.latitude = self.latitude
   
            self.gps_msg.longitude = self.longitude
   
            self.gps_msg.dip = self.heading

            # publishing            
            self.pub_gps_fix.publish(self.gps_msg)

            r_time.sleep()

    def odom_callback(self, msg):

        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        self.theta = y

    def 

    def shutdown(self):

        pass
    
        
if __name__ == "__main__":

    try:

        node = Node()

        node.run()

    except rospy.ROSInterruptException:

        pass

    rospy.loginfo("Exiting")
