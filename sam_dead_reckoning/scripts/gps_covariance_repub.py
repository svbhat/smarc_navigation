#! /usr/bin/env python

#Wrapper node to read control inputs and republish at above 10hz 

import rospy
from nav_msgs.msg import Odometry

class GPSRepub(object):

    def gps_cb(self,gps_odom):
        self.gps_odom = gps_odom

    def __init__(self, name):
        gps_odom_topic = rospy.get_param('~gps_odom_topic', '/sam/dr/odometry/gps')
        new_gps_odom_topic =  rospy.get_param('~new_gps_odom_topic', '/sam/dr/odometry/gps_new')
        
        self.loop_freq = rospy.get_param("~loop_freq", 21)

        self.gps_sub = rospy.Subscriber(gps_odom_topic, Odometry, self.gps_cb)

        self.gps_pub = rospy.Publisher(new_gps_odom_topic, Odometry , queue_size=10)
        

        self.rate = rospy.Rate(self.loop_freq) 

        #initialize actuator commands
        self.gps_odom = Odometry()
        
        while not rospy.is_shutdown():

            
            self.gps_odom.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.gps_odom.pose.covariance[0] = 40.0
            self.gps_odom.pose.covariance[7] = 40.0 
            self.gps_odom.pose.covariance[14] = 0.003
            self.gps_odom.pose.covariance[21] = 0.1 # 10.
            self.gps_odom.pose.covariance[28] = 0.1 # 10.
            self.gps_odom.pose.covariance[35] = .5

            rospy.loginfo_throttle(1,'Republishing GPS covariance')
    
            self.gps_pub.publish(self.gps_odom)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rpm_repub")
    rpm_repub_obj = GPSRepub(rospy.get_name())

