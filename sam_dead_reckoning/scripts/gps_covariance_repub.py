#! /usr/bin/env python

#Wrapper node to read gps inputs and republish after changing covariance 
 
import rospy
from nav_msgs.msg import Odometry

class GPSRepub(object):

    def gps_cb(self,gps_odom):
        self.gps_odom = gps_odom
        self.gps_odom.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gps_odom.pose.covariance[0] = 40.0
        self.gps_odom.pose.covariance[7] = 40.0
        self.gps_odom.pose.covariance[14] = 0.003
        self.gps_odom.pose.covariance[21] = 0.1 # 10.
        self.gps_odom.pose.covariance[28] = 0.1 # 10.
        self.gps_odom.pose.covariance[35] = .5
        rospy.loginfo_throttle(1,'Republishing GPS covariance')
        self.gps_pub.publish(self.gps_odom)

    def __init__(self, name):

        gps_odom_topic = rospy.get_param('~gps_odom_topic', '/sam/dr/odometry/gps')
        new_gps_odom_topic =  rospy.get_param('~new_gps_odom_topic', '/sam/dr/odometry/gps_new')
        #self.loop_freq = rospy.get_param("~loop_freq", 21)
        self.gps_sub = rospy.Subscriber(gps_odom_topic, Odometry, self.gps_cb)
        self.gps_pub = rospy.Publisher(new_gps_odom_topic, Odometry , queue_size=10)

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("gps_repub")
    gps_repub_obj = GPSRepub(rospy.get_name())