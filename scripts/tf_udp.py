#! /usr/bin/env python
# -*- coding:utf-8 -*-

import socket 
import rospy
from std_msgs.msg import String
from  nav_msgs.msg  import Odometry
import numpy as np
import struct

class OdomUDP():
    def __init__(self, host_ip, host_port,  dest_ip="127.0.0.1", dest_port=5050):
        self.host_ip = host_ip
        self.host_port = host_port
        self.dest_ip = dest_ip
        self.dest_port = dest_port
        
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('',0))
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_socket.settimeout(10)
        # self.udp_socket.bind((self.host_ip, int(self.host_port))) 
        self.data = {
            'time_stamp_secs' : 0,
            'time_stamp_nsecs' : 0,
            "pose_orient":[]
        }


    def publish_lidar_odom(self, msg):
        self.data['time_stamp_secs']= msg.header.stamp.secs
        self.data['time_stamp_nsecs'] = msg.header.stamp.nsecs
        self.data['pose_orient']  = []
        self.data['pose_orient'].append(str(round(msg.pose.pose.position.x, 4)))
        self.data['pose_orient'].append(str(round(msg.pose.pose.position.y, 4)))
        self.data['pose_orient'].append(str(round(msg.pose.pose.position.z, 4)))
        self.data['pose_orient'].append(str(round(msg.pose.pose.orientation.x, 4)))
        self.data['pose_orient'].append(str(round(msg.pose.pose.orientation.y, 4)))
        self.data['pose_orient'].append(str(round(msg.pose.pose.orientation.z, 4)))
        self.data['pose_orient'].append(str(round(msg.pose.pose.orientation.w, 4)))

        msg_time = str(self.data['time_stamp_secs']) + '.' + str(self.data['time_stamp_nsecs'] )
        msg_pose = ','.join( self.data['pose_orient']) 
        udp_msg = msg_time + "," + msg_pose
        # rospy.loginfo("msg_pose:%s", msg_pose)
        self.udp_socket.sendto(udp_msg.encode('ascii'), (self.dest_ip, self.dest_port))

if __name__ ==  "__main__":
    rospy.init_node("publisy_odom_node", anonymous = False)
    rospy.loginfo(
        "Start sending lidar odom data....  "
    )
    odom_pub = OdomUDP(host_ip = "192.168.1.13", host_port=5610, dest_ip="127.0.0.1", dest_port=5050 )
    tf_sub = rospy.Subscriber('/odom',  Odometry,  odom_pub.publish_lidar_odom, \
            queue_size=10, buff_size=52428800)

    while not rospy.is_shutdown():
        rospy.spin()
    
    odom_pub.udp_socket.close()
