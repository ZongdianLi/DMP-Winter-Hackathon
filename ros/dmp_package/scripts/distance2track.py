#!/usr/bin/env python2
# coding:utf-8

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import numpy as np

# track side right: (-13513.5306, -43806.6676) --> (-13510.6027, -43785.5708)
# track side left: (-13512.5401, -43806.8040) --> (-13509.6122, -43785.7072)

field_index = 1 # 0: Tokyo Tech; 1: Futagotamagawa

if field_index == 0: 
    right_track_point1 = [-13513.5306, -43806.6676]
    right_track_point2 = [-13510.6027, -43785.5708]
    left_track_point1 = [-13512.5401, -43806.8040]
    left_track_point2 = [-13509.6122, -43785.7072]

elif field_index == 1:
    right_track_point1 = [328698.1049, -36695.9893]
    right_track_point2 = [328682.5791, -36808.0498]
    left_track_point1 = [328697.1156, -36695.8522]
    left_track_point2 = [328681.5899, -36807.9128]

lane_width = 1

pub1 = rospy.Publisher('/s2/distance2track/left', Float32, queue_size=10)
pub2 = rospy.Publisher('/s2/distance2track/right', Float32, queue_size=10)
pub3 = rospy.Publisher('/s2/risk_level', Float32, queue_size=10)

def distanceCal(point, line_point1, line_point2):
    # Lane line parameters
    A = line_point2[1] - line_point1[1]
    B = line_point1[0] - line_point2[0]
    C = (line_point1[1] - line_point2[1]) * line_point1[0] + \
        (line_point2[0] - line_point1[0]) * line_point1[1]
    # Distance from point to line
    distance = np.abs(A * point[0] + B * point[1] + C) / (np.sqrt(A**2 + B**2)+1e-6)
    return distance

def poseCallback(msg):
    rospy.loginfo("walker pose:x:%0.6f, y:%0.6f", msg.pose.position.x, msg.pose.position.y)
    
    point = [msg.pose.position.x, msg.pose.position.y]
    right_distance2track = distanceCal(point, right_track_point1, right_track_point2)
    left_distance2track = distanceCal(point, left_track_point1, left_track_point2)
    pub1.publish(Float32(right_distance2track))
    pub2.publish(Float32(left_distance2track))

    s2_risk_level = 0.02 / min(right_distance2track, left_distance2track)

    if right_distance2track > lane_width or left_distance2track > lane_width:
        s2_risk_level = 1

    pub3.publish(Float32(s2_risk_level))

def ndt_pose_subscriber():
    rospy.init_node('ndt_pose_subscriber', anonymous=True)
    rospy.Subscriber("/ndt_pose", PoseStamped, poseCallback)
    rospy.spin()

if __name__=='__main__':
    ndt_pose_subscriber()