#!/usr/bin/env python2
# coding:utf-8

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import numpy as np

# Scene 1: (-13472.3559, -43879.6831) --> (-13468.4040, -43844.9833)
# Scene 2: (-13518.0648, -43803.0736) --> (-13513.9875, -43773.2982)

# For Tokyo Tech
#s1_right_lane_point1 = [-13472.3559, -43879.6831]
#s1_right_lane_point2 = [-13468.4040, -43844.9833]

# For Futago
s1_right_lane_point1 = [328696.4605, -36671.8093]
s1_right_lane_point2 = [328674.8039, -36829.5759]

s1_left_lane_point1 = [-13467.1769, -43880.2562]
s1_left_lane_point2 = [-13463.2250, -43845.5564]

lane_width = 5.21 

s2_lane_point1 = [-13518.0648, -43803.0736]
s2_lane_point2 = [-13513.9875, -43773.2982]

pub1 = rospy.Publisher('/s1/distance2lane', Float32, queue_size=10)
pub2 = rospy.Publisher('/s2/distance2lane', Float32, queue_size=10)
pub3 = rospy.Publisher('/s1/risk_level', Float32, queue_size=10)
pub4 = rospy.Publisher('/s2/risk_level', Float32, queue_size=10)

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
    s1_distance2lane = distanceCal(point, s1_right_lane_point1, s1_right_lane_point2)
    s2_distance2lane = distanceCal(point, s2_lane_point1, s2_lane_point2)
    pub1.publish(Float32(s1_distance2lane))
    pub2.publish(Float32(s2_distance2lane))

    s1_risk_level = 0.1 / s1_distance2lane
    s2_risk_level = 0.1 / s2_distance2lane

    s1_distance2lane_left = distanceCal(point, s1_left_lane_point1, s1_left_lane_point2)
    if s1_distance2lane_left < lane_width:
        s1_risk_level = 1

    pub3.publish(Float32(s1_risk_level))
    pub4.publish(Float32(s2_risk_level))

def ndt_pose_subscriber():
    rospy.init_node('ndt_pose_subscriber', anonymous=True)
    rospy.Subscriber("/ndt_pose", PoseStamped, poseCallback)
    rospy.spin()

if __name__=='__main__':
    ndt_pose_subscriber()