#!/usr/bin/env python  
import rospy
import math

from tr5_kinematics.srv import DoForwardKinematics, DoForwardKinematicsResponse
from tr5_kinematics.srv import DoInverseKinematics, DoInverseKinematicsResponse
# load private parameters
svc_fk = rospy.get_param("~for_kin_service", "/do_fk")
svc_ik = rospy.get_param("~inv_kin_service", "/do_ik")     

# DH Parameters for ROB3/TR5
d1 = 0.275
a2 = 0.200
a3 = 0.130
d5 = 0.130


def doForwardKinematics(srv):

    return

"""
TODO Implement the callback to handle the inverse kinematics calculations 
"""
def doInverseKinematics(srv):
    return 

if __name__ == "__main__":
    rospy.init_node('tr5_kinematics')
    srv_forward_kinematics = rospy.Service("do_fk", DoForwardKinematics, doForwardKinematics)
    srv_inverse_kinematics = rospy.Service("do_ik", DoInverseKinematics, doInverseKinematics)

    while not rospy.is_shutdown():
        rospy.spin()
    