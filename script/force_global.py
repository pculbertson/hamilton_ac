#!/usr/bin/env python
import numpy as np
import rospy
from numpy import sin, cos
from geometry_msgs.msg import Twist,Vector3,PoseStamped
from hamilton_ac.msg import Reference

class ForceGlobal():
    """Translates global payload wrench into local frame"""
    def __init__(self):
        self.theta = 0.
        self.tau = np.zeros(3)
        self.offset_angle = rospy.get_param('offset_angle',0.) #angle offset
            #from payload frame, default to zero

        self.cmd_pub = rospy.Publisher('cmd_wrench',Twist,queue_size=1)
	self.state_sub = rospy.Subscriber('state',PoseStamped,
            self.stateCallback)

        self.wrench_sub = rospy.Subscriber('/fc/cmd_wrench',Twist,self.refCallback)
        self.cmd_timer = rospy.Timer(rospy.Duration(0.1),
            self.controllerCallback)


    def controllerCallback(self,event):
        """defines a timer callback to implement controller"""
        #define dynamics terms
        rot_mat = world_to_body(self.theta+self.offset_angle)
        tau_body = np.matmul(rot_mat.T, self.tau)

        lin_cmd = Vector3(x=tau_body[0],y=tau_body[1],z=0.)
        ang_cmd = Vector3(x=0.,y=0.,z=tau_body[2])
        cmd_msg = Twist(linear=lin_cmd,angular=ang_cmd)
        self.cmd_pub.publish(cmd_msg)

    def stateCallback(self,data):
        '''handles measurement callback from Optitrack'''
        self.theta = quaternion_to_angle(data.pose.orientation)

    def refCallback(self,data):
        self.tau = np.array([data.linear.x,data.linear.y,data.angular.z])

def quaternion_to_angle(q):
    #assuming the axis is always standing straight up
    # TODO(Preston): check this is actually correct!
    cos_th = 2*(q.x**2 + q.w**2)-1
    sin_th = -2*(q.x*q.y - q.z*q.w)
    return np.arctan2(sin_th,cos_th)

def world_to_body(angle):
    """returns world-to-body rotation matrix"""
    return np.array([[cos(angle),-sin(angle),0.],[sin(angle),cos(angle),0.],
        [0.,0.,1.]])

def main():
    rospy.init_node('force_global')
    try:
        ForceGlobal()
        rospy.logwarn('starting force global')
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logwarn('closing force global')
        raise e

if __name__ == '__main__':
    main()
