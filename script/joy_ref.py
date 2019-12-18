#!/usr/bin/env python
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from hamilton_ac.msg import Reference
import rospy
import numpy as np

class JoyReference():
    """node to generate reference trajectories for ac experiments"""
    def __init__(self):
        self.ref_pub = rospy.Publisher('/ac/ref',Reference,queue_size=1)
        self.max_lin_vel = rospy.get_param('~max_lin_vel')
        self.max_ang_vel = rospy.get_param('~max_ang_vel')
        self.max_vel = np.array([self.max_lin_vel,self.max_lin_vel,
            self.max_ang_vel])
        self.lin_accel_scale = rospy.get_param('~lin_accel_scale')
        self.ang_accel_scale = rospy.get_param('~ang_accel_scale')
        self.vel_decay = rospy.get_param('~vel_decay')
        self.cmd_time = -1.
        self.max_delay = rospy.get_param('~max_delay')
        self.dq_des = self.ddq_des = np.array([0.,0.,0.])
        self.active = False

        self.ref_timer = rospy.Timer(rospy.Duration(1./10),self.ref_callback)
        self.joy_sub = rospy.Subscriber('/joy',Joy,self.joy_callback,
            queue_size=1)
        self.active_sub = rospy.Subscriber('/ac/active',Bool, self.active_callback, queue_size=1)

    def joy_callback(self,data):
        ddx_des = -self.lin_accel_scale * data.axes[0] #x-axis backwards
        ddy_des = self.lin_accel_scale * data.axes[1]
        ddth_des = self.ang_accel_scale * data.axes[2]
        self.ddq_des = np.array([ddx_des,ddy_des,ddth_des])

    def active_callback(self,msg):
        self.active = msg.data

    def ref_callback(self,event):
        dt = event.current_real.to_sec() - self.cmd_time
        if self.cmd_time == -1.:
            self.cmd_time = event.current_real.to_sec()
            return
        elif dt > self.max_delay or not self.active:
            self.cmd_time = -1.
            self.dq_des = self.ddq_des = np.array([0.,0.,0.])
        else:
            self.dq_des = self.dq_des + dt*(self.ddq_des
                - self.vel_decay*self.dq_des)
            self.dq_des = np.clip(self.dq_des,-self.max_vel,self.max_vel)

        q_msg = Vector3(0.,0.,0.) #q_des not used w/ Joystick
        dq_msg = Vector3(*self.dq_des)
        ddq_msg = Vector3(*self.ddq_des)
        msg = Reference(q_msg,dq_msg,ddq_msg)

        self.ref_pub.publish(msg)
        self.cmd_time = event.current_real.to_sec()

def main():
    rospy.init_node('joy_reference')
    try:
        JoyReference()
        rospy.spin()
    except rospy.ROSException as e:
        print('closing joy ref')
        raise e

if __name__ == "__main__":
    main()
