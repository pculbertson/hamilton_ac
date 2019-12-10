#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from hamilton_ac.msg import Reference
from std_msgs.msg import Bool

class CircleRef():
    def __init__(self):
        self.active = False
        self.start_time = -1.
        self.R = rospy.get_param('/ref/R')
        self.T = rospy.get_param('/ref/T')
        self.c_x = rospy.get_param('/ref/c_x')
        self.c_y = rospy.get_param('/ref/c_y')
        self.c = np.array([self.c_x,self.c_y])
        self.rot = rospy.get_param('/ref/rot')
        self.max_angle = rospy.get_param('/ref/max_angle')
        self.pub_freq = rospy.get_param('/ref/pub_freq')
        self.w = 2*np.pi/self.T

        self.ref_timer = rospy.Timer(rospy.Duration(1/self.pub_freq),self.ref_callback)
        self.active_sub = rospy.Subscriber('/ac/active',Bool,self.active_callback,queue_size=1)
        self.ref_pub = rospy.Publisher('/ac/ref',Reference,queue_size=1)
        print('setup finished')

    def ref_callback(self,event):
        if not self.active:
            return

        t = event.current_real.to_sec() - self.start_time
        q_x = self.c_x + self.R*np.cos(self.w*t)
        q_y = self.c_y + self.R*np.sin(self.w*t)
        dq_x = -self.w*self.R*np.sin(self.w*t)
        dq_y = self.w*self.R*np.cos(self.w*t)
        ddq_x = -(self.w**2)*self.R*np.cos(self.w*t)
        ddq_y = -(self.w**2)*self.R*np.sin(self.w*t)

        if self.rot:
            q_t = -(self.max_angle/2)*(np.cos(self.w*t)+1)
            dq_t = self.w*(self.max_angle/2)*np.sin(self.w*t)
            ddq_t = (self.w**2)*(self.max_angle/2)*np.cos(self.w*t)
        else:
            q_t = dq_t = ddq_t = 0.

        q_des = Vector3(q_x,q_y,q_t)
        dq_des = Vector3(dq_x,dq_y,dq_t)
        ddq_des = Vector3(ddq_x,ddq_y,ddq_t)

        msg = Reference(q_des,dq_des,ddq_des)
        self.ref_pub.publish(msg)

    def active_callback(self,msg):
        print(msg.data)
        if self.active and not msg.data: #turn off
            self.active = False
            self.time = -1.
        elif not self.active and msg.data: #turn on
            self.active = True
            self.time = rospy.Time.now().to_sec()

def main():
    rospy.init_node("circle_ref")
    try:
        CircleRef()
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logwarn('closing reference')
        raise e

if __name__ == "__main__":
    main()
