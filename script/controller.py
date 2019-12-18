#!/usr/bin/env python
import numpy as np
from numpy import sin, cos
from geometry_msgs.msg import Twist,Vector3,PoseStamped
from hamilton_ac.msg import Reference

class AdaptiveController():
    #implements adaptive controller for ouijabot in 2D manipulation
    #a = [m,J,m*rpx,m*rpy,u1,u1*rix,u1*riy,u1*||ri||,rix,riy]
    def __init__(self):
        self.getParams()
        self.active = False
        self.controllerReset()
        self.q_des, self.dq_des = np.zeros(3), np.zeros(3)
        self.ddq_des = np.zeros(3)

        self.cmd_pub = rospy.Publisher('cmd_wrench',Twist)
        self.state_sub = rospy.Subscriber('state',PoseStamped,
            self.stateCallback)

        self.ref_sub = rospy.Subscriber('/ac/ref',Reference,self.refCallback)
        self.cmd_timer = rospy.Timer(rospy.Duration(0.1),
            self.controllerCallback)
        self.active_sub = rospy.Subscriber('/ac/active', Bool, self.activeCallback)


    def controllerReset(self):
        self.q = np.zeros(3)
        self.dq = np.zeros(3)
        self.q_prev = np.zeros(3) #used for backwards difference
        self.tau, self.F = np.zeros(3), np.zeros(3)
        self.a_hat = np.zeros(10)

    def getParams(self):
        self.L = rospy.get_param('/ac/L')*np.eye(3)
        self.Kd = rospy.get_param('/ac/Kd')*np.eye(3)
        self.Gamma = rospy.get_param('/ac/Gamma')*np.eye(3)
        self.pos_elems = [0,1,4,7] #flags which elements to project to >0
        self.q_filt = rospy.get_param('/ac/q_filt')
        self.dq_filt = rospy.get_param('/ac/dq_filt')
        self.offset_angle = rospy.get_param('offset_angle','0.') #angle offset
            #from payload frame, default to zero

    def activeCallback(self,msg):
        if not self.active and msg.data:
            self.getParams()
        else if self.active and not msg.data:
            self.controllerReset()

        self.active = msg.data

    def controllerCallback(self,event):
        """defines a timer callback to implement controller"""
        #define dynamics terms
        if not self.active:
            return

        dt = event.current_real - event.last_real
        q_err = self.q - self.q_des
        dq_err = self.dq - self.dq_des
        s = dq_err + self.L@dq_err
        dq_r = self.dq_des - self.L@q_err
        ddq_r = self.ddq_des - self.L@dq_err

        #control law
        self.F = self.Y() @ self.a_hat - self.Kd @ s #world frame
        self.tau = self.Mhat_inv() @ self.F #world frame

        rot_mat = world_to_body(self.q[-1]+self.offset_angle)
        self.tau_body = rot_mat @ self.tau

        lin_cmd = Vector3(x=self.tau_body[0],y=self.tau_body[1],z=0.)
        ang_cmd = Vector3(x=0.,y=0.,z=self.tau_body[2])
        cmd_msg = Twist(linear=lin_cmd,angular=ang_cmd)
        self.cmd_pub.publish(cmd_msg)

        #adaptation law:
        if np.linalg.norm > self.deadband:
            param_derivative = self.Gamma @ (self.Y()+self.Z()).T @ s
            self.a_hat = self.a_hat - dt*(param_derivative)
            '''#TODO(Preston): implement Heun's method for integration;
                do projection step here & finish w/next value of s above.'''

        #projection step:
        self.a_hat[self.pos_elems] = np.maximum(self.a_hat[self.pos_elems],0.)

    def stateCallback(self,data):
        '''handles measurement callback from Optitrack'''
        if self.state_time == -1:
            self.state_time = data.header.stamp.to_sec()
        else:
            dt = data.header.stamp.to_sec() - self.state_time
            th = quaternion_to_angle(data.orientation)
            q_new = np.array([data.position.x,data.position.y,th])
            q_smoothed = (1-self.q_filt)*q_new + self.q_filt*self.q

            dq_new = (3*q_smoothed - 4*self.q + self.q_prev)/(2*dt)

            self.q_prev = self.q
            self.q = q_smoothed
            self.dq = (1-self.dq_filt)*dq_new + self.dq_filt*self.dq
            self.state_time= data.header.stamp.to_sec()

    def refCallback(self,data):
        self.q_des = np.array([data.q.x,data.q.y,data.q.z])
        self.dq_des = np.array([data.dq.x,data.dq.y,data.dq.z])
        self.ddq_des = np.array([data.ddq.x,data.ddq.y,data.ddq.z])

    def Mhat_inv(self):
        """defines correction term for moment arms in control law"""
        rhx, rhy = self.a_hat[-2:]
        _, _, th = self.q

        rhx_n = rhx*cos(th)+rhy*sin(th)
        rhy_n = -rhx*sin(th)+rhy*cos(th)

        return np.array([[1,0,0],[0,1,0],[rhy_n,-rhx_n,1])

    def Y(self):
        """Y*a = H*ddqr + (C+D)dqr"""
        x, y, th = self.q
        dx, dy, dth = self.dq
        dxr, dyr, dthr = self.dqr
        ddxr, ddyr, ddthr = self.ddqr
        block1h = np.array([[ddxr,0,-sin(th)*ddthr, cos(th)*ddthr],
            [ddyr,0,-cos(th)*ddthr,-sin(th)*ddthr],
            [0,ddthr,-sin(th)*ddxr-cos(th)*ddyr,cos(th)*ddxr-sin(th)*ddyr]])
        block1c = np.array([[0,0,dth*dthr,0],[0,0,0,dth*dthr],[0,0,0,0]])
        block2 = np.array([[dxr,sin(th)*dthr,-cos(th)*dthr,0],
            [dyr,cos(th)*dthr,sin(th)*dthr,0],
            [0,sin(th)*dxr+cos(th)*dyr,-cos(th)*dxr+sin(th)*dyr,dthr]])
        Y = np.concatenate((block1h+block1c,block2,np.zeros((3,2))),axis=1)
        return Y

    def Z(self):
        """F + Z(q,F)@(ahat-a) = G*inv(Ghat)*F"""
        _ ,_ ,th = self.q
        Fx, Fy, _ = self.F
        block = np.array([[0,0],[0,0],
            [-(sin(th)*Fx+cos(th)*Fy),cos(th)*Fx-sin(th)*Fy]])
        return np.concatenate((np.zeros((3,8)),block),axis=1)

def quaternion_to_angle(quaternion):
    #assuming the axis is always standing straight up
    # TODO(Preston): check this is actually correct!
    return 2*np.arccos(quaternion.w)

def world_to_body(angle):
    """returns world-to-body rotation matrix"""
    return np.array([cos(angle),-sin(angle),0.],[sin(angle),cos(angle),0.],
        [0.,0.,1.])

def main():
    rospy.init_node('hamilton_ac')
    try:
        AdaptiveController()
        rospy.logwarn('starting ac')
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logwarn('closing ac')
        raise e

if __name__ == '__main__':
    main()
