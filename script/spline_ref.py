#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from hamilton_ac.msg import Reference
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class CircleRef():
    def __init__(self):
        self.active = False
        self.start_time = -1.
        self.T = rospy.get_param('~T')
        self.knots_x = np.fromstring(rospy.get_param('~knots_x'), sep=", ")
        self.knots_y = np.fromstring(rospy.get_param('~knots_y'), sep=", ")
        self.knots_th = np.fromstring(rospy.get_param('~knots_th'), sep=", ")
        self.traj_scale = rospy.get_param('~traj_scale')
        self.pub_freq = rospy.get_param('~pub_freq')

        self.solve_splines()

        self.go_home = False

        self.ref_timer = rospy.Timer(rospy.Duration(1/self.pub_freq),self.ref_callback)
        self.active_sub = rospy.Subscriber('/ac/active',Bool,self.active_callback,queue_size=1)
        self.ref_pub = rospy.Publisher('/ac/ref',Reference,queue_size=1)
        self.joy_sub = rospy.Subscriber('/joy',Joy,self.joy_callback,queue_size=1)
        print('setup finished')

    def solve_splines(self):
        n = len(self.knots_x)
        rospy.logwarn(self.knots_x)
        regressor_mat = np.zeros((4*n,4*n))
        regressor_vec_x = np.zeros(4*n)
        regressor_vec_y = np.zeros(4*n)
        regressor_vec_th = np.zeros(4*n)
        #populate constraint matrix for i = 0:n-1
        for i in range(0,n-1):
            regressor_mat[4*i,4*i] = 1 #a_i
            regressor_mat[4*i+1,4*(i+1)] = 1 #a_(i+1)
            regressor_mat[4*i+1,4*i:4*(i+1)] = [-1,-1,-1,-1] #[a_i,b_i,c_i,d_i]
            regressor_mat[4*i+2,4*(i+1)+1] = 1 #b_(i+1)
            regressor_mat[4*i+2,4*i:4*(i+1)] = [0,-1,-2,-3] #[a_i,b_i,c_i,d_i]
            regressor_mat[4*i+3,4*(i+1)+2] = 2 #c_(i+1)
            regressor_mat[4*i+3,4*i:4*(i+1)] = [0,0,-2,-6] #[a_i,b_i,c_i,d_i]
            regressor_vec_x[4*i] = self.knots_x[i]
            regressor_vec_y[4*i] = self.knots_y[i]
            regressor_vec_th[4*i] = self.knots_th[i]

        regressor_mat[4*(n-1),4*(n-1)] = 1
        regressor_mat[4*(n-1)+1,0] = 1 #a_0
        regressor_mat[4*(n-1)+1,4*(n-1):] = [-1,-1,-1,-1]
        regressor_mat[4*(n-1)+2,1] = 1 #b_0
        regressor_mat[4*(n-1)+2,4*(n-1):] = [0,-1,-2,-3]
        regressor_mat[4*(n-1)+3,2] = 2 #c_0
        regressor_mat[4*(n-1)+3,4*(n-1):] = [0,0,-2,-6]
        regressor_vec_x[4*(n-1)] = self.knots_x[-1]
        regressor_vec_y[4*(n-1)] = self.knots_y[-1]
        regressor_vec_th[4*(n-1)] = self.knots_th[-1]

        self.coeffs_x = np.linalg.inv(regressor_mat)@regressor_vec_x
        self.coeffs_y = np.linalg.inv(regressor_mat)@regressor_vec_y
        self.coeffs_th = np.linalg.inv(regressor_mat)@regressor_vec_th

    def spline_eval(self,t):
        t = t % self.T #mod for overall cycle length
        q, dq, ddq = np.zeros(3), np.zeros(3), np.zeros(3)
        coeffs_list = [self.coeffs_x, self.coeffs_y, self.coeffs_th]

        n = int(len(self.coeffs_x)/4) #count number of segments in spline
        seg_len = self.T/n #get relative length of spline
        for ii in range(0,n): #evaluate which spline segment you're in
            if t < seg_len*(ii+1):
                ind = ii
                break

        tau = (t - ind*seg_len)/seg_len #change variables so tau in [0,1]

        for i in range(0,len(coeffs_list)):
            coeffs = coeffs_list[i]
            seg_coeffs = coeffs[4*ind:4*(ind+1)]

            #generate tau arrays for each derivative (0th, 1st, 2nd)
            t0 = np.array([1,tau,np.power(tau,2),np.power(tau,3)])
            t1 = (1/seg_len)*np.array([0,1,2*tau,3*np.power(tau,2)])
            t2 = np.power(1/seg_len,2)*np.array([0,0,2,6*tau])

            q[i] = np.dot(t0,seg_coeffs)
            dq[i] = np.dot(t1,seg_coeffs)
            ddq[i] = np.dot(t2,seg_coeffs)

        return q,dq,ddq

    def ref_callback(self,event):
        if not self.active:
            return

        if self.go_home:
            q_des, dq_des, ddq_des = Vector3(), Vector3(), Vector3()
        else:
            t = event.current_real.to_sec() - self.start_time
            q_d, dq_d, ddq_d = self.spline_eval(t)

            q_des = Vector3(q_d[0],q_d[1],q_d[2])
            dq_des = Vector3(dq_d[0],dq_d[1],dq_d[2])
            ddq_des = Vector3(ddq_d[0],ddq_d[1],ddq_d[2])

        msg = Reference(q_des,dq_des,ddq_des)
        self.ref_pub.publish(msg)

    def active_callback(self,msg):
        if self.active and not msg.data: #turn off
            self.active = False
            self.time = -1.
        elif not self.active and msg.data: #turn on
            self.active = True
            self.start_time = rospy.Time.now().to_sec()

    def joy_callback(self,msg):
        if msg.axes[3] < 0:
            self.go_home = True
        else:
            self.go_home = False

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
