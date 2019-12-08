import numpy as np
from numpy import sin, cos
from geometry_msgs.msg import Twist,Vector3

class AdaptiveController():
    #implements adaptive controller for ouijabot in 2D manipulation
    #a = [m,J,m*rpx,m*rpy,u1,u1*rix,u1*riy,u1*||ri||,rix,riy]
    def __init__(self):
        ## TODO:
        self.pos_elems = [0,1,4,7]
        pass

    def controllerCallback(self,event):
        """defines a timer callback to implement controller"""
        #define dynamics terms
        dt = event.current_real - event.last_real
        q_err = self.q - self.q_des
        dq_err = self.dq - self.dq_des
        s = dq_err + self.L@dq_err
        dq_r = self.dq_des - self.L@q_err
        ddq_r = self.ddq_des - self.L@dq_err

        #control law
        self.F = self.Y() @ self.a_hat - self.Kd @ s
        self.tau = self.Mhat_inv() @ self.F

        lin_cmd = Vector3(x=self.F[0],y=self.F[1],z=0.)
        ang_cmd = Vector3(x=0.,y=0.,z=self.F[2])
        cmd_msg = Twist(linear=lin_cmd,angular=ang_cmd)
        self.cmd_pub.publish(cmd_msg)

        #adaptation law:
        if np.linalg.norm > self.deadband:
            param_derivative = self.Gamma @ (self.Y()+self.Z()).T @ s
            self.a_hat = self.a_hat - dt*(param_derivative)
            '''TODO: implement Heun's method for integration;
                do projection step here & finish w/next value of s above.'''

        #projection step:
        self.a_hat[self.pos_elems] = np.maximum(self.a_hat[self.pos_elems],0.)

    def stateCallback(self,data):
        ##TODO: measurement callback from Optitrack
        pass

    def refCallback(self,data):
        ##TODO: reference callback (joystick or planned trajectory)
        pass

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



def main():


if __name__ == '__main__':
    main()
