
from os import umask
import numpy as np
import general_robotics_toolbox as rox
from numpy.core.fromnumeric import size
from numpy.core.records import array
import quadprog as qp
from math import pi, sin,cos,atan2
from scipy.linalg import norm

ex = np.array([1,0,0])
ey = np.array([0,1,0])
ez = np.array([0,0,1])

class Oarbot(object):
    def __init__(self) -> None:
        super().__init__()

        # robot parameters
        wheel_r,d1 = 0.127, 0.2755
        l1,l2 = 0.3, wheel_r+0.3+d1
        d2, d3, e2, d4, d5, d6= 0.41, 0.2073, 0.0098, 0.0741, 0.0741, 0.16
        aa = pi/6
        p89x = d4*(sin(aa)/sin(2*aa)) + 2*d4*(sin(aa)/sin(2*aa))*cos(2*aa)
        p89y = 2*d4*sin(aa)
        H = np.array([ex,ey,ez,ez,ez,ey,-ey,-ex,-sin(pi/6)*ex+cos(pi/6)*ey,-ex]).T
        P = np.array([0*ex,0*ex,0*ex,l1*ex+l2*ez,0*ex,0*ex,d2*ez,d3*ex+e2*ey,p89x*ex-p89y*ey,0*ex,(d6+d4*sin(aa)/sin(2*aa))*ex]).T
        joint_type = np.array([1,1,0,1,0,0,0,0,0,0])
        joint_upper_limit = np.append([10000,10000,10000,0.5],np.radians([10000,130,180,10000,10000,10000]))
        joint_lower_limit = np.append([-10000,-10000,-10000,0],np.radians([-10000,-130,-71,-10000,-10000,-10000]))
        
        # decalre robot
        self.bot = rox.Robot(H,P,joint_type,joint_lower_limit,joint_upper_limit)
        self.q_zeros = np.array([0,0,0,0,pi,pi,pi/2,0,0,0])

        # opt param
        self._ep = 0.01
        self._er = 0.02
        self._n = 10

    def fwdkin(self,q):
        
        # foward kinematics for oarbot
        return rox.fwdkin(self.bot,q)
    
    def jacobian(self,q):

        # jacobian matrix for oarbot
        return rox.robotjacobian(self.bot,q)

    def invkin(self, end_T, init_guess):

        alpha = 1
        Kp = 1
        
        robot_i = self.bot
        q = init_guess

        # start qp
        now_T = self.fwdkin(q)
        dX = np.reshape(np.append(self.s_err(now_T.R*end_T.R.T,2), now_T.p-end_T.p),(6,1))
        umax = (self.bot.joint_upper_limit-q)/alpha
        umin = (self.bot.joint_lower_limit-q)/alpha

        upper = np.array([1,1,2*pi,1,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi])

        umax = np.multiply((umax>upper),upper)+np.multiply((umax<=upper),umax)
        umin = np.multiply((umin<-upper),-upper)+np.multiply((umin>=-upper),umin)

        A = np.vstack((-np.eye(10), np.eye(10)))
        # b = np.reshape(np.append(-umax, umin),(20,))
        b = np.reshape(np.append(-umax, umin),(20,))
        J = self.jacobian(q)
        # H = np.matmul(J.T,J) + 0.00001*np.eye(10) # make sure the matrix is positive definite.
        H = np.matmul(J.T,J) + 0.0000001*np.eye(10)
        # H = (H+H.T)/2
        f = Kp*np.matmul(J.T,dX).flatten()

        # H = self.getqp_H(J,dX[0:3],dX[3:6])
        # H = 0.5*(H+H.T)+0.0000001*np.eye(self._n+2)
        # f = self.getqp_f()
        # f = f.reshape((self._n+2,))
        # umax = np.append(np.multiply((umax>upper),upper)+np.multiply((umax<=upper),umax),[1,1])
        # umin = np.append(np.multiply((umin<-upper),-upper)+np.multiply((umin>=-upper),umin),[-1,-1])
        # A = np.vstack((-np.eye(12), np.eye(12)))
        # b = np.reshape(np.append(-umax, umin),(24,))

        sc = norm(H,'fro')
        qp_sln = qp.solve_qp(H/sc, -f/sc, A.T, b)[0]
        q = q+alpha*qp_sln[0:self._n]

        return q
    
    def inequality_bound(self, h):
        sigma = np.zeros((h.shape))
        h2 = h - self._eta
        sigma[np.array(h2 >= self._epsilon)] = -np.tan(self._c*np.pi/2)
        sigma[np.array(h2 >= 0) & np.array(h2 < self._epsilon)] = -np.tan(self._c*np.pi/2/self._epsilon*h2[np.array(h2 >= 0) & np.array(h2 < self._epsilon)])
        sigma[np.array(h >= 0) & np.array(h2 < 0)] = -self._E*h2[np.array(h >= 0) & np.array(h2 < 0)]/self._eta
        sigma[np.array(h < 0)] = self._E

        return sigma

    def getqp_f(self):
        f = -2*np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, self._er, self._ep]).reshape(self._n+2, 1)

        return f

    def getqp_H(self, J, vr, vp):
        H1 = np.dot(np.hstack((J,np.zeros((6,2)))).T,np.hstack((J,np.zeros((6,2)))))

        tmp = np.vstack((np.hstack((np.hstack((np.zeros((3, self._n)),vr)),np.zeros((3,1)))),np.hstack((np.hstack((np.zeros((3,self._n)),np.zeros((3,1)))),vp)))) 
        H2 = np.dot(tmp.T,tmp)

        H3 = -2*np.dot(np.hstack((J,np.zeros((6,2)))).T, tmp)
        H3 = (H3+H3.T)/2

        tmp2 = np.vstack((np.array([0,0,0,0,0,0,0,0,0,0,np.sqrt(self._er),0]),np.array([0,0,0,0,0,0,0,0,0,0,0,np.sqrt(self._ep)])))
        H4 = np.dot(tmp2.T, tmp2)

        H = 2*(H1+H2+H3+H4)

        return H

    def s_err(self,er_mat,type):
        
        qua = rox.R2q(er_mat)
        qk = rox.R2rot(er_mat)
        if type == 1:
            err = 4*qua[0]*qua[1:4]
        elif type == 2:
            err = 2*qua[1:4]
        elif type == 3:
            err = 2*qk[1]*qk[1:4]
        else:
            raise ValueError("Type = 1 or 2 or 3")
        
        return err
