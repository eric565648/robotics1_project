import numpy as np
import general_robotics_toolbox as rox
from math import sin, cos, pi, fabs, atan2

### read param and change r and theta to real finaly pose
param=np.genfromtxt('/media/eric/Transcend/motion_lib/motion_data/data_param.csv',delimiter=',',dtype=float)

ex = np.array([1,0,0])
ey = np.array([0,1,0])
ez = np.array([0,0,1])
d1 = 0.2755
d2, d3, e2, d4, d5, d6= 0.41, 0.2073, 0.0098, 0.0741, 0.0741, 0.16
aa = pi/6
p89x = d4*(sin(aa)/sin(2*aa)) + 2*d4*(sin(aa)/sin(2*aa))*cos(2*aa)
p89y = 2*d4*sin(aa)
H_arm = np.array([ez,ey,-ey,-ex,-sin(pi/6)*ex+cos(pi/6)*ey,-ex]).T
P_arm = np.array([d1*ez,0*ez,d2*ez+e2*ey,0*ex,(p89x+d3)*ex-p89y*ey,0*ex,(d6+d4*sin(aa)/sin(2*aa))*ex]).T
joint_type_arm = np.array([0,0,0,0,0,0])        
arm_bot = rox.Robot(H_arm,P_arm,joint_type_arm)

def fwdkin(q):
    global arm_bot

    T = rox.Transform(rox.rot(ex,0),[0,0,0])
    for i in range(5):
        T = T*rox.Transform(rox.rot(arm_bot.H[:,[i]],q[i+4]),arm_bot.P[:,[i]])
    
    # print(T)
    # print(T.p-arm_bot.P[:,0])
    return T.p-arm_bot.P[:,0]

max_x=0
max_z=0
count_500=0
count_not_500=0
changed = 0
for i in range(1000):
    test_id=i+1
    qpath=np.genfromtxt('/media/eric/Transcend/motion_lib/motion_data/'+str(test_id)+'.csv',delimiter=',',dtype=float)
    fwdp = fwdkin(qpath[:,-1])
    # print("qpath:",qpath[:,0])
    # print("fwdp:",fwdp)
    # print("param:",param[test_id-1,:])
    # print("rtp:",param[test_id-1,0]*cos(param[test_id-1,1]),param[test_id-1,0]*sin(param[test_id-1,1]))
    rtp = np.array([param[test_id-1,5]*cos(param[test_id-1,6]),0,param[test_id-1,5]*sin(param[test_id-1,6])])
    
    r_fwdp = np.linalg.norm([fwdp[0],fwdp[2]])
    th_fwdp = atan2(fwdp[2],fwdp[0])
    # print("fwdp,r theta:",r_fwdp,th_fwdp)
    # print("rtp,r theta:",param[test_id-1,0],param[test_id-1,1])

    # change r theta from here
    if fabs(rtp[0]-fwdp[0])>0.11 or fabs(rtp[2]-fwdp[2])>0.11:

        if len(qpath[0,:])<499:
            count_not_500 += 1
            # print("testid:",test_id)
            # print("len q",len(qpath[0,:]))
            # print("displacement x z:",fabs(rtp[0]-fwdp[0]),fabs(rtp[2]-fwdp[2]))
            # print("===================")
        else: # larger than 499 steps (which the path was cut off)
            count_500 += 1
            # print("testid:",test_id)
            # print(param[test_id-1,:])
            param[test_id-1,5] = r_fwdp
            param[test_id-1,6] = th_fwdp
            # print(param[test_id-1,:])
            # print("======================")
    
    # if fabs(rtp[0]-fwdp[0])>max_x:
    #     max_x=fabs(rtp[0]-fwdp[0])
    #     print("testid:",test_id)
    #     print("len q",len(qpath[0,:]))
    #     print("maxx change:",max_x)
    #     print("=========")
    # if fabs(rtp[2]-fwdp[2])>max_z:
    #     max_z=fabs(rtp[2]-fwdp[2])
    #     print("testid:",test_id)
    #     print("len q",len(qpath[0,:]))
    #     print("maxz change:",max_z)
    #     print("==========")

print("500:",count_500)
print("not 500:",count_not_500)

np.savetxt('/media/eric/Transcend/motion_lib/motion_data/data_param_new.csv', param, delimiter=',')
