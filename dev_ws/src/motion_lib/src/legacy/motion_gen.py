#!/usr/bin/env python3
import rospy
import general_robotics_toolbox as rox
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from oarbot_msgs.msg import OarbotJointState
from oarbot_moveit.oarbot_moveit import Oarbot
from std_srvs.srv import Trigger, TriggerResponse
from math import pi, cos, sin, radians
from random import uniform

class Motion(object):
    def __init__(self) -> None:
        super().__init__()

        # parameters
        self.c_rate = 0.1
        self.joint_vel_limit = np.deg2rad(np.array([36,36,36,48,48,48]))
        self.motion_T = 5
        self._n = 10

        # variables
        self.oarbot = Oarbot()
        self.joint_msg = None
        self.home_q = np.array([0,0,0,0,0,0,0,pi/6,pi/6,pi/6])
        self.now_q = self.home_q

        # publisher-subscriber
        self.joint_sub = rospy.Subscriber("joint_states",JointState,self.joint_state_cb,queue_size=1)
        self.fwd_sub = rospy.Subscriber("oarbot_joints",OarbotJointState,self.robot_fwd_cb,queue_size=1)
        
        self.joint_pub = rospy.Publisher("to_joint_states",JointState,queue_size=1)
        self.base_pose_pub = rospy.Publisher("oarbot_base_pose",PoseStamped,queue_size=1)
        self.path_pub = rospy.Publisher("path_markers",MarkerArray,queue_size=1)
        self.pathd_pub = rospy.Publisher("pathd_markers",MarkerArray,queue_size=1)

        # service server
        self.motion_srv = rospy.Service('motions',Trigger,self.motion_srv_cb)
        self.home_srv = rospy.Service('go_home',Trigger,self.home_srv_cb)

        rospy.sleep(1)
        # go home
        self.go_home()

    def motion_srv_cb(self, req):

        sr,stheta,sp,sy,sh = uniform(0.35,0.95),uniform(radians(-60),radians(70)),uniform(radians(-90),radians(90)),uniform(radians(-90),radians(90)),uniform(0,0.5)
        er,etheta,ep,ey,eh = uniform(0.35,0.95),uniform(radians(-60),radians(70)),uniform(radians(-90),radians(90)),uniform(radians(-90),radians(90)),uniform(0,0.5)
        exp_rate,exp_r = 0.1,0.1
        vel_type = 1
        arm_speed,base_speed = 1,1

        param = np.array([sr,stheta,sp,sy,sh,er,etheta,ep,ey,eh,exp_rate,exp_r,vel_type,arm_speed,base_speed])

        path = np.zeros((10,100))
        s1 = -0.54
        s2 = 1.88
        e1 = 0.5
        e2 = -1.66
        for i in range(100):
            path[:,i] = np.array([0,0,0,0,0,0,0,(e1-s1)*i/100+s1,(e2-s2)*i/100+s2,0])
            

        # path = self.create_path(param)
        # self.viz_path(path)

        for i in range(len(path[1,:])):
            self.robot_fwd(np.reshape(path[:,i],(self._n,)))
            rospy.sleep(self.c_rate)

        return TriggerResponse(
            success=True,
            message="Motion Complete"
        )
    
    def home_srv_cb(self, req):
        self.go_home()

        return TriggerResponse(
            success=True,
            message="Go home"
        )
    def go_home(self):
        self.robot_fwd(self.home_q)

    def robot_fwd_cb(self,msg):

        self.robot_fwd(msg.position)

    def robot_fwd(self,q):
        
        if self.joint_msg is None:
            return
        
        if len(q) != 10:
            rospy.logwarn("Oarbot has 10 joint. Ignoring this msg.")
            return
        
        try:
            # update oarbot fwdkin
            end_T = self.oarbot.fwdkin(q)
            
            # update joint state
            joint_msg = self.joint_msg
            joint_array = np.asarray(joint_msg.position)
            joint_array[0] = q[3]
            joint_array[5:11] = q[4:10]+self.oarbot.q_zeros[4:10]
            joint_msg.position = tuple(joint_array)
            base_msg = PoseStamped()
            base_msg.pose.position.x = q[0]
            base_msg.pose.position.y = q[1]
            qua = rox.rot2q(self.oarbot.bot.H[:,2],q[2])
            base_msg.pose.orientation.w = qua[0]
            base_msg.pose.orientation.x = qua[1]
            base_msg.pose.orientation.y = qua[2]
            base_msg.pose.orientation.z = qua[3]
            joint_msg.header.stamp = rospy.Time.now()
            base_msg.header.stamp = joint_msg.header.stamp
            self.joint_pub.publish(joint_msg)
            self.base_pose_pub.publish(base_msg)

            self.now_q = q

        except Exception as e:
            rospy.logwarn(e)
            return

    def joint_state_cb(self, msg):
        
        self.joint_msg = msg

    def create_path(self,param):

        sr,stheta,sp,sy,sh = param[0:5]
        er,etheta,ep,ey,eh = param[5:10]
        exp_rate,exp_r = param[10:12]
        vel_type = param[12]
        arm_speed,base_speed = param[13:15]

        print(param)

        N = 250 # TODO: should related to velocity

        spoint = np.array([sr*cos(stheta),0,sr*sin(stheta)])
        srpy = np.array([0,sp,sy])
        epoint = np.array([er*cos(etheta),0,er*sin(etheta)])
        erpy = np.array([0,ep,ey])
        sT = rox.fwdkin(self.oarbot.arm_bot,self.home_q[4:])
        sspoint = sT.p
        ssrpy = np.array(rox.R2rpy(sT.R))

        # ebx = base_speed*self.motion_T
        ebx = 0
        eh,sh = 0.01,0.01

        path = np.zeros((self._n,2*N))
        path[:,-1] = self.home_q
        path_d = np.array([])
        for i in range(N):
            arm_p = (spoint-sspoint)*i/N+sspoint
            # TODO: add exploration
            arm_rpy = (srpy-ssrpy)*i/N+ssrpy
            arm_T = rox.Transform(rox.rpy2R(arm_rpy),arm_p)

            # qd = np.radians(np.array([uniform(-180,180),uniform(-130,130),uniform(-71,90),uniform(-180,180),uniform(-180,180),uniform(-180,180)]))
            # arm_T = rox.fwdkin(self.oarbot.arm_bot,qd)
            path_d = np.append(path_d,arm_T)
            # print("guess",path[4:,i-1])
            arm_q = self.oarbot.invkin_arm(arm_T,path[4:,i-1])
            # print("qd",qd)
            print("arm_q",arm_q)
            print("====")
            sup_h = (eh-sh)/N*i
            bx = ebx/N*i
            q = np.append([bx,0,0,sup_h],arm_q)
            # print("ans",arm_q)

            path[:,i] = q
        for i in range(N+1,2*N):
            arm_p = (epoint-spoint)*(i-N)/N+spoint
            # TODO: add exploration
            arm_rpy = (erpy-srpy)*(i-N)/N+srpy
            arm_T = rox.Transform(rox.rpy2R(arm_rpy),arm_p)

            # qd = np.radians(np.array([uniform(-180,180),uniform(-130,130),uniform(-71,90),uniform(-180,180),uniform(-180,180),uniform(-180,180)]))
            # arm_T = rox.fwdkin(self.oarbot.arm_bot,qd)
            path_d = np.append(path_d,arm_T)
            # print("guess",path[4:,i-1])
            arm_q = self.oarbot.invkin_arm(arm_T,path[4:,i-1])
            # print("qd",qd)
            print("arm_q",arm_q)
            print("====")
            sup_h = (eh-sh)/N*i
            bx = ebx/N*i
            q = np.append([bx,0,0,sup_h],arm_q)
            # print("ans",arm_q)

            path[:,i] = q
        
        self.viz_pathd(path_d)
        return path

    def viz_path(self, path):
        msg_markers = MarkerArray()
        print(path.shape)
        for i in np.arange(1,len(path[0]),10):
            T_now = self.oarbot.fwdkin(path[:,i])

            m = Marker()
            m.header.frame_id='world'
            m.ns = str(i)
            m.id = 0
            m.action = 0
            m.type = 0
            m.color.a = 1
            m.color.r = 1
            m.scale.x = 0.1
            m.scale.y = 0.01
            m.scale.z = 0.01
            m.pose.position.x = T_now.p[0]
            m.pose.position.y = T_now.p[1]
            m.pose.position.z = T_now.p[2]
            qua = rox.R2q(T_now.R)
            m.pose.orientation.w = qua[0]
            m.pose.orientation.x = qua[1]
            m.pose.orientation.y = qua[2]
            m.pose.orientation.z = qua[3]
            msg_markers.markers.append(m)
        
        self.path_pub.publish(msg_markers)

    def viz_pathd(self, path):
        msg_markers = MarkerArray()
        for i in np.arange(1,len(path),10):
            m = Marker()
            m.header.frame_id='j2n6s300_link_base'
            m.ns = str(i)
            m.id = 0
            m.action = 0
            m.type = 0
            m.color.a = 1
            m.color.g = 1
            m.scale.x = 0.1
            m.scale.y = 0.01
            m.scale.z = 0.01
            m.pose.position.x = path[i].p[0]
            m.pose.position.y = path[i].p[1]
            m.pose.position.z = path[i].p[2]
            qua = rox.R2q(path[i].R)
            m.pose.orientation.w = qua[0]
            m.pose.orientation.x = qua[1]
            m.pose.orientation.y = qua[2]
            m.pose.orientation.z = qua[3]
            msg_markers.markers.append(m)
        self.pathd_pub.publish(msg_markers)

def main():

    rospy.init_node('motion_example')
    mo = Motion()

    rospy.spin()


if __name__ == '__main__':
    main()