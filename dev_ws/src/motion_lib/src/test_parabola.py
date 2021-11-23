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
from math import pi, cos, sin


class Motion(object):
    def __init__(self) -> None:
        super().__init__()

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

        # service server
        self.motion_srv = rospy.Service('motions',Trigger,self.motion_srv_cb)
        self.home_srv = rospy.Service('go_home',Trigger,self.home_srv_cb)

        rospy.sleep(1)
        # go home
        self.go_home()

    def motion_srv_cb(self, req):

        path = self.create_path()
        self.viz_path(path)

        path_q = np.zeros((10,len(path)))
        for i in range(len(path)):
            if i == 0:
                init_guess = self.now_q
            else:
                init_guess = path_q[:,i-1]
            
            q = self.oarbot.invkin(path[i],init_guess)
            path_q[:,i] = q
            self.robot_fwd(q)
            rospy.sleep(0.1)

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

    def create_path(self):

        x = [0.8, 3.5, 6.2]
        z = [1.11,1.5,1.11]
        p = np.polyfit(x,z,2)
        x_end = 7.5; z_end=p[0]*x_end*x_end + p[1]*x_end + p[2]
        N = 100
        S_length = N+1
        p_Sx = np.arange(x[0],x_end,(x_end-x[0])/N)
        p_Sz = p[0]*p_Sx.T*p_Sx + p[1]*p_Sx + p[2]
        p_S = np.array([p_Sx,p_Sz])
        
        # make the path equal length
        diff = np.linalg.norm(np.diff(p_S),2,0)
        ls = np.append([0],np.cumsum(diff))
        lf = np.sum(diff)
        l = np.arange(0,lf+1e-5,lf/N) # plus a tiny number to include the last point

        lx = np.interp(l,ls,p_S[0])
        lz = np.interp(l,ls,p_S[1])
        p_Ts = np.array([lx,np.zeros(len(lx)),lz])
        T_path = np.array([])
        for i in range(S_length):
            eyt = np.array([0,1,0])

            if i == S_length-1:
                ext = p_Ts[:,i] - p_Ts[:,i-1]
            else:
                ext = p_Ts[:,i+1] - p_Ts[:,i]
            pass
            ext = ext - np.dot(ext,eyt)*eyt
            ext = ext/np.linalg.norm(ext,2)
            ezt = np.cross(ext,eyt)
            R = np.array([ext,eyt,ezt]).T
            T = rox.Transform(R,p_Ts[:,i])
            T_path = np.append(T_path,T)
        
        return T_path

    def viz_path(self, path):
        msg_markers = MarkerArray()
        for i in np.arange(1,len(path),5):
            m = Marker()
            m.header.frame_id='world'
            m.ns = str(i)
            m.id = 0
            m.action = 0
            m.type = 0
            m.color.a = 1
            m.color.r = 1
            m.scale.x = 0.2
            m.scale.y = 0.05
            m.scale.z = 0.05
            m.pose.position.x = path[i].p[0]
            m.pose.position.z = path[i].p[2]
            qua = rox.R2q(path[i].R)
            m.pose.orientation.w = qua[0]
            m.pose.orientation.x = qua[1]
            m.pose.orientation.y = qua[2]
            m.pose.orientation.z = qua[3]
            msg_markers.markers.append(m)
        
        self.path_pub.publish(msg_markers)

def main():

    rospy.init_node('motion_example')
    mo = Motion()

    rospy.spin()


if __name__ == '__main__':
    main()