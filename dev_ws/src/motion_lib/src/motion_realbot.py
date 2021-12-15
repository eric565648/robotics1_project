#!/usr/bin/env python3
import rospy
import tf2_ros
import general_robotics_toolbox as rox
import numpy as np
from geometry_msgs.msg import PoseStamped,TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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
        self.s_rate=50
        self.s_rate_ = rospy.Rate(self.s_rate)
        self.wheel_r = 0.127
        self.hand_size=1.25

        self.dataN = rospy.get_param("data_num",5)
        self.file_folder = rospy.get_param("motion_folder","/media/eric/Transcend/motion_lib/happy_motion_50/")
        self.record = rospy.get_param("record",True)

        # variables
        self.oarbot = Oarbot()
        self.joint_msg = None
        self.home_q = np.array([0,0,0,0,0,0,0,pi/6,pi/6,pi/6,0])
        self.now_q = self.home_q
        self.br = tf2_ros.TransformBroadcaster()

        # publisher-subscriber
        self.joint_sub = rospy.Subscriber("joint_states",JointState,self.joint_state_cb,queue_size=1)
        self.fwd_sub = rospy.Subscriber("oarbot_joints",OarbotJointState,self.robot_fwd_cb,queue_size=1)
        
        self.joint_pub = rospy.Publisher("to_joint_states",JointState,queue_size=1)

        self.rec_trigger = rospy.Publisher("screen_record",String,queue_size=1)

        # service server
        self.motion_srv = rospy.Service('motions',Trigger,self.motion_srv_cb)
        self.home_srv = rospy.Service('go_home',Trigger,self.home_srv_cb)

        rospy.sleep(1)
        # go home
        self.go_home()

    def motion_srv_cb(self, req):

        dataN=5
        play_t=1

        for i in range(dataN):
            print("Path:",i+1)
            path=np.genfromtxt(self.file_folder+str(i+1)+'.csv',delimiter=',',dtype=float)
            
            # rospy.sleep(0.5)
            for t in range(play_t):
                for p in path.T:
                    self.robot_fwd(p)
                    self.s_rate_.sleep()

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
        
        if len(q) != 11:
            rospy.logwarn("Oarbot has 11 joint (including finger). Ignoring this msg.")
            return
        
        try:
            # update oarbot fwdkin
            end_T = self.oarbot.fwdkin(q[:-1])
            
            # update joint state
            joint_msg = self.joint_msg
            joint_array = np.asarray(joint_msg.position)
            joint_array[0] = q[0]
            joint_array[1] = q[3]
            joint_array[6:12] = q[4:10]+self.oarbot.q_zeros[4:10]
            joint_array[12] = q[10]*self.hand_size
            joint_array[14] = q[10]*self.hand_size
            joint_array[16] = q[10]*self.hand_size
            diffw = q[0]/self.wheel_r
            joint_array[2:6] = [diffw,diffw,diffw,diffw]

            joint_msg.position = tuple(joint_array)
            
            joint_msg.header.stamp = rospy.Time.now()
            # base_msg.header.stamp = joint_msg.header.stamp
            self.joint_pub.publish(joint_msg)

            self.now_q = q

        except Exception as e:
            rospy.logwarn(e)
            return

    def joint_state_cb(self, msg):
        
        self.joint_msg = msg

def main():

    rospy.init_node('motion_example')
    mo = Motion()

    rospy.spin()


if __name__ == '__main__':
    main()